//! Streaming LoRa demodulator.
//!
//! Samples arrive at `osr` times the LoRa bandwidth. The decoder works through
//! them in three states:
//!
//! 1. **Detect** dechirps overlapping windows and looks for a tone that stays in
//!    the same FFT bin across several symbols, which only a preamble does.
//! 2. **Scan** follows the preamble to its downchirps. An up-chirp peak sits at
//!    `cfo + timing` and a down-chirp peak at `cfo - timing`, so the two together
//!    separate the carrier offset from the symbol timing. The reference signal
//!    (the sync word and both chirp directions) settles the remaining half
//!    symbol ambiguity, and the preamble then refines both to a fraction of a
//!    bin.
//! 3. **Demod** reads the header and the payload one symbol at a time, advancing
//!    by a symbol period scaled for the clock error and tracking any remaining
//!    drift from where each FFT peak lands.
//!
//! A dongle's crystal can be tens of ppm out and clocks both its tuner and its
//! ADC, as a radio's does. So the sample clock error follows from the carrier
//! offset: `error = cfo / rf_frequency`. Without this, long frames at high
//! spreading factors would drift by many bins.

use super::coding::{self, Header};
use super::params::LoraParams;
use num_complex::{Complex, Complex32};
use rustfft::{Fft, FftPlanner};
use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};
use std::sync::Arc;

/// Preamble symbols kept behind the scan position for fine synchronisation.
const HISTORY_SYMBOLS: usize = 14;
/// Preamble windows averaged for the fine estimates.
const FINE_WINDOWS: usize = 6;
/// Per-symbol gain of the drift tracking loop.
const TRACK_GAIN: f64 = 0.08;

/// A decoded frame, good or bad.
#[derive(Debug, Clone)]
pub struct Frame {
    /// Position of the frame's first payload symbol, in decoder input samples.
    pub sample_index: u64,
    pub sf: u8,
    pub bw_hz: u32,
    pub cr: u8,
    pub has_crc: bool,
    pub implicit_header: bool,
    pub ldro: bool,
    pub sync_word: u8,
    pub payload: Vec<u8>,
    /// `None` if the frame carries no CRC to check.
    pub crc_ok: Option<bool>,
    /// Estimated signal to noise ratio in the LoRa bandwidth.
    pub snr_db: f32,
    /// Channel power relative to full scale.
    pub rssi_dbfs: f32,
    pub cfo_hz: f64,
    /// Clock error between the transmitter and receiver, from the carrier offset.
    pub clock_ppm: f64,
    /// Codewords repaired by the Hamming code.
    pub fec_corrected: u32,
    /// Codewords with detected but uncorrectable errors.
    pub fec_detected: u32,
}

/// What the decoder reports.
#[derive(Debug, Clone)]
pub enum DecoderEvent {
    Frame(Frame),
    /// A preamble with the right sync word whose header failed its checksum.
    HeaderError { snr_db: f32, cfo_hz: f64, reason: String },
    /// A preamble with a different sync word.
    SyncMismatch { found: u8, snr_db: f32, cfo_hz: f64 },
}

#[derive(Clone, Copy, PartialEq)]
enum Chirp {
    Up,
    Down,
}

#[derive(Default)]
struct Track {
    spectra: VecDeque<Vec<f32>>,
    sum: Vec<f32>,
}

struct Scan {
    /// Start of the next window, on the grid the preamble was detected on.
    window: i64,
    /// Fractional bin of the preamble peak on that grid.
    k_up: f64,
    /// Consecutive windows showing neither the preamble nor downchirps.
    misses: u32,
}

/// Synchronisation results carried from the preamble into the payload.
#[derive(Clone, Copy)]
struct Link {
    cfo_bins: f64,
    snr_db: f32,
    rssi_dbfs: f32,
    sync_word: u8,
}

struct Demod {
    link: Link,
    start: f64,
    step: f64,
    /// Offset tracked since synchronisation, in bins.
    drift: f64,
    symbols: Vec<u16>,
    /// `None` until the header is known.
    shape: Option<Header>,
    total: usize,
}

enum State {
    Detect,
    Scan(Scan),
    Demod(Box<Demod>),
}

pub struct Decoder {
    p: LoraParams,
    osr: usize,
    n: usize,
    l: usize,
    rf_hz: f64,
    up: Vec<Complex32>,
    down: Vec<Complex32>,
    fir: Vec<f32>,
    half: usize,
    fft: Arc<dyn Fft<f32>>,
    fft_buf: Vec<Complex32>,
    fft_scratch: Vec<Complex32>,
    power: Vec<f32>,
    rotated: Vec<Complex32>,
    buf: Vec<Complex32>,
    /// Absolute index of `buf[0]`.
    base: i64,
    state: State,
    scan: i64,
    tracks: [Track; 2],
    next_track: usize,
    accumulate: usize,
    detect_threshold: f32,
    down_threshold: f32,
    pub stats: DecoderStats,
}

/// Running counters, for the GUI.
#[derive(Debug, Clone, Copy, Default)]
pub struct DecoderStats {
    pub preambles: u64,
    pub sync_mismatches: u64,
    pub header_errors: u64,
    pub frames: u64,
    pub crc_failures: u64,
}

impl Decoder {
    /// `rf_hz` is the carrier frequency, used to infer the clock error.
    pub fn new(p: LoraParams, osr: usize, rf_hz: f64) -> Result<Self, String> {
        p.validate()?;
        let n = p.bins();
        let l = n * osr;
        let up: Vec<Complex32> = (0..n)
            .map(|i| {
                let i = i as f64;
                let phase = TAU * (i * i / (2.0 * n as f64) - i / 2.0);
                Complex32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect();
        let down: Vec<Complex32> = up.iter().map(|c| c.conj()).collect();

        // Narrow filter applied just before picking one sample per chip, once
        // any carrier offset has been removed. The channel filter ahead of the
        // decoder is wider so that it passes an offset carrier; without this
        // second step twice the noise would fold into the band.
        let half = if osr > 1 { 4 * osr } else { 0 };
        let fc = 0.55 / osr as f64;
        let mut fir: Vec<f32> = (0..=2 * half)
            .map(|i| {
                let x = i as f64 - half as f64;
                let sinc = if x == 0.0 { 2.0 * fc } else { (TAU * fc * x).sin() / (PI * x) };
                let w = 0.54 + 0.46 * (PI * x / (half as f64 + 1.0)).cos();
                (sinc * w) as f32
            })
            .collect();
        let gain: f32 = fir.iter().sum();
        fir.iter_mut().for_each(|v| *v /= gain);

        let fft = FftPlanner::new().plan_fft_forward(n);
        let accumulate = (p.preamble_len as usize).saturating_sub(4).clamp(2, 4);
        let sf = p.sf as f32;
        Ok(Self {
            osr,
            n,
            l,
            rf_hz,
            up,
            down,
            fir,
            half,
            fft_buf: vec![Complex32::default(); n],
            fft_scratch: vec![Complex32::default(); fft.get_inplace_scratch_len()],
            fft,
            power: vec![0.0; n],
            rotated: vec![Complex32::default(); l + 2 * half + 1],
            buf: Vec::with_capacity(1 << 20),
            base: 0,
            state: State::Detect,
            scan: half as i64,
            tracks: Default::default(),
            next_track: 0,
            accumulate,
            detect_threshold: 3.5 + 0.25 * sf,
            down_threshold: 3.5 + 0.4 * sf,
            stats: DecoderStats::default(),
            p,
        })
    }

    pub fn params(&self) -> &LoraParams {
        &self.p
    }

    /// Feeds samples and reports anything completed by them.
    pub fn push(&mut self, samples: &[Complex32], events: &mut Vec<DecoderEvent>) {
        if self.p.invert_iq {
            self.buf.extend(samples.iter().map(|s| s.conj()));
        } else {
            self.buf.extend_from_slice(samples);
        }
        while self.step(events) {}
        self.trim();
    }

    /// Flushes a frame ending exactly at the end of a recording.
    pub fn finish(&mut self, events: &mut Vec<DecoderEvent>) {
        let silence = vec![Complex32::default(); self.l * 2 + 2 * self.half + 2];
        self.push(&silence, events);
    }

    fn end(&self) -> i64 {
        self.base + self.buf.len() as i64
    }

    fn available(&self, start: i64) -> bool {
        start - self.half as i64 >= self.base && start + (self.l + self.half + 1) as i64 <= self.end()
    }

    fn trim(&mut self) {
        let position = match &self.state {
            State::Detect => self.scan,
            State::Scan(s) => s.window,
            State::Demod(d) => (d.start + d.symbols.len() as f64 * d.step) as i64,
        };
        let keep_from = position - (HISTORY_SYMBOLS * self.l + self.half) as i64;
        let drop = keep_from - self.base;
        if drop > (64 * self.l).max(1 << 18) as i64 {
            self.buf.drain(..drop as usize);
            self.base = keep_from;
        }
    }

    /// Dechirps one symbol window into `fft_buf`, in the frequency domain.
    /// `freq` is removed first, in bins.
    fn window(&mut self, start: i64, freq: f64, chirp: Chirp) {
        let h = self.half;
        let count = self.l + 2 * h + 1;
        let first = (start - h as i64 - self.base) as usize;
        let omega = -TAU * freq / self.l as f64;
        let step = Complex::<f64>::cis(omega);
        let mut rotor = Complex::<f64>::cis(-omega * h as f64);
        for (i, (out, input)) in self
            .rotated
            .iter_mut()
            .zip(&self.buf[first..first + count])
            .enumerate()
        {
            *out = Complex32::new(
                (input.re as f64 * rotor.re - input.im as f64 * rotor.im) as f32,
                (input.re as f64 * rotor.im + input.im as f64 * rotor.re) as f32,
            );
            rotor *= step;
            if i & 1023 == 1023 {
                rotor /= rotor.norm();
            }
        }
        let reference = if chirp == Chirp::Up { &self.down } else { &self.up };
        for (m, (out, chip)) in self.fft_buf.iter_mut().zip(reference).enumerate() {
            let at = m * self.osr;
            let mut acc = Complex32::default();
            for (tap, s) in self.fir.iter().zip(&self.rotated[at..at + 2 * h + 1]) {
                acc.re += tap * s.re;
                acc.im += tap * s.im;
            }
            *out = acc * chip;
        }
        self.fft.process_with_scratch(&mut self.fft_buf, &mut self.fft_scratch);
    }

    fn update_power(&mut self) {
        for (p, x) in self.power.iter_mut().zip(&self.fft_buf) {
            *p = x.norm_sqr();
        }
    }

    /// Peak bin, its power, and the mean power of the other bins.
    fn peak(&self) -> (usize, f32, f32) {
        let mut best = 0;
        let mut top = -1.0f32;
        let mut total = 0.0f32;
        for (i, &p) in self.power.iter().enumerate() {
            total += p;
            if p > top {
                top = p;
                best = i;
            }
        }
        let mean = (total - top) / (self.n - 1) as f32;
        (best, top, mean.max(1e-20))
    }

    fn power_near(&self, bin: usize) -> f32 {
        let n = self.n;
        self.power[bin % n]
            .max(self.power[(bin + 1) % n])
            .max(self.power[(bin + n - 1) % n])
    }

    /// Position of the peak near `bin` to a fraction of a bin (Jacobsen).
    fn interpolate(&self, bin: usize) -> f64 {
        let n = self.n;
        let lo = self.fft_buf[(bin + n - 1) % n];
        let mid = self.fft_buf[bin];
        let hi = self.fft_buf[(bin + 1) % n];
        let denom = mid * 2.0 - lo - hi;
        if denom.norm_sqr() < 1e-30 {
            return 0.0;
        }
        (-(hi - lo) / denom).re.clamp(-0.5, 0.5) as f64
    }

    fn step(&mut self, events: &mut Vec<DecoderEvent>) -> bool {
        match self.state {
            State::Detect => self.step_detect(),
            State::Scan(_) => self.step_scan(events),
            State::Demod(_) => self.step_demod(events),
        }
    }

    fn step_detect(&mut self) -> bool {
        if !self.available(self.scan) {
            return false;
        }
        let start = self.scan;
        self.window(start, 0.0, Chirp::Up);
        self.update_power();
        let track_index = self.next_track;
        self.next_track ^= 1;
        self.scan += (self.l / 2) as i64;

        let track = &mut self.tracks[track_index];
        if track.sum.is_empty() {
            track.sum = vec![0.0; self.n];
        }
        if track.spectra.len() == self.accumulate {
            let oldest = track.spectra.pop_front().unwrap();
            track.sum.iter_mut().zip(&oldest).for_each(|(s, o)| *s -= o);
        }
        track.sum.iter_mut().zip(&self.power).for_each(|(s, p)| *s += p);
        track.spectra.push_back(self.power.clone());
        if track.spectra.len() < self.accumulate {
            return true;
        }

        let (mut best, mut top, mut total) = (0, -1.0f32, 0.0f32);
        for (i, &v) in track.sum.iter().enumerate() {
            total += v;
            if v > top {
                top = v;
                best = i;
            }
        }
        let mean = ((total - top) / (self.n - 1) as f32).max(1e-20);
        if top / mean < self.detect_threshold {
            return true;
        }
        // A strong data symbol can win the sum on its own, so every window must
        // show the tone.
        let n = self.n;
        let steady = track.spectra.iter().all(|spec| {
            let local = spec[best].max(spec[(best + 1) % n]).max(spec[(best + n - 1) % n]);
            let m = spec.iter().sum::<f32>() / n as f32;
            local > 3.0 * m
        });
        if !steady {
            return true;
        }

        let sum = &track.sum;
        let (a, b, c) = (
            sum[(best + n - 1) % n].sqrt() as f64,
            sum[best].sqrt() as f64,
            sum[(best + 1) % n].sqrt() as f64,
        );
        let denom = a - 2.0 * b + c;
        let frac = if denom.abs() > 1e-12 { (0.5 * (a - c) / denom).clamp(-0.5, 0.5) } else { 0.0 };
        self.stats.preambles += 1;
        self.state = State::Scan(Scan {
            window: start + self.l as i64,
            k_up: best as f64 + frac,
            misses: 0,
        });
        true
    }

    fn back_to_detect(&mut self, from: i64) {
        self.scan = from.max(self.base + self.half as i64);
        self.state = State::Detect;
        for t in &mut self.tracks {
            t.spectra.clear();
            t.sum.clear();
        }
    }

    fn step_scan(&mut self, events: &mut Vec<DecoderEvent>) -> bool {
        let (window, k_up) = match &self.state {
            State::Scan(s) => (s.window, s.k_up),
            _ => unreachable!(),
        };
        // Synchronising needs the rest of the downchirps too.
        if !self.available(window + 3 * self.l as i64) {
            return false;
        }

        self.window(window, 0.0, Chirp::Up);
        self.update_power();
        let up_bin = k_up.round() as usize % self.n;
        let up_power = self.power_near(up_bin);
        let up_mean = self.power.iter().sum::<f32>() / self.n as f32;

        // Downchirps fill at least two successive windows, which rejects the
        // chance peaks that noise produces in any one window.
        self.window(window, 0.0, Chirp::Down);
        self.update_power();
        let first = self.power.clone();
        self.window(window + self.l as i64, 0.0, Chirp::Down);
        self.update_power();
        self.power.iter_mut().zip(&first).for_each(|(p, f)| *p += f);
        let (down_bin, down_power, down_mean) = self.peak();

        if down_power / down_mean >= self.down_threshold && down_power > 2.0 * up_power {
            let n = self.n;
            let (a, b, c) = (
                self.power[(down_bin + n - 1) % n].sqrt() as f64,
                self.power[down_bin].sqrt() as f64,
                self.power[(down_bin + 1) % n].sqrt() as f64,
            );
            let denom = a - 2.0 * b + c;
            let frac = if denom.abs() > 1e-12 { (0.5 * (a - c) / denom).clamp(-0.5, 0.5) } else { 0.0 };
            if self.synchronise(window, k_up, down_bin as f64 + frac, events) {
                return true;
            }
        }

        let preamble_continues = up_power > 4.0 * up_mean;
        let l = self.l as i64;
        let State::Scan(scan) = &mut self.state else { unreachable!() };
        scan.window += l;
        scan.misses = if preamble_continues { 0 } else { scan.misses + 1 };
        if scan.misses > 4 {
            let resume = scan.window;
            self.back_to_detect(resume);
        }
        true
    }

    /// Sum of the expected tones if the downchirps start at `t` with offset `cfo`.
    fn alignment_score(&mut self, t: i64, cfo: f64) -> f32 {
        let l = self.l as i64;
        if !self.available(t - 3 * l) || !self.available(t + l) {
            return 0.0;
        }
        let (s1, s2) = self.p.sync_symbols();
        let any = self.p.any_sync_word;
        let mut score = 0.0;
        for (offset, chirp, bin) in [
            (-3, Chirp::Up, Some(0)),
            (-2, Chirp::Up, (!any).then_some(s1)),
            (-1, Chirp::Up, (!any).then_some(s2)),
            (0, Chirp::Down, Some(0)),
            (1, Chirp::Down, Some(0)),
        ] {
            self.window(t + offset * l, cfo, chirp);
            self.update_power();
            score += match bin {
                Some(b) => self.power_near(b),
                None => self.peak().1,
            };
        }
        score
    }

    /// Tries to lock onto a frame given the two peaks. Returns false if this was
    /// not a real set of downchirps, in which case the scan carries on.
    fn synchronise(&mut self, window: i64, k_up: f64, k_down: f64, events: &mut Vec<DecoderEvent>) -> bool {
        let n = self.n as f64;
        let wrap = |x: f64, m: f64| x.rem_euclid(m);
        let centred = |x: f64| {
            let v = wrap(x, n);
            if v >= n / 2.0 {
                v - n
            } else {
                v
            }
        };

        // `k_up = cfo + timing` and `k_down = cfo - timing` determine both only
        // modulo half a symbol, so test each reading and each nearby symbol.
        let base_cfo = {
            let mut sum = k_up + k_down;
            if (k_up - k_down).abs() > n / 2.0 {
                sum += n;
            }
            sum / 2.0
        };
        let l = self.l as i64;
        let mut best = (0.0f32, 0i64, 0.0f64);
        for hypothesis in 0..2 {
            let cfo = centred(base_cfo + hypothesis as f64 * n / 2.0);
            let timing = wrap(k_up - cfo, n);
            let t0 = window - (timing * self.osr as f64).round() as i64;
            for m in -1..=2 {
                let t = t0 + m * l;
                let score = self.alignment_score(t, cfo);
                if score > best.0 {
                    best = (score, t, cfo);
                }
            }
        }
        let (score, t, cfo) = best;
        if score <= 0.0 {
            return false;
        }

        let (t, cfo, snr_db, rssi_dbfs) = self.refine(t, cfo);
        let exact_t = t;
        let t = t.round() as i64;
        let cfo_hz = cfo * self.p.bw() / n;

        // Read the sync word at the refined alignment.
        self.window(t - 2 * l, cfo, Chirp::Up);
        self.update_power();
        let (s1, p1, m1) = self.peak();
        let power_s1 = self.power.clone();
        self.window(t - l, cfo, Chirp::Up);
        self.update_power();
        let (s2, p2, m2) = self.peak();
        let power_s2 = self.power.clone();
        self.window(t, cfo, Chirp::Down);
        self.update_power();
        let down_ok = self.power_near(0) / self.power.iter().sum::<f32>() * n as f32 > 3.0;

        let found = (((s1 + 4) / 8) as u8 & 0x0f) << 4 | (((s2 + 4) / 8) as u8 & 0x0f);
        if !down_ok {
            return false;
        }
        let (e1, e2) = self.p.sync_symbols();
        let near = |a: usize, b: usize| {
            let d = (a + self.n - b) % self.n;
            d.min(self.n - d) <= 2
        };
        // A noise peak can beat a weak sync symbol, so accept the expected tone
        // being clearly present even if it is not the strongest.
        let expected_1 = near(s1, e1) || power_s1[e1] > 6.0 * m1;
        let expected_2 = near(s2, e2) || power_s2[e2] > 6.0 * m2;
        if !self.p.any_sync_word && !(expected_1 && expected_2) {
            self.stats.sync_mismatches += 1;
            events.push(DecoderEvent::SyncMismatch { found, snr_db, cfo_hz });
            self.back_to_detect(t + l);
            return true;
        }
        if self.p.any_sync_word && (p1 / m1 < 4.0 || p2 / m2 < 4.0) {
            return false;
        }

        let scale = 1.0 - self.clock_error(cfo);
        let step = self.l as f64 * scale;
        let link = Link {
            cfo_bins: cfo,
            snr_db,
            rssi_dbfs,
            sync_word: if self.p.any_sync_word { found } else { self.p.sync_word },
        };
        let (shape, total) = if self.p.implicit_header {
            let header = Header {
                payload_len: self.p.implicit_len,
                cr: self.p.cr,
                has_crc: self.p.implicit_crc,
            };
            let total = coding::frame_symbols(&self.p, header.payload_len as usize, header.cr, header.has_crc);
            (Some(header), total)
        } else {
            (None, 8)
        };
        self.state = State::Demod(Box::new(Demod {
            link,
            start: exact_t + 2.25 * step,
            step,
            drift: 0.0,
            symbols: Vec::with_capacity(total.max(64)),
            shape,
            total,
        }));
        true
    }

    /// Clock error as a fraction, positive when the transmitter runs fast.
    fn clock_error(&self, cfo_bins: f64) -> f64 {
        if self.rf_hz <= 0.0 {
            return 0.0;
        }
        let sign = if self.p.invert_iq { -1.0 } else { 1.0 };
        sign * cfo_bins * self.p.bw() / self.n as f64 / self.rf_hz
    }

    /// Position of the strongest tone in a chirp's windows, as a signed bin.
    fn tone_position(&mut self, starts: &[i64], cfo: f64, chirp: Chirp) -> f64 {
        let n = self.n;
        let mut sum = vec![0.0f32; n];
        let mut position = 0.0;
        for &start in starts {
            self.window(start, cfo, chirp);
            self.update_power();
            sum.iter_mut().zip(&self.power).for_each(|(s, p)| *s += p);
        }
        let bin = (0..n).max_by(|&a, &b| sum[a].total_cmp(&sum[b])).unwrap();
        for &start in starts {
            self.window(start, cfo, chirp);
            position += self.interpolate(bin);
        }
        let signed = if bin >= n / 2 { bin as f64 - n as f64 } else { bin as f64 };
        signed + position / starts.len() as f64
    }

    /// Refines the carrier offset and timing from the preamble, and measures it.
    /// Returns the downchirp start, the offset, the SNR and the power.
    fn refine(&mut self, t: i64, cfo: f64) -> (f64, f64, f32, f32) {
        let l = self.l as i64;
        let usable = (self.p.preamble_len as usize - 1).clamp(2, FINE_WINDOWS);

        // The first estimate came from windows straddling two chirps, where the
        // peak can land a bin or two out. Repeat it on aligned windows.
        let (mut t, mut cfo) = (t, cfo);
        for _ in 0..2 {
            let ups: Vec<i64> = (0..usable as i64).map(|j| t - (3 + j) * l).take_while(|&p| self.available(p)).collect();
            if ups.is_empty() || !self.available(t + l) {
                break;
            }
            let up = self.tone_position(&ups, cfo, Chirp::Up);
            let down = self.tone_position(&[t, t + l], cfo, Chirp::Down);
            if (up - down).abs() > 16.0 || (up + down).abs() > 16.0 {
                break;
            }
            cfo += (up + down) / 2.0;
            t -= ((up - down) / 2.0 * self.osr as f64).round() as i64;
        }

        let positions: Vec<i64> = (0..usable as i64)
            .map(|j| t - (3 + j) * l)
            .take_while(|&p| self.available(p))
            .collect();
        if positions.len() < 2 {
            return (t as f64, cfo, 0.0, -100.0);
        }

        // The tone advances by the carrier offset every symbol, which gives the
        // fractional part of that offset very precisely.
        let coarse = cfo.round();
        let mut sum = vec![0.0f32; self.n];
        let mut tones: Vec<Vec<Complex32>> = Vec::with_capacity(positions.len());
        for &p in &positions {
            self.window(p, coarse, Chirp::Up);
            self.update_power();
            sum.iter_mut().zip(&self.power).for_each(|(s, v)| *s += v);
            tones.push(self.fft_buf.clone());
        }
        let n = self.n;
        let bin = [0, 1, 2, 3, n - 3, n - 2, n - 1]
            .into_iter()
            .max_by(|&a, &b| sum[a].total_cmp(&sum[b]))
            .unwrap();
        let mut rotation = Complex::<f64>::default();
        for pair in tones.windows(2) {
            // Positions run backwards in time, so pair[0] is the later window.
            let later = pair[0][bin];
            let earlier = pair[1][bin];
            let z = later * earlier.conj();
            rotation += Complex::new(z.re as f64, z.im as f64);
        }
        let frac = rotation.arg() / TAU;
        let refined = frac + (cfo - frac).round();

        // With the offset known, whatever remains of the peak position is timing:
        // a window starting late moves an up-chirp's tone up.
        let mut timing = 0.0f64;
        for &p in &positions {
            self.window(p, refined, Chirp::Up);
            self.update_power();
            let near = [0, 1, n - 1]
                .into_iter()
                .max_by(|&a, &b| self.power[a].total_cmp(&self.power[b]))
                .unwrap();
            let signed = if near == n - 1 { -1.0 } else { near as f64 };
            timing += signed + self.interpolate(near);
        }
        let exact = t as f64 - timing / positions.len() as f64 * self.osr as f64;

        // Measure on the aligned preamble.
        let (mut snr, mut power, mut count) = (0.0f64, 0.0f64, 0.0f64);
        for j in 0..positions.len() as i64 {
            let ideal = exact - ((3 + j) * l) as f64;
            let start = ideal.round() as i64;
            if !self.available(start) {
                continue;
            }
            self.window(start, refined + (start as f64 - ideal) / self.osr as f64, Chirp::Up);
            self.update_power();
            let peak = self.power_near(0) as f64;
            let total: f64 = self.power.iter().map(|&v| v as f64).sum();
            let noise = ((total - peak) / (n - 1) as f64).max(1e-20);
            snr += (peak / noise - 1.0).max(1e-3) / n as f64;
            let first = (start - self.base) as usize;
            power += self.buf[first..first + self.l].iter().map(|v| v.norm_sqr() as f64).sum::<f64>()
                / self.l as f64;
            count += 1.0;
        }
        if count == 0.0 {
            return (exact, refined, 0.0, -100.0);
        }
        (
            exact,
            refined,
            10.0 * (snr / count).log10() as f32,
            10.0 * (power / count).max(1e-12).log10() as f32,
        )
    }

    fn step_demod(&mut self, events: &mut Vec<DecoderEvent>) -> bool {
        let (position, cfo, drift) = {
            let State::Demod(d) = &self.state else { unreachable!() };
            (d.start + d.symbols.len() as f64 * d.step, d.link.cfo_bins, d.drift)
        };
        let start = position.round() as i64;
        if !self.available(start) {
            return false;
        }
        // Starting late by a fraction of a sample shifts an up-chirp up in
        // frequency by the same fraction of a bin.
        let late = (start as f64 - position) / self.osr as f64;
        self.window(start, cfo + drift + late, Chirp::Up);
        self.update_power();
        let (bin, top, mean) = self.peak();
        let frac = self.interpolate(bin);

        let ldro = self.p.ldro_active();
        let State::Demod(d) = &mut self.state else { unreachable!() };
        let reduced = d.symbols.len() < 8 || ldro;
        let n = self.n as f64;
        let error = if reduced {
            // These symbols sit on a known grid, which measures larger offsets.
            let value = bin as f64 + frac - 1.0;
            let off = value - 4.0 * (value / 4.0).round();
            off.clamp(-1.5, 1.5)
        } else {
            frac
        };
        if top / mean > 6.0 {
            d.drift += TRACK_GAIN * error;
            if d.drift.abs() > n / 2.0 {
                d.drift = 0.0;
            }
        }
        d.symbols.push(bin as u16);

        if d.symbols.len() == 8 && d.shape.is_none() {
            let (header, _) = coding::decode_first_block(&self.p, &d.symbols);
            match header {
                Ok(Some(h)) => {
                    d.total = coding::frame_symbols(&self.p, h.payload_len as usize, h.cr, h.has_crc);
                    d.shape = Some(h);
                }
                Ok(None) => unreachable!("implicit frames start with a known shape"),
                Err(reason) => {
                    let (snr_db, cfo_bins) = (d.link.snr_db, d.link.cfo_bins);
                    let resume = (d.start + 8.0 * d.step) as i64;
                    self.stats.header_errors += 1;
                    events.push(DecoderEvent::HeaderError {
                        snr_db,
                        cfo_hz: cfo_bins * self.p.bw() / n,
                        reason,
                    });
                    self.back_to_detect(resume);
                    return true;
                }
            }
        }
        if d.symbols.len() < d.total {
            return true;
        }

        let h = d.shape.expect("the shape is known once all symbols are in");
        let out = coding::decode_payload(&self.p, &d.symbols, h.payload_len as usize, h.cr, h.has_crc);
        let link = d.link;
        let start_index = d.start.max(0.0) as u64;
        let resume = (d.start + d.total as f64 * d.step) as i64;
        let cfo_hz = link.cfo_bins * self.p.bw() / n;
        let frame = Frame {
            sample_index: start_index,
            sf: self.p.sf,
            bw_hz: self.p.bw_hz,
            cr: h.cr,
            has_crc: h.has_crc,
            implicit_header: self.p.implicit_header,
            ldro,
            sync_word: link.sync_word,
            payload: out.data,
            crc_ok: out.crc_ok,
            snr_db: link.snr_db,
            rssi_dbfs: link.rssi_dbfs,
            cfo_hz,
            clock_ppm: self.clock_error(link.cfo_bins) * 1e6,
            fec_corrected: out.corrected,
            fec_detected: out.detected,
        };
        self.stats.frames += 1;
        if frame.crc_ok == Some(false) {
            self.stats.crc_failures += 1;
        }
        events.push(DecoderEvent::Frame(frame));
        self.back_to_detect(resume);
        true
    }
}

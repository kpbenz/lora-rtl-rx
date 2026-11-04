//! Channel selection: moves the LoRa channel to DC, filters it, and converts
//! the sample rate to a whole multiple of the LoRa bandwidth.

use num_complex::{Complex, Complex32};
use std::f64::consts::{PI, TAU};

fn bessel_i0(x: f64) -> f64 {
    let (mut sum, mut term) = (1.0, 1.0);
    for k in 1..64 {
        let t = x / (2.0 * k as f64);
        term *= t * t;
        sum += term;
        if term < 1e-12 * sum {
            break;
        }
    }
    sum
}

fn kaiser(x: f64, half_width: f64, beta: f64) -> f64 {
    let r = x / half_width;
    if r.abs() >= 1.0 {
        return 0.0;
    }
    bessel_i0(beta * (1.0 - r * r).sqrt()) / bessel_i0(beta)
}

fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-9 {
        1.0
    } else {
        (PI * x).sin() / (PI * x)
    }
}

const STOPBAND_DB: f64 = 70.0;

/// Taps for a Kaiser low pass with a transition width `width` (of the rate).
fn taps_for(width: f64) -> usize {
    let taps = (STOPBAND_DB - 8.0) / (2.285 * TAU * width);
    (taps.ceil() as usize).max(5) | 1
}

fn kaiser_beta() -> f64 {
    0.1102 * (STOPBAND_DB - 8.7)
}

/// Integer decimating FIR.
struct Decimator {
    taps: Vec<f32>,
    factor: usize,
    history: Vec<Complex32>,
    position: usize,
}

impl Decimator {
    fn new(factor: usize, pass: f64, stop: f64) -> Self {
        let count = taps_for(stop - pass);
        let centre = (count - 1) as f64 / 2.0;
        let cut = (pass + stop) / 2.0;
        let mut taps: Vec<f32> = (0..count)
            .map(|i| {
                let x = i as f64 - centre;
                (2.0 * cut * sinc(2.0 * cut * x) * kaiser(x, centre + 1.0, kaiser_beta())) as f32
            })
            .collect();
        let gain: f32 = taps.iter().sum();
        taps.iter_mut().for_each(|t| *t /= gain);
        Self { taps, factor, history: Vec::new(), position: 0 }
    }

    fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        self.history.extend_from_slice(input);
        let m = self.taps.len();
        while self.position + m <= self.history.len() {
            let mut acc = Complex32::default();
            for (t, s) in self.taps.iter().zip(&self.history[self.position..self.position + m]) {
                acc.re += t * s.re;
                acc.im += t * s.im;
            }
            out.push(acc);
            self.position += self.factor;
        }
        let used = self.position.min(self.history.len());
        self.history.drain(..used);
        self.position -= used;
    }
}

/// Polyphase resampler for any rational or irrational ratio.
struct Resampler {
    phases: usize,
    taps: Vec<f32>,
    length: usize,
    ratio: f64,
    history: Vec<Complex32>,
    position: f64,
}

impl Resampler {
    /// `ratio` is input rate over output rate; `pass` and `stop` are fractions
    /// of the input rate.
    fn new(ratio: f64, pass: f64, stop: f64) -> Self {
        let length = taps_for(stop - pass);
        let phases = 1024;
        let centre = (length - 1) as f64 / 2.0;
        let cut = (pass + stop) / 2.0;
        let mut taps = Vec::with_capacity(length * (phases + 1));
        for p in 0..=phases {
            let frac = p as f64 / phases as f64;
            let mut row: Vec<f64> = (0..length)
                .map(|i| {
                    let x = i as f64 - centre - frac;
                    2.0 * cut * sinc(2.0 * cut * x) * kaiser(x, centre + 1.0, kaiser_beta())
                })
                .collect();
            let gain: f64 = row.iter().sum();
            row.iter_mut().for_each(|t| *t /= gain);
            taps.extend(row.into_iter().map(|t| t as f32));
        }
        Self { phases, taps, length, ratio, history: Vec::new(), position: 0.0 }
    }

    fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        self.history.extend_from_slice(input);
        loop {
            let whole = self.position.floor() as usize;
            if whole + self.length + 1 > self.history.len() {
                break;
            }
            let frac = self.position - whole as f64;
            let phase = (frac * self.phases as f64).round() as usize;
            let row = &self.taps[phase * self.length..(phase + 1) * self.length];
            let mut acc = Complex32::default();
            for (t, s) in row.iter().zip(&self.history[whole..whole + self.length]) {
                acc.re += t * s.re;
                acc.im += t * s.im;
            }
            out.push(acc);
            self.position += self.ratio;
        }
        let whole = (self.position.floor() as usize).min(self.history.len());
        self.history.drain(..whole);
        self.position -= whole as f64;
    }
}

/// The full chain from raw samples to the decoder's rate.
pub struct ChannelFilter {
    rotor: Complex<f64>,
    step: Complex<f64>,
    mixed: Vec<Complex32>,
    middle: Vec<Complex32>,
    decimator: Option<Decimator>,
    resampler: Resampler,
    counter: u32,
}

impl ChannelFilter {
    /// Oversampling used for a given input rate, or an error if it is too low.
    pub fn oversampling(input_rate: f64, bw: f64) -> Result<usize, String> {
        let ratio = input_rate / bw;
        if ratio >= 3.999 {
            Ok(4)
        } else if ratio >= 1.999 {
            Ok(2)
        } else {
            Err(format!(
                "a sample rate of {:.0} kHz is too low for {:.1} kHz LoRa: it needs at least twice the bandwidth",
                input_rate / 1e3,
                bw / 1e3
            ))
        }
    }

    /// `offset_hz` is where the channel sits relative to the tuned frequency.
    pub fn new(input_rate: f64, offset_hz: f64, bw: f64, osr: usize) -> Self {
        let output_rate = bw * osr as f64;
        // Pass as much carrier offset as the rate allows without aliasing.
        let pass_hz = bw.min(output_rate * 0.375);
        let stop_hz = output_rate - pass_hz;

        // Bring the rate down cheaply by a whole factor first, keeping at least
        // twice the output rate for the fractional stage.
        let factor = ((input_rate / output_rate / 2.0).floor() as usize).max(1);
        let middle_rate = input_rate / factor as f64;
        let decimator = (factor > 1).then(|| {
            Decimator::new(factor, pass_hz / input_rate, (middle_rate - pass_hz) / input_rate)
        });
        let ratio = middle_rate / output_rate;
        let resampler = Resampler::new(ratio, pass_hz / middle_rate, stop_hz.min(middle_rate / 2.0) / middle_rate);

        Self {
            rotor: Complex::new(1.0, 0.0),
            step: Complex::cis(-TAU * offset_hz / input_rate),
            mixed: Vec::new(),
            middle: Vec::new(),
            decimator,
            resampler,
            counter: 0,
        }
    }

    /// Filters `input`, appending to `output`.
    pub fn process(&mut self, input: &[Complex32], output: &mut Vec<Complex32>) {
        self.mixed.clear();
        self.mixed.reserve(input.len());
        for &s in input {
            let (re, im) = (s.re as f64, s.im as f64);
            self.mixed.push(Complex32::new(
                (re * self.rotor.re - im * self.rotor.im) as f32,
                (re * self.rotor.im + im * self.rotor.re) as f32,
            ));
            self.rotor *= self.step;
        }
        self.counter = self.counter.wrapping_add(1);
        self.rotor /= self.rotor.norm();
        match &mut self.decimator {
            Some(decimator) => {
                self.middle.clear();
                decimator.process(&self.mixed, &mut self.middle);
                self.resampler.process(&self.middle, output);
            }
            None => self.resampler.process(&self.mixed, output),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(freq: f64, rate: f64, count: usize) -> Vec<Complex32> {
        (0..count)
            .map(|i| {
                let phase = TAU * freq * i as f64 / rate;
                Complex32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect()
    }

    fn power_after(input_rate: f64, tone_hz: f64, offset_hz: f64, bw: f64) -> f64 {
        let mut filter = ChannelFilter::new(input_rate, offset_hz, bw, 4);
        let input = tone(tone_hz, input_rate, 400_000);
        let mut out = Vec::new();
        for chunk in input.chunks(7001) {
            filter.process(chunk, &mut out);
        }
        let tail = &out[out.len() / 2..];
        tail.iter().map(|v| v.norm_sqr() as f64).sum::<f64>() / tail.len() as f64
    }

    #[test]
    fn rate_conversion_is_exact() {
        for (rate, bw) in [(2.4e6, 125e3), (1e6, 125e3), (2.048e6, 62.5e3), (1e6, 250e3), (2.4e6, 7812.5)] {
            let mut filter = ChannelFilter::new(rate, 0.0, bw, 4);
            let mut out = Vec::new();
            let input = vec![Complex32::new(1.0, 0.0); 1 << 16];
            for _ in 0..20 {
                filter.process(&input, &mut out);
            }
            let want = 20.0 * (1u64 << 16) as f64 * bw * 4.0 / rate;
            assert!((out.len() as f64 - want).abs() < 300.0, "{rate} {bw}: {} vs {want}", out.len());
        }
    }

    #[test]
    fn passes_the_channel_and_rejects_neighbours() {
        let rate = 2.4e6;
        let bw = 125e3;
        let offset = 300e3;
        for inside in [0.0, 50e3, -50e3, 100e3] {
            let p = power_after(rate, offset + inside, offset, bw);
            assert!((p - 1.0).abs() < 0.05, "{inside}: {p}");
        }
        for outside in [400e3, -400e3, 600e3] {
            let p = power_after(rate, offset + outside, offset, bw);
            assert!(p < 1e-4, "{outside}: {p}");
        }
    }
}

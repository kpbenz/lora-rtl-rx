//! End-to-end tests of the receive chain using the crate's own modulator.

use lora_rtl_rx::dsp::ChannelFilter;
use lora_rtl_rx::lora::modulator::{modulate, TxImpairments};
use lora_rtl_rx::lora::{Decoder, DecoderEvent, Frame, Ldro, LoraParams};
use num_complex::Complex32;
use rand::rngs::StdRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

fn add_noise(signal: &mut [Complex32], snr_db: f32, rng: &mut StdRng, oversample: f32) {
    // SNR is defined in the LoRa bandwidth; the noise fills the sampled band.
    let noise_power = oversample * 10f32.powf(-snr_db / 10.0);
    let normal = Normal::new(0.0, (noise_power / 2.0).sqrt()).unwrap();
    for s in signal.iter_mut() {
        s.re += normal.sample(rng);
        s.im += normal.sample(rng);
    }
}

struct Case {
    p: LoraParams,
    cr: u8,
    fs: f64,
    offset: f64,
    imp: TxImpairments,
    snr_db: f32,
    rf_hz: f64,
}

fn run(case: &Case, payload: &[u8], frames: usize, seed: u64) -> Vec<Frame> {
    let mut rng = StdRng::seed_from_u64(seed);
    let bw = case.p.bw();
    let osr = ChannelFilter::oversampling(case.fs, bw).unwrap();
    let mut filter = ChannelFilter::new(case.fs, case.offset, bw, osr);
    let mut decoder = Decoder::new(case.p.clone(), osr, case.rf_hz).unwrap();
    let mut events = Vec::new();
    let mut output = Vec::new();

    let imp = case.imp;
    let shift = std::f64::consts::TAU * case.offset / case.fs;
    let gap = (case.p.symbol_time_s() * 11.3 * case.fs) as usize;
    let mut stream: Vec<Complex32> = vec![Complex32::default(); gap];
    for _ in 0..frames {
        stream.extend(modulate(&case.p, payload, case.cr, true, case.fs, imp));
        stream.extend(std::iter::repeat_n(Complex32::default(), gap));
    }
    let oversample = (case.fs / bw) as f32;
    add_noise(&mut stream, case.snr_db, &mut rng, oversample);
    for (i, s) in stream.iter_mut().enumerate() {
        let phase = shift * i as f64;
        *s *= Complex32::new(phase.cos() as f32, phase.sin() as f32);
    }
    for chunk in stream.chunks(16384) {
        output.clear();
        filter.process(chunk, &mut output);
        decoder.push(&output, &mut events);
    }
    decoder.finish(&mut events);
    events
        .into_iter()
        .filter_map(|e| match e {
            DecoderEvent::Frame(f) => Some(f),
            _ => None,
        })
        .collect()
}

fn clean(sf: u8, bw_hz: u32) -> Case {
    Case {
        p: LoraParams { sf, bw_hz, ..Default::default() },
        cr: 1,
        fs: 1_000_000.0,
        offset: 200_000.0,
        imp: TxImpairments::default(),
        snr_db: 30.0,
        rf_hz: 915e6,
    }
}

fn assert_all(case: &Case, payload: &[u8], frames: usize) {
    let got = run(case, payload, frames, 1);
    let good = got.iter().filter(|f| f.crc_ok == Some(true) && f.payload == payload).count();
    let brief: Vec<_> = got.iter().map(|f| (f.crc_ok, f.payload.len(), f.cr, f.snr_db, f.cfo_hz, f.fec_corrected, f.fec_detected)).collect();
    assert_eq!(good, frames, "sf{} bw{} cr{} fs{} cfo{} got {:?}", case.p.sf, case.p.bw_hz, case.cr, case.fs, case.imp.cfo_hz, brief);
}

#[test]
fn every_spreading_factor_at_125k() {
    for sf in 7..=12 {
        for cr in 1..=4 {
            let mut c = clean(sf, 125_000);
            c.cr = cr;
            assert_all(&c, b"Hello LoRa from Rust!", 2);
        }
    }
}

#[test]
fn large_carrier_and_clock_offset() {
    // A dongle and a transmitter that disagree by 40 ppm at 915 MHz.
    for sf in 7..=12 {
        for ppm in [-40.0, -17.0, 13.0, 40.0] {
            let mut c = clean(sf, 125_000);
            c.imp.cfo_hz = ppm * 915.0;
            c.imp.sfo_ppm = ppm;
            assert_all(&c, b"offset frame", 2);
        }
    }
}

#[test]
fn longest_payload_with_a_clock_error() {
    let payload: Vec<u8> = (0..=254u8).collect();
    for sf in [7, 9, 12] {
        let mut c = clean(sf, 125_000);
        c.cr = 4;
        c.imp.cfo_hz = 25.0 * 915.0;
        c.imp.sfo_ppm = 25.0;
        assert_all(&c, &payload, 1);
    }
}

#[test]
fn drift_during_a_long_frame() {
    // SF12 with LDRO: a crystal warming up moves 30 Hz/s across a 12 s frame.
    let mut c = clean(12, 125_000);
    c.cr = 4;
    c.imp.drift_hz_per_s = 30.0;
    c.imp.cfo_hz = 2000.0;
    c.imp.sfo_ppm = 2000.0 / 915.0;
    let payload: Vec<u8> = (0..120u8).collect();
    assert_all(&c, &payload, 1);
}

#[test]
fn every_bandwidth_and_common_sample_rates() {
    for &bw in &[7_812u32, 10_417, 15_625, 20_833, 31_250, 41_667, 62_500, 125_000, 250_000, 500_000] {
        for fs in [2_400_000.0, 2_048_000.0, 1_000_000.0] {
            if fs < bw as f64 * 2.0 {
                continue;
            }
            let sf = if bw < 62_500 { 7 } else { 9 };
            let mut c = clean(sf, bw);
            c.fs = fs;
            c.offset = (fs / 5.0).min(fs / 2.0 - bw as f64 * 1.5);
            assert_all(&c, b"rate test", 2);
        }
    }
}

#[test]
fn sync_word_filtering_and_any() {
    let c = Case {
        p: LoraParams { sync_word: 0x34, ..Default::default() },
        ..clean(8, 125_000)
    };
    assert_all(&c, b"lorawan style", 2);

    // Transmit 0x34, listen for 0x12: nothing should come out.
    let frames = {
        let bw = 125_000.0;
        let fs = 1e6;
        let tx = LoraParams { sf: 8, sync_word: 0x34, ..Default::default() };
        let rx = LoraParams { sf: 8, sync_word: 0x12, ..Default::default() };
        let mut stream = vec![Complex32::default(); 30_000];
        stream.extend(modulate(&tx, b"not for us", 1, true, fs, TxImpairments::default()));
        stream.extend(vec![Complex32::default(); 30_000]);
        let mut filter = ChannelFilter::new(fs, 0.0, bw, 4);
        let mut dec = Decoder::new(rx, 4, 915e6).unwrap();
        let mut out = Vec::new();
        filter.process(&stream, &mut out);
        let mut ev = Vec::new();
        dec.push(&out, &mut ev);
        dec.finish(&mut ev);
        let mismatch = ev.iter().any(|e| matches!(e, DecoderEvent::SyncMismatch { found: 0x34, .. }));
        assert!(mismatch, "the wrong sync word should be reported: {ev:?}");
        ev.iter().filter(|e| matches!(e, DecoderEvent::Frame(_))).count()
    };
    assert_eq!(frames, 0);

    let any = Case {
        p: LoraParams { sync_word: 0x34, ..Default::default() },
        ..clean(8, 125_000)
    };
    let mut listen_any = any;
    listen_any.p.any_sync_word = true;
    let got = run(&listen_any, b"any sync", 1, 3);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].sync_word, 0x34);
}

#[test]
fn implicit_header_and_low_spreading_factors() {
    for sf in 5..=9 {
        let payload = b"implicit!";
        let p = LoraParams {
            sf,
            implicit_header: true,
            implicit_len: payload.len() as u8,
            implicit_crc: true,
            cr: 2,
            ..Default::default()
        };
        let c = Case { p, cr: 2, ..clean(sf, 125_000) };
        assert_all(&c, payload, 2);
    }
}

#[test]
fn ldro_forced_on_and_inverted_iq_and_long_preamble() {
    let mut c = clean(9, 125_000);
    c.p.ldro = Ldro::On;
    assert_all(&c, b"ldro on", 2);

    let mut c = clean(8, 125_000);
    c.p.invert_iq = true;
    c.imp.cfo_hz = 12_000.0;
    c.imp.sfo_ppm = 13.0;
    assert_all(&c, b"inverted", 2);

    let mut c = clean(8, 125_000);
    c.p.preamble_len = 16;
    assert_all(&c, b"meshtastic style", 2);

    let mut c = clean(7, 125_000);
    c.p.preamble_len = 64;
    assert_all(&c, b"long preamble", 2);

    let mut c = clean(7, 125_000);
    c.p.preamble_len = 6;
    assert_all(&c, b"short preamble", 2);
}

/// Semtech quote -7.5 dB at SF7 and -20 dB at SF12. A software decoder with hard
/// decisions should reach within a few dB.
#[test]
fn sensitivity() {
    for (sf, snr) in [(7u8, -5.0f32), (8, -7.5), (9, -10.0), (10, -12.5), (11, -15.0), (12, -17.5)] {
        let mut c = clean(sf, 125_000);
        c.cr = 4;
        c.snr_db = snr;
        c.imp.cfo_hz = 15_000.0;
        c.imp.sfo_ppm = 15_000.0 / 915.0;
        let payload = b"sensitivity test";
        let frames = if sf >= 11 { 6 } else { 20 };
        let got = run(&c, payload, frames, sf as u64);
        let good = got.iter().filter(|f| f.crc_ok == Some(true) && f.payload == payload).count();
        assert!(good * 10 >= frames * 9, "SF{sf} at {snr} dB: {good}/{frames}");
    }
}

#[test]
fn frames_with_no_gap_between_them() {
    let p = LoraParams { sf: 8, ..Default::default() };
    let fs = 1e6;
    let payload = b"back to back";
    let mut stream = vec![Complex32::default(); 5000];
    for _ in 0..4 {
        stream.extend(modulate(&p, payload, 1, true, fs, TxImpairments::default()));
    }
    let mut filter = ChannelFilter::new(fs, 0.0, 125e3, 4);
    let mut decoder = Decoder::new(p, 4, 915e6).unwrap();
    let mut out = Vec::new();
    filter.process(&stream, &mut out);
    let mut events = Vec::new();
    decoder.push(&out, &mut events);
    decoder.finish(&mut events);
    let good = events
        .iter()
        .filter(|e| matches!(e, DecoderEvent::Frame(f) if f.crc_ok == Some(true)))
        .count();
    assert_eq!(good, 4);
}

/// What a real dongle does to a signal: 8 bit samples, a DC offset, IQ imbalance,
/// a strong spur near the channel, and a tens of ppm crystal error.
#[test]
fn dongle_impairments() {
    for (sf, snr) in [(7u8, 0.0f32), (9, -5.0), (12, -12.0)] {
        let p = LoraParams { sf, ..Default::default() };
        let fs = 1e6;
        let offset = 200e3;
        let rf = 915e6;
        let ppm = -32.0;
        let imp = TxImpairments { cfo_hz: ppm * rf * 1e-6, sfo_ppm: ppm, drift_hz_per_s: 0.0 };
        let payload = b"through a dongle";
        let mut rng = StdRng::seed_from_u64(sf as u64);
        let frame = modulate(&p, payload, 1, true, fs, imp);
        let gap = (p.symbol_time_s() * 9.7 * fs) as usize;
        let mut stream = vec![Complex32::default(); gap];
        for _ in 0..3 {
            stream.extend_from_slice(&frame);
            stream.extend(vec![Complex32::default(); gap]);
        }
        // A weak signal, a few LSBs at the ADC.
        let amplitude = 0.08f32;
        let noise_power = amplitude * amplitude * (fs / p.bw()) as f32 * 10f32.powf(-snr / 10.0);
        let normal = Normal::new(0.0, (noise_power / 2.0).sqrt()).unwrap();
        let tau = std::f64::consts::TAU;
        let quantise = |x: f32| ((x * 128.0 + 127.4).round().clamp(0.0, 255.0) - 127.4) / 128.0;
        let spur_hz = offset + 40_000.0;
        for (i, s) in stream.iter_mut().enumerate() {
            let t = i as f64 / fs;
            let mix = (tau * offset * t) as f32;
            let mut v = *s * amplitude * Complex32::new(mix.cos(), mix.sin());
            let spur = (tau * spur_hz * t) as f32;
            v += Complex32::new(spur.cos(), spur.sin()) * 0.05;
            v += Complex32::new(normal.sample(&mut rng), normal.sample(&mut rng));
            // 1 dB gain and 3 degree phase imbalance, and an offset at DC.
            let (re, im) = (v.re * 1.12 + 0.02, (v.im + 0.05 * v.re) - 0.015);
            *s = Complex32::new(quantise(re), quantise(im));
        }
        let mut filter = ChannelFilter::new(fs, offset, p.bw(), 4);
        let mut decoder = Decoder::new(p, 4, rf).unwrap();
        let mut events = Vec::new();
        let mut out = Vec::new();
        for chunk in stream.chunks(32768) {
            out.clear();
            filter.process(chunk, &mut out);
            decoder.push(&out, &mut events);
        }
        decoder.finish(&mut events);
        let good = events
            .iter()
            .filter(|e| matches!(e, DecoderEvent::Frame(f) if f.crc_ok == Some(true) && f.payload == payload))
            .count();
        assert_eq!(good, 3, "SF{sf} at {snr} dB: {events:?}");
    }
}

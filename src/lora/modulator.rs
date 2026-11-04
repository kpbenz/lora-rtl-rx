//! A LoRa modulator, for the built-in simulator and for tests.

use super::coding::encode_symbols;
use super::params::LoraParams;
use num_complex::Complex32;
use std::f64::consts::TAU;

/// Channel impairments applied while generating a frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct TxImpairments {
    /// Carrier offset in hertz.
    pub cfo_hz: f64,
    /// Transmit clock error in parts per million (positive runs fast).
    pub sfo_ppm: f64,
    /// Carrier drift over the frame in hertz per second.
    pub drift_hz_per_s: f64,
}

/// Generates the frame as a continuous phase signal at sample rate `fs`, as a
/// real radio does.
pub fn modulate(
    p: &LoraParams,
    payload: &[u8],
    cr: u8,
    has_crc: bool,
    fs: f64,
    imp: TxImpairments,
) -> Vec<Complex32> {
    let n = p.bins() as f64;
    let bw = p.bw();
    let (s1, s2) = p.sync_symbols();
    let data = encode_symbols(p, payload, cr, has_crc);

    // (start offset in chips, duration in symbols, is_down)
    let mut plan: Vec<(f64, f64, bool)> = Vec::with_capacity(data.len() + p.preamble_len as usize + 5);
    for _ in 0..p.preamble_len {
        plan.push((0.0, 1.0, false));
    }
    plan.push((s1 as f64, 1.0, false));
    plan.push((s2 as f64, 1.0, false));
    plan.push((0.0, 1.0, true));
    plan.push((0.0, 1.0, true));
    plan.push((0.0, 0.25, true));
    for &s in &data {
        plan.push((s as f64, 1.0, false));
    }
    let total_sym: f64 = plan.iter().map(|x| x.1).sum();

    let dt = (1.0 + imp.sfo_ppm * 1e-6) / fs;
    let t_sym = n / bw;
    let mut out = Vec::with_capacity((total_sym * t_sym * fs) as usize + 8);
    let mut phase = 0.0f64;
    let mut t_abs = 0.0f64;
    let mut sym_start = 0.0f64;
    for &(s, dur, down) in &plan {
        let t_end = sym_start + dur * t_sym;
        while t_abs < t_end {
            // Mid-sample frequency, so the sweep has no half sample bias.
            let tau = (t_abs + dt / 2.0 - sym_start) / t_sym;
            let f = if down {
                bw * (0.5 - tau.fract())
            } else {
                bw * ((tau + s / n).fract() - 0.5)
            };
            let carrier = imp.cfo_hz + imp.drift_hz_per_s * t_abs;
            out.push(Complex32::new(phase.cos() as f32, phase.sin() as f32));
            phase = (phase + TAU * (f + carrier) * dt) % TAU;
            t_abs += dt;
        }
        sym_start = t_end;
    }
    if p.invert_iq {
        for v in &mut out {
            *v = v.conj();
        }
    }
    out
}

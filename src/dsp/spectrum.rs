//! Spectrum lines for the waterfall.

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::TAU;
use std::sync::Arc;

/// Produces peak-hold spectrum lines, so that a burst shorter than one line of
/// the waterfall still shows up.
pub struct Spectrum {
    size: usize,
    window: Vec<f32>,
    fft: Arc<dyn Fft<f32>>,
    buffer: Vec<Complex32>,
    held: Vec<f32>,
    samples_per_line: usize,
    since_line: usize,
}

impl Spectrum {
    pub fn new(size: usize, sample_rate: f64, lines_per_second: f64) -> Self {
        let window: Vec<f32> = (0..size)
            .map(|i| 0.5 - 0.5 * (TAU * i as f32 / size as f32).cos())
            .collect();
        Self {
            size,
            window,
            fft: FftPlanner::new().plan_fft_forward(size),
            buffer: Vec::with_capacity(size),
            held: vec![0.0; size],
            samples_per_line: (sample_rate / lines_per_second) as usize,
            since_line: 0,
        }
    }

    /// Adds samples, calling `emit` with each finished line: dB, DC centred.
    pub fn push(&mut self, samples: &[Complex32], mut emit: impl FnMut(Vec<f32>)) {
        for &s in samples {
            self.buffer.push(s);
            if self.buffer.len() == self.size {
                for (v, w) in self.buffer.iter_mut().zip(&self.window) {
                    *v *= *w;
                }
                self.fft.process(&mut self.buffer);
                let scale = 1.0 / (self.size as f32 * self.size as f32 * 0.25);
                for (h, v) in self.held.iter_mut().zip(&self.buffer) {
                    *h = h.max(v.norm_sqr() * scale);
                }
                self.buffer.clear();
            }
            self.since_line += 1;
            if self.since_line >= self.samples_per_line {
                self.since_line = 0;
                let half = self.size / 2;
                let line = (0..self.size)
                    .map(|i| 10.0 * self.held[(i + half) % self.size].max(1e-14).log10())
                    .collect();
                self.held.iter_mut().for_each(|h| *h = 0.0);
                emit(line);
            }
        }
    }
}

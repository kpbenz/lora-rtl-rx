//! A built-in LoRa transmitter, for trying the program without a radio.

use super::SourceMessage;
use crate::lora::modulator::{modulate, TxImpairments};
use crate::lora::LoraParams;
use crossbeam_channel::Sender;
use num_complex::Complex32;
use rand::Rng;
use rand_distr::{Distribution, Normal};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Simulated channel.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimSettings {
    pub sample_rate: f64,
    /// Signal to noise ratio in the LoRa bandwidth.
    pub snr_db: f32,
    /// Crystal disagreement between transmitter and receiver.
    pub ppm: f64,
    /// Seconds between frames.
    pub interval_s: f64,
}

impl Default for SimSettings {
    fn default() -> Self {
        Self { sample_rate: 1_000_000.0, snr_db: 5.0, ppm: 8.0, interval_s: 1.0 }
    }
}

const BLOCK_SAMPLES: usize = 16_384;

/// Starts the simulator. `offset_hz` places the signal relative to the centre.
pub fn start(
    settings: SimSettings,
    params: LoraParams,
    rf_hz: f64,
    offset_hz: f64,
    tx: Sender<SourceMessage>,
    stop: Arc<AtomicBool>,
) -> Result<std::thread::JoinHandle<()>, String> {
    std::thread::Builder::new()
        .name("simulator".into())
        .spawn(move || run(settings, params, rf_hz, offset_hz, tx, stop))
        .map_err(|e| e.to_string())
}

fn run(
    s: SimSettings,
    p: LoraParams,
    rf_hz: f64,
    offset_hz: f64,
    tx: Sender<SourceMessage>,
    stop: Arc<AtomicBool>,
) {
    let mut rng = rand::thread_rng();
    // Full scale is 1.0; a realistic level leaves headroom for the noise.
    let amplitude = 0.05f32;
    let oversample = (s.sample_rate / p.bw()) as f32;
    let noise_power = amplitude * amplitude * oversample * 10f32.powf(-s.snr_db / 10.0);
    let normal = Normal::new(0.0, (noise_power / 2.0).sqrt()).expect("noise power is finite");

    let mut counter = 0u32;
    let mut burst: Vec<Complex32> = Vec::new();
    let mut burst_pos = 0;
    let mut until_next = (s.sample_rate * 0.2) as usize;
    let mut sent = 0u64;
    let mut phase = 0.0f64;
    let step = std::f64::consts::TAU * offset_hz / s.sample_rate;
    let started = Instant::now();

    while !stop.load(Ordering::Relaxed) {
        let mut block = Vec::with_capacity(BLOCK_SAMPLES);
        for _ in 0..BLOCK_SAMPLES {
            if burst_pos >= burst.len() && until_next == 0 {
                counter += 1;
                let text = format!("Simulated frame {counter}: t={:.1}C rh={}%", 18.0 + (counter % 40) as f32 * 0.1, 40 + counter % 30);
                let bytes = if p.implicit_header {
                    let mut b = text.into_bytes();
                    b.resize(p.implicit_len as usize, b' ');
                    b
                } else {
                    text.into_bytes()
                };
                let ppm = s.ppm + rng.gen_range(-0.5..0.5);
                let has_crc = !p.implicit_header || p.implicit_crc;
                burst = modulate(
                    &p,
                    &bytes,
                    p.cr,
                    has_crc,
                    s.sample_rate,
                    TxImpairments { cfo_hz: ppm * rf_hz * 1e-6, sfo_ppm: ppm, drift_hz_per_s: 0.0 },
                );
                burst_pos = 0;
                until_next = (s.interval_s * s.sample_rate) as usize;
            }
            let mut value = Complex32::new(normal.sample(&mut rng), normal.sample(&mut rng));
            if burst_pos < burst.len() {
                let rotor = Complex32::new(phase.cos() as f32, phase.sin() as f32);
                value += burst[burst_pos] * amplitude * rotor;
                burst_pos += 1;
            } else {
                until_next -= 1;
            }
            phase = (phase + step) % std::f64::consts::TAU;
            block.push(value);
        }
        sent += block.len() as u64;
        if tx.send(SourceMessage::Samples(block)).is_err() {
            return;
        }
        let due = Duration::from_secs_f64(sent as f64 / s.sample_rate);
        if let Some(wait) = due.checked_sub(started.elapsed()) {
            std::thread::sleep(wait);
        }
    }
}

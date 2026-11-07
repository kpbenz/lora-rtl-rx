//! Writes a LoRa test recording, for trying the receiver without a transmitter.

use lora_rtl_rx::lora::modulator::{modulate, TxImpairments};
use lora_rtl_rx::lora::LoraParams;
use num_complex::Complex32;
use rand::rngs::StdRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};
use std::collections::HashMap;
use std::io::{BufWriter, Write};

const USAGE: &str = "\
usage: lora-gen OUT.cu8 [options]     (8 bit unsigned IQ, as rtl_sdr records)

  --frames N --gap SECONDS     frames and the silence around them (5, 0.2)
  --text STRING                payload (a counter is appended)
  --hex HEX                    payload bytes in hex, sent unchanged every frame
  --rate HZ --offset HZ        sample rate and channel offset (1000000, 200000)
  --sf N --bw HZ --cr N        spreading factor, bandwidth, coding rate 1..4
  --sync 0xNN                  sync word
  --snr DB --ppm PPM           channel quality and crystal error (10, 5)
  --freq HZ                    carrier frequency, for the crystal error (915000000)";

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}\n\n{USAGE}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let mut path = None;
    let mut options: HashMap<String, String> = HashMap::new();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--help" || arg == "-h" {
            println!("{USAGE}");
            return Ok(());
        } else if let Some(name) = arg.strip_prefix("--") {
            options.insert(name.to_string(), args.next().ok_or(format!("{arg} needs a value"))?);
        } else {
            path = Some(arg);
        }
    }
    let path = path.ok_or("no output file given")?;
    let number = |name: &str, default: f64| -> Result<f64, String> {
        match options.get(name) {
            Some(text) => text.parse().map_err(|_| format!("--{name}: cannot read {text}")),
            None => Ok(default),
        }
    };
    let rate = number("rate", 1e6)?;
    let offset = number("offset", 200e3)?;
    let snr = number("snr", 10.0)? as f32;
    let ppm = number("ppm", 5.0)?;
    let freq = number("freq", 915e6)?;
    let frames = number("frames", 5.0)? as usize;
    let gap = (number("gap", 0.2)? * rate) as usize;
    let sync = match options.get("sync") {
        Some(text) => u8::from_str_radix(text.trim_start_matches("0x"), 16)
            .map_err(|_| format!("--sync: cannot read {text}"))?,
        None => 0x12,
    };
    let p = LoraParams {
        sf: number("sf", 7.0)? as u8,
        bw_hz: number("bw", 125_000.0)? as u32,
        cr: number("cr", 1.0)? as u8,
        sync_word: sync,
        ..Default::default()
    };
    p.validate()?;
    let text = options.get("text").cloned().unwrap_or_else(|| "Hello from lora-gen".into());
    let fixed = match options.get("hex") {
        Some(hex) => {
            let clean: String = hex.chars().filter(|c| !c.is_whitespace()).collect();
            if clean.is_empty() || clean.len() % 2 != 0 || clean.len() > 510 {
                return Err("--hex needs 1 to 255 bytes as an even number of hex digits".into());
            }
            let bytes = (0..clean.len() / 2)
                .map(|i| u8::from_str_radix(&clean[2 * i..2 * i + 2], 16))
                .collect::<Result<Vec<u8>, _>>()
                .map_err(|_| format!("--hex: cannot read {hex}"))?;
            Some(bytes)
        }
        None => None,
    };
    let impairments = TxImpairments { cfo_hz: ppm * freq * 1e-6, sfo_ppm: ppm, drift_hz_per_s: 0.0 };

    let amplitude = 0.1f32;
    let noise_power = amplitude * amplitude * (rate / p.bw()) as f32 * 10f32.powf(-snr / 10.0);
    let normal = Normal::new(0.0, (noise_power / 2.0).sqrt()).map_err(|e| e.to_string())?;
    let mut rng = StdRng::seed_from_u64(42);
    let mut out = BufWriter::new(std::fs::File::create(&path).map_err(|e| format!("{path}: {e}"))?);
    let step = std::f64::consts::TAU * offset / rate;
    let mut index = 0u64;
    let mut write = |signal: &[Complex32], gain: f32, out: &mut BufWriter<std::fs::File>, rng: &mut StdRng| {
        for s in signal {
            let phase = step * index as f64;
            index += 1;
            let rotor = Complex32::new(phase.cos() as f32, phase.sin() as f32);
            let v = s * rotor * gain;
            let re = v.re + normal.sample(rng);
            let im = v.im + normal.sample(rng);
            let quantise = |x: f32| (x * 128.0 + 127.4).round().clamp(0.0, 255.0) as u8;
            out.write_all(&[quantise(re), quantise(im)]).expect("write failed");
        }
    };
    let silence = vec![Complex32::default(); gap];
    write(&silence, 0.0, &mut out, &mut rng);
    for i in 1..=frames {
        let payload = match &fixed {
            Some(bytes) => bytes.clone(),
            None => format!("{text} #{i}").into_bytes(),
        };
        let signal = modulate(&p, &payload, p.cr, true, rate, impairments);
        write(&signal, amplitude, &mut out, &mut rng);
        write(&silence, 0.0, &mut out, &mut rng);
    }
    out.flush().map_err(|e| e.to_string())?;
    Ok(())
}

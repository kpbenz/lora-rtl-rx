//! Decodes an IQ recording without the GUI, printing one JSON object per frame.

use chrono::Utc;
use lora_rtl_rx::engine::{FrameRecord, Pipeline};
use lora_rtl_rx::lora::{DecoderEvent, Ldro, LoraParams};
use lora_rtl_rx::source::IqFormat;
use std::collections::HashMap;
use std::io::Read;

const USAGE: &str = "\
usage: lora-decode FILE [options]

  --format cu8|cs8|cs16|cf32   sample format (cu8)
  --rate HZ                    sample rate of the recording (1000000)
  --freq HZ                    LoRa carrier frequency (915000000)
  --offset HZ                  channel offset from the recording's centre (0)
  --sf N --bw HZ --cr N        spreading factor, bandwidth, coding rate 1..4
  --sync 0xNN | --any-sync     sync word, or accept any
  --preamble N                 preamble length
  --implicit LEN               implicit header with LEN byte payloads
  --no-crc                     implicit header without a CRC
  --ldro auto|on|off           low data rate optimisation
  --invert-iq                  receive inverted IQ
  --stats                      print decoder counters to stderr";

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}\n\n{USAGE}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let mut path = None;
    let mut options: HashMap<String, String> = HashMap::new();
    let mut flags: Vec<String> = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--any-sync" | "--no-crc" | "--invert-iq" | "--stats" => flags.push(arg),
            "--help" | "-h" => {
                println!("{USAGE}");
                return Ok(());
            }
            a if a.starts_with("--") => {
                let value = args.next().ok_or(format!("{a} needs a value"))?;
                options.insert(a[2..].to_string(), value);
            }
            _ => path = Some(arg),
        }
    }
    let path = path.ok_or("no input file given")?;
    let flag = |name: &str| flags.iter().any(|f| f == name);
    let number = |name: &str, default: f64| -> Result<f64, String> {
        match options.get(name) {
            Some(text) => text.parse().map_err(|_| format!("--{name}: cannot read {text}")),
            None => Ok(default),
        }
    };
    let format = match options.get("format") {
        Some(text) => IqFormat::parse(text).ok_or(format!("unknown format {text}"))?,
        None => IqFormat::Cu8,
    };
    let rate = number("rate", 1e6)?;
    let rf_hz = number("freq", 915e6)?;
    let offset = number("offset", 0.0)?;
    let defaults = LoraParams::default();
    let sync = match options.get("sync") {
        Some(text) => u8::from_str_radix(text.trim_start_matches("0x"), 16)
            .map_err(|_| format!("--sync: cannot read {text}"))?,
        None => defaults.sync_word,
    };
    let params = LoraParams {
        sf: number("sf", 7.0)? as u8,
        bw_hz: number("bw", 125_000.0)? as u32,
        cr: number("cr", 1.0)? as u8,
        sync_word: sync,
        any_sync_word: flag("--any-sync"),
        preamble_len: number("preamble", 8.0)? as u16,
        implicit_header: options.contains_key("implicit"),
        implicit_len: number("implicit", 0.0)? as u8,
        implicit_crc: !flag("--no-crc"),
        ldro: match options.get("ldro").map(String::as_str) {
            Some("on") => Ldro::On,
            Some("off") => Ldro::Off,
            _ => Ldro::Auto,
        },
        invert_iq: flag("--invert-iq"),
    };

    let mut file = std::fs::File::open(&path).map_err(|e| format!("{path}: {e}"))?;
    let mut pipeline = Pipeline::new(rate, offset, rf_hz, params)?;
    let size = format.sample_size();
    let mut bytes = vec![0u8; 65_536 * size];
    let mut events = Vec::new();
    let mut seq = 0;
    loop {
        let mut filled = 0;
        while filled < bytes.len() {
            let count = file.read(&mut bytes[filled..]).map_err(|e| e.to_string())?;
            if count == 0 {
                break;
            }
            filled += count;
        }
        if filled < size {
            break;
        }
        let samples = format.convert(&bytes[..filled - filled % size]);
        pipeline.process(&samples, &mut events);
        print_events(&mut events, &mut seq, rf_hz);
    }
    pipeline.finish(&mut events);
    print_events(&mut events, &mut seq, rf_hz);
    if flag("--stats") {
        eprintln!("{:?}", pipeline.stats());
    }
    Ok(())
}

fn print_events(events: &mut Vec<DecoderEvent>, seq: &mut u64, rf_hz: f64) {
    for event in events.drain(..) {
        match event {
            DecoderEvent::Frame(frame) => {
                *seq += 1;
                let record = FrameRecord { seq: *seq, time: Utc::now(), rf_hz, frame };
                println!("{}", record.to_json());
            }
            other => eprintln!("{other:?}"),
        }
    }
}

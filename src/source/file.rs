//! Replay of IQ recordings.

use super::SourceMessage;
use crossbeam_channel::Sender;
use num_complex::Complex32;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Sample formats of common tools.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IqFormat {
    /// Unsigned 8 bit, from `rtl_sdr`.
    Cu8,
    /// Signed 8 bit, from `hackrf_transfer`.
    Cs8,
    /// Signed 16 bit little endian.
    Cs16,
    /// 32 bit float little endian, GNU Radio's `.cfile`.
    Cf32,
}

impl IqFormat {
    pub const ALL: [IqFormat; 4] = [IqFormat::Cu8, IqFormat::Cs8, IqFormat::Cs16, IqFormat::Cf32];

    pub fn name(self) -> &'static str {
        match self {
            IqFormat::Cu8 => "cu8 (rtl_sdr)",
            IqFormat::Cs8 => "cs8 (hackrf)",
            IqFormat::Cs16 => "cs16",
            IqFormat::Cf32 => "cf32 (.cfile)",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "cu8" => Some(IqFormat::Cu8),
            "cs8" => Some(IqFormat::Cs8),
            "cs16" => Some(IqFormat::Cs16),
            "cf32" | "cfile" => Some(IqFormat::Cf32),
            _ => None,
        }
    }

    /// Bytes per complex sample.
    pub fn sample_size(self) -> usize {
        match self {
            IqFormat::Cu8 | IqFormat::Cs8 => 2,
            IqFormat::Cs16 => 4,
            IqFormat::Cf32 => 8,
        }
    }

    /// Converts whole samples, ignoring any partial one at the end.
    pub fn convert(self, bytes: &[u8]) -> Vec<Complex32> {
        let size = self.sample_size();
        bytes
            .chunks_exact(size)
            .map(|s| match self {
                IqFormat::Cu8 => Complex32::new((s[0] as f32 - 127.4) / 128.0, (s[1] as f32 - 127.4) / 128.0),
                IqFormat::Cs8 => Complex32::new(s[0] as i8 as f32 / 128.0, s[1] as i8 as f32 / 128.0),
                IqFormat::Cs16 => Complex32::new(
                    i16::from_le_bytes([s[0], s[1]]) as f32 / 32768.0,
                    i16::from_le_bytes([s[2], s[3]]) as f32 / 32768.0,
                ),
                IqFormat::Cf32 => Complex32::new(
                    f32::from_le_bytes([s[0], s[1], s[2], s[3]]),
                    f32::from_le_bytes([s[4], s[5], s[6], s[7]]),
                ),
            })
            .collect()
    }
}

/// How to replay a file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FileSettings {
    pub path: PathBuf,
    pub format: IqFormat,
    /// The rate the file was recorded at.
    pub sample_rate: f64,
    /// Replay at the recorded rate rather than as fast as possible.
    pub realtime: bool,
    pub looped: bool,
}

impl Default for FileSettings {
    fn default() -> Self {
        Self {
            path: PathBuf::new(),
            format: IqFormat::Cu8,
            sample_rate: 1_000_000.0,
            realtime: true,
            looped: false,
        }
    }
}

const BLOCK_SAMPLES: usize = 32_768;

/// Starts replaying on its own thread.
pub fn start(
    settings: FileSettings,
    tx: Sender<SourceMessage>,
    stop: Arc<AtomicBool>,
) -> Result<std::thread::JoinHandle<()>, String> {
    let file = open(&settings.path)?;
    std::thread::Builder::new()
        .name("iq-file".into())
        .spawn(move || replay(file, settings, tx, stop))
        .map_err(|e| e.to_string())
}

fn open(path: &Path) -> Result<File, String> {
    File::open(path).map_err(|e| format!("cannot open {}: {e}", path.display()))
}

fn replay(mut file: File, s: FileSettings, tx: Sender<SourceMessage>, stop: Arc<AtomicBool>) {
    let size = s.format.sample_size();
    let mut bytes = vec![0u8; BLOCK_SAMPLES * size];
    let started = Instant::now();
    let mut sent = 0u64;
    while !stop.load(Ordering::Relaxed) {
        let mut filled = 0;
        while filled < bytes.len() {
            match file.read(&mut bytes[filled..]) {
                Ok(0) => break,
                Ok(count) => filled += count,
                Err(e) => {
                    let _ = tx.send(SourceMessage::Failed(format!("reading the file failed: {e}")));
                    return;
                }
            }
        }
        if filled < size {
            if s.looped && file.seek(SeekFrom::Start(0)).is_ok() {
                continue;
            }
            let _ = tx.send(SourceMessage::End);
            return;
        }
        let samples = s.format.convert(&bytes[..filled - filled % size]);
        sent += samples.len() as u64;
        if tx.send(SourceMessage::Samples(samples)).is_err() {
            return;
        }
        if s.realtime {
            let due = Duration::from_secs_f64(sent as f64 / s.sample_rate);
            if let Some(wait) = due.checked_sub(started.elapsed()) {
                std::thread::sleep(wait);
            }
        }
    }
}

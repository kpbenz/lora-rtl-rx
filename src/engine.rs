//! The receive pipeline: a source thread feeding a processing thread, which
//! reports to the GUI and the socket.

use crate::dsp::{ChannelFilter, Spectrum};
use crate::lora::decoder::DecoderStats;
use crate::lora::{Decoder, DecoderEvent, Frame, LoraParams};
use crate::socket::{hex, SocketServer};
use crate::source::{file, simulator, FileSettings, RadioSettings, RtlDevice, SimSettings, SourceMessage};
use chrono::{DateTime, SecondsFormat, Utc};
use crossbeam_channel::{bounded, select, unbounded, Receiver, Sender};
use num_complex::Complex32;
use serde_json::json;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Columns in a waterfall line.
pub const SPECTRUM_SIZE: usize = 1024;
const WATERFALL_LINES_PER_SECOND: f64 = 30.0;

/// A frame with where and when it was received.
#[derive(Debug, Clone)]
pub struct FrameRecord {
    pub seq: u64,
    pub time: DateTime<Utc>,
    pub rf_hz: f64,
    pub frame: Frame,
}

impl FrameRecord {
    pub fn crc_text(&self) -> &'static str {
        match self.frame.crc_ok {
            Some(true) => "ok",
            Some(false) => "bad",
            None => "none",
        }
    }

    pub fn to_json(&self) -> serde_json::Value {
        let f = &self.frame;
        let round = |v: f64, places: i32| {
            let scale = 10f64.powi(places);
            (v * scale).round() / scale
        };
        json!({
            "seq": self.seq,
            "time": self.time.to_rfc3339_opts(SecondsFormat::Millis, true),
            "freq_hz": self.rf_hz.round() as u64,
            "sf": f.sf,
            "bw_hz": f.bw_hz,
            "cr": format!("4/{}", f.cr + 4),
            "crc": self.crc_text(),
            "sync_word": format!("0x{:02x}", f.sync_word),
            "ldro": f.ldro,
            "implicit_header": f.implicit_header,
            "snr_db": round(f.snr_db as f64, 1),
            "rssi_dbfs": round(f.rssi_dbfs as f64, 1),
            "cfo_hz": round(f.cfo_hz, 1),
            "clock_ppm": round(f.clock_ppm, 2),
            "fec_corrected": f.fec_corrected,
            "fec_detected": f.fec_detected,
            "length": f.payload.len(),
            "payload_hex": hex(&f.payload),
        })
    }
}

/// Which input to use.
#[derive(Debug, Clone)]
pub enum SourceConfig {
    Rtl(RadioSettings),
    File(FileSettings),
    Simulator(SimSettings),
}

/// What a run starts with.
#[derive(Debug, Clone)]
pub struct EngineConfig {
    pub source: SourceConfig,
    pub lora: LoraParams,
    /// LoRa carrier frequency.
    pub rf_hz: f64,
    /// Where the channel sits relative to the tuned or recorded centre.
    pub offset_hz: f64,
}

/// Changes that take effect while running.
#[derive(Debug, Clone)]
pub enum Command {
    Lora(LoraParams),
    Tune { rf_hz: f64, offset_hz: f64 },
    Gain { auto: bool, tenths: i32 },
    Ppm(i32),
    BiasTee(bool),
    Stop,
}

/// Rolling figures for the status bar.
#[derive(Debug, Clone, Copy, Default)]
pub struct EngineStats {
    pub sample_rate: f64,
    pub decoder: DecoderStats,
    /// Power in the LoRa channel relative to full scale.
    pub channel_dbfs: f32,
    /// Share of samples at the ADC's limits.
    pub clipped_percent: f32,
    /// Samples lost because processing fell behind.
    pub dropped_samples: u64,
    /// Time spent processing as a share of real time.
    pub load_percent: f32,
}

/// What the engine reports.
#[derive(Debug)]
pub enum EngineEvent {
    Frame(FrameRecord),
    Waterfall(Vec<f32>),
    Stats(EngineStats),
    Notice(String),
    /// Hardware details once a dongle opens.
    DeviceReady { tuner: &'static str, gains: Vec<i32>, sample_rate: f64 },
    /// The run has ended, with the reason if it failed.
    Stopped(Option<String>),
}

/// Channel filter and decoder together.
pub struct Pipeline {
    filter: ChannelFilter,
    decoder: Decoder,
    osr: usize,
    buffer: Vec<Complex32>,
    power: f64,
    power_count: u64,
}

impl Pipeline {
    pub fn new(sample_rate: f64, offset_hz: f64, rf_hz: f64, p: LoraParams) -> Result<Self, String> {
        p.validate()?;
        let bw = p.bw();
        let osr = ChannelFilter::oversampling(sample_rate, bw)?;
        if offset_hz.abs() + bw / 2.0 > sample_rate / 2.0 {
            return Err(format!(
                "the channel at {:+.0} kHz does not fit in the {:.0} kHz sampled band",
                offset_hz / 1e3,
                sample_rate / 1e3
            ));
        }
        Ok(Self {
            filter: ChannelFilter::new(sample_rate, offset_hz, bw, osr),
            decoder: Decoder::new(p, osr, rf_hz)?,
            osr,
            buffer: Vec::with_capacity(1 << 16),
            power: 0.0,
            power_count: 0,
        })
    }

    pub fn oversampling(&self) -> usize {
        self.osr
    }

    pub fn process(&mut self, samples: &[Complex32], events: &mut Vec<DecoderEvent>) {
        self.buffer.clear();
        self.filter.process(samples, &mut self.buffer);
        for s in &self.buffer {
            self.power += s.norm_sqr() as f64;
        }
        self.power_count += self.buffer.len() as u64;
        self.decoder.push(&self.buffer, events);
    }

    pub fn finish(&mut self, events: &mut Vec<DecoderEvent>) {
        self.decoder.finish(events);
    }

    pub fn stats(&self) -> DecoderStats {
        self.decoder.stats
    }

    /// Mean channel power since last asked, in dBFS.
    pub fn take_power_dbfs(&mut self) -> f32 {
        let mean = self.power / self.power_count.max(1) as f64;
        self.power = 0.0;
        self.power_count = 0;
        10.0 * mean.max(1e-12).log10() as f32
    }
}

/// A running receiver.
pub struct Engine {
    commands: Sender<Command>,
    pub events: Receiver<EngineEvent>,
    thread: Option<std::thread::JoinHandle<()>>,
}

/// The socket is shared so that it survives stopping and starting the radio.
pub type SharedSocket = Arc<Mutex<Option<SocketServer>>>;

struct Running {
    data: Receiver<SourceMessage>,
    stop: Arc<AtomicBool>,
    worker: Option<std::thread::JoinHandle<()>>,
    device: Option<Arc<RtlDevice>>,
    sample_rate: f64,
    realtime: bool,
}

impl Running {
    fn shut_down(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(device) = self.device.take() {
            device.stop();
        }
        // A source blocked on a full channel needs the receiver gone to wake.
        drop(self.data);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn tuned_frequency(rf_hz: f64, offset_hz: f64) -> f64 {
    rf_hz - offset_hz
}

fn start_source(config: &EngineConfig, events: &Sender<EngineEvent>) -> Result<Running, String> {
    let stop = Arc::new(AtomicBool::new(false));
    let (tx, data) = bounded::<SourceMessage>(64);
    match &config.source {
        SourceConfig::Rtl(radio) => {
            let tuned = tuned_frequency(config.rf_hz, config.offset_hz);
            let device = Arc::new(RtlDevice::open(radio, tuned)?);
            let sample_rate = device.sample_rate();
            let _ = events.send(EngineEvent::DeviceReady {
                tuner: device.tuner_name(),
                gains: device.gains().to_vec(),
                sample_rate,
            });
            device.start(tx)?;
            Ok(Running { data, stop, worker: None, device: Some(device), sample_rate, realtime: true })
        }
        SourceConfig::File(settings) => {
            let realtime = settings.realtime;
            let worker = file::start(settings.clone(), tx, Arc::clone(&stop))?;
            Ok(Running {
                data,
                stop,
                worker: Some(worker),
                device: None,
                sample_rate: settings.sample_rate,
                realtime,
            })
        }
        SourceConfig::Simulator(settings) => {
            let worker = simulator::start(
                settings.clone(),
                config.lora.clone(),
                config.rf_hz,
                config.offset_hz,
                tx,
                Arc::clone(&stop),
            )?;
            Ok(Running {
                data,
                stop,
                worker: Some(worker),
                device: None,
                sample_rate: settings.sample_rate,
                realtime: true,
            })
        }
    }
}

impl Engine {
    pub fn start(config: EngineConfig, socket: SharedSocket) -> Result<Self, String> {
        config.lora.validate()?;
        let (commands, command_rx) = unbounded();
        let (event_tx, events) = unbounded();
        let (ready_tx, ready_rx) = bounded(1);
        let thread = std::thread::Builder::new()
            .name("dsp".into())
            .spawn(move || {
                let running = start_source(&config, &event_tx);
                let running = match running {
                    Ok(running) => running,
                    Err(e) => {
                        let _ = ready_tx.send(Err(e));
                        return;
                    }
                };
                let pipeline = Pipeline::new(running.sample_rate, config.offset_hz, config.rf_hz, config.lora.clone());
                let pipeline = match pipeline {
                    Ok(pipeline) => pipeline,
                    Err(e) => {
                        running.shut_down();
                        let _ = ready_tx.send(Err(e));
                        return;
                    }
                };
                let _ = ready_tx.send(Ok(()));
                let mut worker = Worker { config, running, pipeline, event_tx, socket, seq: 0 };
                worker.run(command_rx);
            })
            .map_err(|e| e.to_string())?;
        match ready_rx.recv() {
            Ok(Ok(())) => Ok(Self { commands, events, thread: Some(thread) }),
            Ok(Err(e)) => {
                let _ = thread.join();
                Err(e)
            }
            Err(_) => Err("the processing thread ended unexpectedly".into()),
        }
    }

    pub fn send(&self, command: Command) {
        let _ = self.commands.send(command);
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        let _ = self.commands.send(Command::Stop);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

struct Worker {
    config: EngineConfig,
    running: Running,
    pipeline: Pipeline,
    event_tx: Sender<EngineEvent>,
    socket: SharedSocket,
    seq: u64,
}

impl Worker {
    fn notice(&self, text: impl Into<String>) {
        let _ = self.event_tx.send(EngineEvent::Notice(text.into()));
    }

    fn rebuild(&mut self) {
        match Pipeline::new(
            self.running.sample_rate,
            self.config.offset_hz,
            self.config.rf_hz,
            self.config.lora.clone(),
        ) {
            Ok(pipeline) => self.pipeline = pipeline,
            Err(e) => self.notice(e),
        }
    }

    fn handle(&mut self, command: Command) -> bool {
        let device = self.running.device.clone();
        let result = match command {
            Command::Stop => return false,
            Command::Lora(p) => {
                // The simulator's transmitter follows the decoder's settings.
                let restart = matches!(self.config.source, SourceConfig::Simulator(_));
                self.config.lora = p;
                if restart {
                    self.restart_simulator();
                }
                self.rebuild();
                Ok(())
            }
            Command::Tune { rf_hz, offset_hz } => {
                self.config.rf_hz = rf_hz;
                self.config.offset_hz = offset_hz;
                self.rebuild();
                match device {
                    Some(d) => d.set_frequency(tuned_frequency(rf_hz, offset_hz)),
                    None => Ok(()),
                }
            }
            Command::Gain { auto, tenths } => device.map_or(Ok(()), |d| d.set_gain(auto, tenths)),
            Command::Ppm(ppm) => device.map_or(Ok(()), |d| d.set_ppm(ppm)),
            Command::BiasTee(on) => device.map_or(Ok(()), |d| d.set_bias_tee(on)),
        };
        if let Err(e) = result {
            self.notice(e);
        }
        true
    }

    fn restart_simulator(&mut self) {
        if let Ok(replacement) = start_source(&self.config, &self.event_tx) {
            let old = std::mem::replace(&mut self.running, replacement);
            old.shut_down();
        }
    }

    fn run(&mut self, commands: Receiver<Command>) {
        let mut spectrum = Spectrum::new(SPECTRUM_SIZE, self.running.sample_rate, WATERFALL_LINES_PER_SECOND);
        let mut events = Vec::new();
        let mut last_stats = Instant::now();
        let (mut busy, mut clipped, mut counted) = (Duration::ZERO, 0u64, 0u64);
        let mut failure = None;

        loop {
            let data = self.running.data.clone();
            select! {
                recv(commands) -> command => {
                    match command {
                        Ok(command) => if !self.handle(command) { break },
                        Err(_) => break,
                    }
                }
                recv(data) -> message => {
                    match message {
                        Ok(SourceMessage::Samples(samples)) => {
                            let began = Instant::now();
                            for s in &samples {
                                if s.re.abs() >= 0.99 || s.im.abs() >= 0.99 {
                                    clipped += 1;
                                }
                            }
                            counted += samples.len() as u64;
                            let tx = &self.event_tx;
                            spectrum.push(&samples, |line| {
                                // Waterfall lines are expendable if the GUI stalls.
                                if tx.len() < 256 {
                                    let _ = tx.send(EngineEvent::Waterfall(line));
                                }
                            });
                            self.pipeline.process(&samples, &mut events);
                            self.dispatch(&mut events);
                            busy += began.elapsed();
                        }
                        Ok(SourceMessage::End) => {
                            self.pipeline.finish(&mut events);
                            self.dispatch(&mut events);
                            break;
                        }
                        Ok(SourceMessage::Failed(e)) => {
                            failure = Some(e);
                            break;
                        }
                        Err(_) => break,
                    }
                }
                default(Duration::from_millis(250)) => {}
            }
            if last_stats.elapsed() >= Duration::from_millis(250) {
                let elapsed = last_stats.elapsed();
                let dropped = self
                    .running
                    .device
                    .as_ref()
                    .map_or(0, |d| d.dropped.load(Ordering::Relaxed));
                let stats = EngineStats {
                    sample_rate: self.running.sample_rate,
                    decoder: self.pipeline.stats(),
                    channel_dbfs: self.pipeline.take_power_dbfs(),
                    clipped_percent: 100.0 * clipped as f32 / counted.max(1) as f32,
                    dropped_samples: dropped,
                    load_percent: if self.running.realtime {
                        100.0 * busy.as_secs_f32() / elapsed.as_secs_f32()
                    } else {
                        0.0
                    },
                };
                let _ = self.event_tx.send(EngineEvent::Stats(stats));
                last_stats = Instant::now();
                busy = Duration::ZERO;
                clipped = 0;
                counted = 0;
            }
        }
        let stats = EngineStats {
            sample_rate: self.running.sample_rate,
            decoder: self.pipeline.stats(),
            ..Default::default()
        };
        let _ = self.event_tx.send(EngineEvent::Stats(stats));
        let placeholder = Running {
            data: bounded(0).1,
            stop: Arc::new(AtomicBool::new(false)),
            worker: None,
            device: None,
            sample_rate: 0.0,
            realtime: false,
        };
        std::mem::replace(&mut self.running, placeholder).shut_down();
        let _ = self.event_tx.send(EngineEvent::Stopped(failure));
    }

    fn dispatch(&mut self, events: &mut Vec<DecoderEvent>) {
        for event in events.drain(..) {
            match event {
                DecoderEvent::Frame(frame) => {
                    self.seq += 1;
                    let record = FrameRecord {
                        seq: self.seq,
                        time: Utc::now(),
                        rf_hz: self.config.rf_hz,
                        frame,
                    };
                    if let Some(server) = self.socket.lock().unwrap().as_ref() {
                        server.publish(&record);
                    }
                    let _ = self.event_tx.send(EngineEvent::Frame(record));
                }
                DecoderEvent::HeaderError { snr_db, cfo_hz, reason } => {
                    self.notice(format!("Header lost: {reason} (SNR {snr_db:.1} dB, CFO {cfo_hz:+.0} Hz)"));
                }
                DecoderEvent::SyncMismatch { found, snr_db, cfo_hz } => {
                    self.notice(format!(
                        "Preamble with sync word 0x{found:02X} ignored (SNR {snr_db:.1} dB, CFO {cfo_hz:+.0} Hz)"
                    ));
                }
            }
        }
    }
}

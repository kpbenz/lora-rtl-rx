//! The main window.

use egui::{
    Align, Color32, ColorImage, Layout, RichText, Sense, Stroke, TextureHandle, TextureOptions,
};
use egui_extras::{Column, TableBuilder};
use lora_rtl_rx::dsp::ChannelFilter;
use lora_rtl_rx::engine::{
    Command, Engine, EngineConfig, EngineEvent, EngineStats, FrameRecord, SharedSocket,
    SourceConfig, SPECTRUM_SIZE,
};
use lora_rtl_rx::lora::{Ldro, LoraParams, BANDWIDTHS_HZ};
use lora_rtl_rx::socket::{hex, OutputFormat, SocketServer, SocketSettings};
use lora_rtl_rx::source::rtlsdr::list_devices;
use lora_rtl_rx::source::{
    DeviceInfo, FileSettings, IqFormat, RadioSettings, SimSettings, SourceKind,
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const SETTINGS_KEY: &str = "settings";
const MAX_FRAMES: usize = 5000;
const MAX_LOG: usize = 500;
const WATERFALL_ROWS: usize = 240;
const SAMPLE_RATES: [u32; 9] = [
    1_000_000, 1_024_000, 1_200_000, 1_400_000, 1_800_000, 2_000_000, 2_048_000, 2_400_000,
    2_560_000,
];

const OK: Color32 = Color32::from_rgb(92, 200, 140);
const BAD: Color32 = Color32::from_rgb(240, 100, 90);
const WARN: Color32 = Color32::from_rgb(235, 180, 80);
const ACCENT: Color32 = Color32::from_rgb(90, 190, 220);
const MUTED: Color32 = Color32::from_rgb(150, 156, 166);

#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
struct Settings {
    source: SourceKind,
    radio: RadioSettings,
    file: FileSettings,
    sim: SimSettings,
    lora: LoraParams,
    rf_mhz: f64,
    offset_khz: f64,
    socket: SocketSettings,
    waterfall_auto: bool,
    waterfall_floor: f32,
    waterfall_top: f32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            source: SourceKind::RtlSdr,
            radio: RadioSettings::default(),
            file: FileSettings::default(),
            sim: SimSettings::default(),
            lora: LoraParams::default(),
            rf_mhz: 915.0,
            offset_khz: 200.0,
            socket: SocketSettings::default(),
            waterfall_auto: true,
            waterfall_floor: -90.0,
            waterfall_top: -30.0,
        }
    }
}

impl Settings {
    fn sample_rate(&self) -> f64 {
        match self.source {
            SourceKind::RtlSdr => self.radio.sample_rate as f64,
            SourceKind::File => self.file.sample_rate,
            SourceKind::Simulator => self.sim.sample_rate,
        }
    }

    fn engine_config(&self) -> EngineConfig {
        EngineConfig {
            source: match self.source {
                SourceKind::RtlSdr => SourceConfig::Rtl(self.radio.clone()),
                SourceKind::File => SourceConfig::File(self.file.clone()),
                SourceKind::Simulator => SourceConfig::Simulator(self.sim.clone()),
            },
            lora: self.lora.clone(),
            rf_hz: self.rf_mhz * 1e6,
            offset_hz: self.offset_khz * 1e3,
        }
    }

    /// Anything that would stop this configuration from working.
    fn problem(&self) -> Option<String> {
        if let Err(e) = self.lora.validate() {
            return Some(e);
        }
        let rate = self.sample_rate();
        let bw = self.lora.bw();
        if let Err(e) = ChannelFilter::oversampling(rate, bw) {
            return Some(e);
        }
        if self.offset_khz.abs() * 1e3 + bw / 2.0 > rate / 2.0 {
            return Some(format!(
                "The tuning offset puts the channel outside the {:.0} kHz sampled band.",
                rate / 1e3
            ));
        }
        if self.source == SourceKind::File && self.file.path.as_os_str().is_empty() {
            return Some("Choose an IQ file to replay.".into());
        }
        None
    }

    /// Things that will work, but less well than they could.
    fn advice(&self) -> Option<String> {
        let rate = self.sample_rate();
        let bw = self.lora.bw();
        if self.source == SourceKind::RtlSdr {
            if self.offset_khz.abs() * 1e3 < bw / 2.0 + 10e3 {
                return Some(
                    "The channel overlaps the dongle's DC spike. Increase the tuning offset."
                        .into(),
                );
            }
            if self.radio.sample_rate > 2_400_000 {
                return Some("Rates above 2.4 MS/s often lose samples on RTL-SDR dongles.".into());
            }
        }
        if rate / bw < 3.999 {
            return Some(
                "Sensitivity improves with a sample rate of at least 4 times the bandwidth.".into(),
            );
        }
        None
    }
}

fn bandwidth_label(bw: u32) -> String {
    if bw >= 100_000 {
        format!("{} kHz", bw / 1000)
    } else {
        format!(
            "{:.1} kHz",
            lora_rtl_rx::lora::exact_bandwidth_hz(bw) / 1000f64
        )
    }
}

/// Maps 0..=1 to a perceptually ordered dark-to-bright colour.
fn heat(t: f32) -> Color32 {
    const STOPS: [(f32, [f32; 3]); 10] = [
        (0.000, [8.0, 10.0, 24.0]),
        (0.125, [24.0, 20.0, 67.0]),
        (0.250, [40.0, 30.0, 110.0]),
        (0.375, [85.0, 35.0, 120.0]),
        (0.500, [130.0, 40.0, 130.0]),
        (0.610, [175.0, 60.0, 100.0]),
        (0.720, [220.0, 80.0, 70.0]),
        (0.810, [235.0, 125.0, 60.0]),
        (0.900, [250.0, 170.0, 50.0]),
        (1.000, [252.0, 250.0, 190.0]),
    ];
    let t = t.clamp(0.0, 1.0);
    let upper = STOPS.iter().position(|s| s.0 >= t).unwrap_or(9).max(1);
    let (a, b) = (STOPS[upper - 1], STOPS[upper]);
    let f = (t - a.0) / (b.0 - a.0);
    let mix = |i: usize| (a.1[i] + (b.1[i] - a.1[i]) * f) as u8;
    Color32::from_rgb(mix(0), mix(1), mix(2))
}

struct Waterfall {
    pixels: Vec<Color32>,
    lut: Vec<Color32>,
    texture: Option<TextureHandle>,
    dirty: bool,
}

impl Waterfall {
    fn new() -> Self {
        Self {
            pixels: vec![heat(0.0); SPECTRUM_SIZE * WATERFALL_ROWS],
            lut: (0..256).map(|i| heat(i as f32 / 255.0)).collect(),
            texture: None,
            dirty: true,
        }
    }

    /// Scrolls down and writes the newest line at the top.
    fn add_line(&mut self, line: &[f32], floor: f32, top: f32) {
        self.pixels
            .copy_within(0..SPECTRUM_SIZE * (WATERFALL_ROWS - 1), SPECTRUM_SIZE);
        let scale = 255.0 / (top - floor).max(1.0);
        for (pixel, &db) in self.pixels[..SPECTRUM_SIZE].iter_mut().zip(line) {
            *pixel = self.lut[((db - floor) * scale).clamp(0.0, 255.0) as usize];
        }
        self.dirty = true;
    }

    fn texture(&mut self, ctx: &egui::Context) -> &TextureHandle {
        let image =
            |pixels: &[Color32]| ColorImage::new([SPECTRUM_SIZE, WATERFALL_ROWS], pixels.to_vec());
        if self.texture.is_none() {
            self.texture =
                Some(ctx.load_texture("waterfall", image(&self.pixels), TextureOptions::LINEAR));
            self.dirty = false;
        }
        let texture = self.texture.as_mut().expect("just created");
        if self.dirty {
            texture.set(image(&self.pixels), TextureOptions::LINEAR);
            self.dirty = false;
        }
        texture
    }
}

pub struct App {
    settings: Settings,
    engine: Option<Engine>,
    socket: SharedSocket,
    socket_error: Option<String>,
    frames: VecDeque<FrameRecord>,
    selected: Option<u64>,
    good_frames: u64,
    bad_frames: u64,
    follow: bool,
    payload_as_hex: bool,
    log: VecDeque<String>,
    stats: EngineStats,
    error: Option<String>,
    devices: Vec<DeviceInfo>,
    device_error: Option<String>,
    gains: Vec<i32>,
    tuner: Option<String>,
    waterfall: Waterfall,
    waterfall_settled: bool,
    sync_text: String,
}

impl App {
    pub fn new(creation: &eframe::CreationContext<'_>) -> Self {
        let mut style = (*creation.egui_ctx.global_style()).clone();
        style.spacing.item_spacing = egui::vec2(8.0, 6.0);
        style.spacing.interact_size.y = 24.0;
        style.visuals = egui::Visuals::dark();
        style.visuals.selection.bg_fill = Color32::from_rgb(30, 92, 116);
        style.visuals.hyperlink_color = ACCENT;
        creation.egui_ctx.set_global_style(style);

        let settings: Settings = creation
            .storage
            .and_then(|s| s.get_string(SETTINGS_KEY))
            .and_then(|text| serde_json::from_str(&text).ok())
            .unwrap_or_default();
        let (devices, device_error) = match list_devices() {
            Ok(devices) => (devices, None),
            Err(e) => (Vec::new(), Some(e)),
        };
        let sync_text = format!("{:02X}", settings.lora.sync_word);
        let mut app = Self {
            settings,
            engine: None,
            socket: Arc::new(Mutex::new(None)),
            socket_error: None,
            frames: VecDeque::new(),
            selected: None,
            good_frames: 0,
            bad_frames: 0,
            follow: true,
            payload_as_hex: false,
            log: VecDeque::new(),
            stats: EngineStats::default(),
            error: None,
            devices,
            device_error,
            gains: Vec::new(),
            tuner: None,
            waterfall: Waterfall::new(),
            waterfall_settled: false,
            sync_text,
        };
        app.sync_socket();
        app
    }

    fn add_log(&mut self, text: String) {
        let time = chrono::Local::now().format("%H:%M:%S");
        self.log.push_front(format!("{time}  {text}"));
        self.log.truncate(MAX_LOG);
    }

    /// Makes the running socket match the settings.
    fn sync_socket(&mut self) {
        let wanted = self.settings.socket.clone();
        let mut slot = self.socket.lock().unwrap();
        if !wanted.enabled {
            *slot = None;
            self.socket_error = None;
            return;
        }
        if let Some(server) = slot.as_mut() {
            let current = server.settings();
            if current.bind == wanted.bind && current.port == wanted.port {
                server.set_output(wanted.format, wanted.send_bad_frames);
                return;
            }
        }
        *slot = None;
        match SocketServer::start(wanted) {
            Ok(server) => {
                *slot = Some(server);
                self.socket_error = None;
            }
            Err(e) => self.socket_error = Some(e),
        }
    }

    fn start(&mut self) {
        self.error = None;
        self.stats = EngineStats::default();
        self.waterfall_settled = false;
        match Engine::start(self.settings.engine_config(), Arc::clone(&self.socket)) {
            Ok(engine) => {
                self.engine = Some(engine);
                self.add_log(format!("Started at {:.3} MHz", self.settings.rf_mhz));
            }
            Err(e) => self.error = Some(e),
        }
    }

    fn stop(&mut self) {
        if let Some(engine) = self.engine.take() {
            drop(engine);
            self.add_log("Stopped".into());
        }
    }

    fn poll_engine(&mut self) {
        let Some(engine) = &self.engine else { return };
        let events: Vec<EngineEvent> = engine.events.try_iter().collect();
        let mut ended = None;
        for event in events {
            match event {
                EngineEvent::Frame(record) => {
                    match record.frame.crc_ok {
                        Some(false) => self.bad_frames += 1,
                        _ => self.good_frames += 1,
                    }
                    self.frames.push_back(record);
                    if self.frames.len() > MAX_FRAMES {
                        self.frames.pop_front();
                    }
                }
                EngineEvent::Waterfall(line) => {
                    if self.settings.waterfall_auto {
                        self.follow_noise_floor(&line);
                    }
                    let (floor, top) = (self.settings.waterfall_floor, self.settings.waterfall_top);
                    self.waterfall.add_line(&line, floor, top);
                }
                EngineEvent::Stats(stats) => self.stats = stats,
                EngineEvent::Notice(text) => self.add_log(text),
                EngineEvent::DeviceReady {
                    tuner,
                    gains,
                    sample_rate,
                } => {
                    self.tuner = Some(tuner.to_string());
                    self.gains = gains;
                    self.add_log(format!("{tuner} tuner at {:.3} MS/s", sample_rate / 1e6));
                }
                EngineEvent::Stopped(reason) => ended = Some(reason),
            }
        }
        if let Some(reason) = ended {
            self.engine = None;
            match reason {
                Some(e) => {
                    self.add_log(e.clone());
                    self.error = Some(e);
                }
                None => self.add_log("Input finished".into()),
            }
        }
    }

    /// Keeps the noise floor dark and leaves room above it for signals.
    fn follow_noise_floor(&mut self, line: &[f32]) {
        let mut sorted = line.to_vec();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted[sorted.len() / 2] - 3.0;
        let s = &mut self.settings;
        let rate = if self.waterfall_settled { 0.1 } else { 1.0 };
        self.waterfall_settled = true;
        s.waterfall_floor += rate * (floor - s.waterfall_floor);
        s.waterfall_top = s.waterfall_floor + 36.0;
    }

    fn send(&self, command: Command) {
        if let Some(engine) = &self.engine {
            engine.send(command);
        }
    }

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let problem = self.settings.problem();
            if self.engine.is_some() {
                if ui
                    .add(
                        egui::Button::new(RichText::new("Stop").strong())
                            .min_size(egui::vec2(84.0, 30.0)),
                    )
                    .clicked()
                {
                    self.stop();
                }
            } else {
                let start =
                    egui::Button::new(RichText::new("Start").strong().color(Color32::BLACK))
                        .fill(OK)
                        .min_size(egui::vec2(84.0, 30.0));
                let response = ui.add_enabled(problem.is_none(), start);
                let response = match &problem {
                    Some(p) => response.on_disabled_hover_text(p.as_str()),
                    None => response,
                };
                if response.clicked() {
                    self.start();
                }
            }
            ui.separator();
            let s = &self.stats;
            let metric = |ui: &mut egui::Ui, label: &str, value: String, colour: Color32| {
                ui.vertical(|ui| {
                    ui.spacing_mut().item_spacing.y = 0.0;
                    ui.label(RichText::new(label).small().color(MUTED));
                    ui.label(RichText::new(value).monospace().color(colour));
                });
                ui.add_space(6.0);
            };
            metric(ui, "FRAMES", self.good_frames.to_string(), OK);
            let bad = self.bad_frames;
            metric(
                ui,
                "CRC FAILED",
                bad.to_string(),
                if bad > 0 { BAD } else { Color32::LIGHT_GRAY },
            );
            metric(
                ui,
                "PREAMBLES",
                s.decoder.preambles.to_string(),
                Color32::LIGHT_GRAY,
            );
            metric(
                ui,
                "HEADER LOST",
                s.decoder.header_errors.to_string(),
                Color32::LIGHT_GRAY,
            );
            let running = self.engine.is_some();
            let level = if running {
                format!("{:.1} dBFS", s.channel_dbfs)
            } else {
                "-".into()
            };
            metric(ui, "CH POWER", level, Color32::LIGHT_GRAY);
            let clip_colour = if s.clipped_percent > 0.1 {
                BAD
            } else {
                Color32::LIGHT_GRAY
            };
            metric(
                ui,
                "CLIPPING",
                format!("{:.2} %", s.clipped_percent),
                clip_colour,
            );
            let drop_colour = if s.dropped_samples > 0 {
                BAD
            } else {
                Color32::LIGHT_GRAY
            };
            metric(ui, "DROPPED", s.dropped_samples.to_string(), drop_colour);
            metric(
                ui,
                "CPU",
                format!("{:.0} %", s.load_percent),
                Color32::LIGHT_GRAY,
            );

            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let server = self.socket.lock().unwrap();
                match (&*server, &self.socket_error) {
                    (Some(s), _) => {
                        let clients = s.client_count();
                        let text = format!(
                            "{}:{}  {} client{}",
                            s.settings().bind,
                            s.port(),
                            clients,
                            if clients == 1 { "" } else { "s" }
                        );
                        ui.label(RichText::new(text).monospace().color(if clients > 0 {
                            OK
                        } else {
                            MUTED
                        }));
                    }
                    (None, Some(_)) => {
                        ui.label(RichText::new("socket failed").color(BAD));
                    }
                    (None, None) => {
                        ui.label(RichText::new("socket off").color(MUTED));
                    }
                }
            });
        });
        ui.add_space(4.0);
    }

    fn radio_section(&mut self, ui: &mut egui::Ui) {
        let running = self.engine.is_some();
        let mut retune = false;
        egui::Grid::new("radio").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
            ui.label("Input");
            ui.add_enabled_ui(!running, |ui| {
                egui::ComboBox::from_id_salt("source")
                    .width(170.0)
                    .selected_text(match self.settings.source {
                        SourceKind::RtlSdr => "RTL-SDR dongle",
                        SourceKind::File => "IQ file",
                        SourceKind::Simulator => "Simulator",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.settings.source, SourceKind::RtlSdr, "RTL-SDR dongle");
                        ui.selectable_value(&mut self.settings.source, SourceKind::File, "IQ file");
                        ui.selectable_value(&mut self.settings.source, SourceKind::Simulator, "Simulator");
                    });
            });
            ui.end_row();

            ui.label("Frequency");
            let freq = egui::DragValue::new(&mut self.settings.rf_mhz)
                .speed(0.01)
                .range(24.0..=1766.0)
                .max_decimals(6)
                .suffix(" MHz");
            retune |= ui.add(freq).changed();
            ui.end_row();

            ui.label("Tuning offset").on_hover_text(
                "Tunes the dongle this far from the channel, keeping the signal clear of its DC spike. \
                 The channel is then shifted back digitally.",
            );
            let offset = egui::DragValue::new(&mut self.settings.offset_khz)
                .speed(1.0)
                .range(-1600.0..=1600.0)
                .max_decimals(3)
                .suffix(" kHz");
            retune |= ui.add(offset).changed();
            ui.end_row();

            match self.settings.source {
                SourceKind::RtlSdr => self.rtl_rows(ui, running),
                SourceKind::File => self.file_rows(ui, running),
                SourceKind::Simulator => self.sim_rows(ui, running),
            }
        });
        if retune {
            self.send(Command::Tune {
                rf_hz: self.settings.rf_mhz * 1e6,
                offset_hz: self.settings.offset_khz * 1e3,
            });
        }
    }

    fn rtl_rows(&mut self, ui: &mut egui::Ui, running: bool) {
        ui.label("Device");
        ui.horizontal(|ui| {
            ui.add_enabled_ui(!running, |ui| {
                let index = self.settings.radio.device_index;
                let current = self
                    .devices
                    .iter()
                    .find(|d| d.index == index)
                    .map_or_else(|| "none found".to_string(), |d| d.name.clone());
                egui::ComboBox::from_id_salt("device")
                    .width(130.0)
                    .selected_text(current)
                    .show_ui(ui, |ui| {
                        for d in &self.devices {
                            let label = format!("{}: {} ({})", d.index, d.name, d.serial);
                            ui.selectable_value(
                                &mut self.settings.radio.device_index,
                                d.index,
                                label,
                            );
                        }
                    });
                if ui.small_button("Rescan").clicked() {
                    match list_devices() {
                        Ok(devices) => {
                            self.devices = devices;
                            self.device_error = None;
                        }
                        Err(e) => self.device_error = Some(e),
                    }
                }
            });
        });
        ui.end_row();

        ui.label("Sample rate");
        ui.add_enabled_ui(!running, |ui| {
            egui::ComboBox::from_id_salt("rate")
                .width(170.0)
                .selected_text(format!(
                    "{:.3} MS/s",
                    self.settings.radio.sample_rate as f64 / 1e6
                ))
                .show_ui(ui, |ui| {
                    for rate in SAMPLE_RATES {
                        let label = format!("{:.3} MS/s", rate as f64 / 1e6);
                        ui.selectable_value(&mut self.settings.radio.sample_rate, rate, label);
                    }
                });
        });
        ui.end_row();

        ui.label("Gain");
        let mut gain_changed = ui
            .checkbox(&mut self.settings.radio.auto_gain, "Automatic")
            .changed();
        ui.end_row();
        if !self.settings.radio.auto_gain {
            ui.label("");
            if self.gains.is_empty() {
                let mut db = self.settings.radio.gain_tenths as f32 / 10.0;
                let slider = egui::Slider::new(&mut db, 0.0..=49.6)
                    .step_by(0.1)
                    .suffix(" dB");
                if ui.add(slider).changed() {
                    self.settings.radio.gain_tenths = (db * 10.0).round() as i32;
                    gain_changed = true;
                }
            } else {
                let wanted = self.settings.radio.gain_tenths;
                let mut index = self
                    .gains
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, g)| (*g - wanted).abs())
                    .map_or(0, |(i, _)| i);
                let gains = self.gains.clone();
                let last = gains.len() - 1;
                let slider =
                    egui::Slider::new(&mut index, 0..=last).custom_formatter(move |v, _| {
                        format!("{:.1} dB", gains[v as usize] as f32 / 10.0)
                    });
                if ui.add(slider).changed() {
                    self.settings.radio.gain_tenths = self.gains[index];
                    gain_changed = true;
                }
            }
            ui.end_row();
        }
        if gain_changed {
            self.send(Command::Gain {
                auto: self.settings.radio.auto_gain,
                tenths: self.settings.radio.gain_tenths,
            });
        }

        ui.label("Crystal error").on_hover_text(
            "Correction in ppm. The CFO column of decoded frames shows how far off a transmitter appears.",
        );
        if ui
            .add(
                egui::DragValue::new(&mut self.settings.radio.ppm)
                    .range(-200..=200)
                    .suffix(" ppm"),
            )
            .changed()
        {
            self.send(Command::Ppm(self.settings.radio.ppm));
        }
        ui.end_row();

        ui.label("");
        ui.add_enabled(
            !running,
            egui::Checkbox::new(&mut self.settings.radio.digital_agc, "RTL2832 digital AGC"),
        );
        ui.end_row();

        ui.label("");
        if ui
            .checkbox(&mut self.settings.radio.bias_tee, "Bias tee")
            .changed()
        {
            self.send(Command::BiasTee(self.settings.radio.bias_tee));
        }
        ui.end_row();
    }

    fn file_rows(&mut self, ui: &mut egui::Ui, running: bool) {
        ui.label("File");
        let mut path = self.settings.file.path.display().to_string();
        let edit = egui::TextEdit::singleline(&mut path)
            .id_salt("file-path")
            .desired_width(170.0)
            .hint_text("/path/to/capture.cu8");
        if ui.add_enabled(!running, edit).changed() {
            self.settings.file.path = path.into();
        }
        ui.end_row();

        ui.label("Format");
        ui.add_enabled_ui(!running, |ui| {
            egui::ComboBox::from_id_salt("format")
                .width(170.0)
                .selected_text(self.settings.file.format.name())
                .show_ui(ui, |ui| {
                    for format in IqFormat::ALL {
                        ui.selectable_value(&mut self.settings.file.format, format, format.name());
                    }
                });
        });
        ui.end_row();

        ui.label("Sample rate");
        let rate = egui::DragValue::new(&mut self.settings.file.sample_rate)
            .speed(1000.0)
            .range(1000.0..=100e6)
            .suffix(" S/s");
        ui.add_enabled(!running, rate);
        ui.end_row();

        ui.label("");
        ui.add_enabled(
            !running,
            egui::Checkbox::new(&mut self.settings.file.realtime, "Play in real time"),
        );
        ui.end_row();
        ui.label("");
        ui.add_enabled(
            !running,
            egui::Checkbox::new(&mut self.settings.file.looped, "Loop"),
        );
        ui.end_row();
    }

    fn sim_rows(&mut self, ui: &mut egui::Ui, running: bool) {
        let sim = &mut self.settings.sim;
        ui.label("Sample rate");
        let rate = egui::DragValue::new(&mut sim.sample_rate)
            .speed(1000.0)
            .range(100e3..=3.2e6)
            .suffix(" S/s");
        ui.add_enabled(!running, rate);
        ui.end_row();
        ui.label("SNR");
        ui.add_enabled(
            !running,
            egui::DragValue::new(&mut sim.snr_db)
                .speed(0.1)
                .range(-30.0..=40.0)
                .suffix(" dB"),
        );
        ui.end_row();
        ui.label("Crystal error");
        ui.add_enabled(
            !running,
            egui::DragValue::new(&mut sim.ppm)
                .speed(0.1)
                .range(-60.0..=60.0)
                .suffix(" ppm"),
        );
        ui.end_row();
        ui.label("Interval");
        ui.add_enabled(
            !running,
            egui::DragValue::new(&mut sim.interval_s)
                .speed(0.05)
                .range(0.1..=60.0)
                .suffix(" s"),
        );
        ui.end_row();
    }

    fn lora_section(&mut self, ui: &mut egui::Ui) {
        let before = self.settings.lora.clone();
        let lora = &mut self.settings.lora;
        egui::Grid::new("lora").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
            ui.label("Spreading factor");
            egui::ComboBox::from_id_salt("sf")
                .width(170.0)
                .selected_text(format!("SF{}", lora.sf))
                .show_ui(ui, |ui| {
                    for sf in 5..=12u8 {
                        ui.selectable_value(&mut lora.sf, sf, format!("SF{sf}"));
                    }
                });
            ui.end_row();

            ui.label("Bandwidth");
            egui::ComboBox::from_id_salt("bw")
                .width(170.0)
                .selected_text(bandwidth_label(lora.bw_hz))
                .show_ui(ui, |ui| {
                    for bw in BANDWIDTHS_HZ {
                        ui.selectable_value(&mut lora.bw_hz, bw, bandwidth_label(bw));
                    }
                });
            ui.end_row();

            ui.label("Header");
            if lora.sf < 7 {
                lora.implicit_header = true;
            }
            ui.horizontal(|ui| {
                ui.add_enabled_ui(lora.sf >= 7, |ui| {
                    ui.radio_value(&mut lora.implicit_header, false, "Explicit");
                });
                ui.radio_value(&mut lora.implicit_header, true, "Implicit");
            });
            ui.end_row();

            let implicit = lora.implicit_header;
            ui.label("Coding rate").on_hover_text("An explicit header carries the coding rate, so this applies to implicit frames only.");
            ui.add_enabled_ui(implicit, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    for cr in 1..=4u8 {
                        ui.selectable_value(&mut lora.cr, cr, format!("4/{}", cr + 4));
                    }
                });
            });
            ui.end_row();

            ui.label("Payload length").on_hover_text("Implicit frames only: an explicit header carries the length.");
            let length = egui::DragValue::new(&mut lora.implicit_len).range(1..=255).suffix(" bytes");
            ui.add_enabled(implicit, length);
            ui.end_row();

            ui.label("Payload CRC").on_hover_text("Implicit frames only: an explicit header says whether a CRC follows.");
            ui.add_enabled(implicit, egui::Checkbox::new(&mut lora.implicit_crc, "Present"));
            ui.end_row();

            ui.label("Sync word");
            ui.horizontal(|ui| {
                let enabled = !lora.any_sync_word;
                let edit = egui::TextEdit::singleline(&mut self.sync_text)
                    .id_salt("sync-word")
                    .desired_width(34.0)
                    .char_limit(2);
                if ui.add_enabled(enabled, edit).changed() {
                    if let Ok(value) = u8::from_str_radix(self.sync_text.trim(), 16) {
                        lora.sync_word = value;
                    }
                }
                ui.checkbox(&mut lora.any_sync_word, "Any");
            });
            ui.end_row();

            ui.label("");
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                for (name, value) in [("12 private", 0x12u8), ("34 LoRaWAN", 0x34), ("2B Meshtastic", 0x2b)] {
                    if ui.small_button(name).clicked() {
                        lora.sync_word = value;
                        self.sync_text = format!("{value:02X}");
                    }
                }
            });
            ui.end_row();

            ui.label("Preamble").on_hover_text("Symbols the transmitter sends. Longer preambles are found regardless.");
            ui.add(egui::DragValue::new(&mut lora.preamble_len).range(5..=65535).suffix(" symbols"));
            ui.end_row();

            ui.label("LDRO").on_hover_text("Low data rate optimisation. Auto follows Semtech's rule: on above 16 ms per symbol.");
            ui.horizontal(|ui| {
                ui.radio_value(&mut lora.ldro, Ldro::Auto, "Auto");
                ui.radio_value(&mut lora.ldro, Ldro::On, "On");
                ui.radio_value(&mut lora.ldro, Ldro::Off, "Off");
            });
            ui.end_row();

            ui.label("IQ");
            ui.checkbox(&mut lora.invert_iq, "Inverted (LoRaWAN downlink)");
            ui.end_row();
        });

        let lora = &self.settings.lora;
        let symbol_ms = lora.symbol_time_s() * 1e3;
        let bits = lora.sf as f64 * lora.bw() / lora.bins() as f64;
        ui.label(
            RichText::new(format!(
                "{symbol_ms:.2} ms per symbol, {:.0} bit/s raw{}",
                bits,
                if lora.ldro_active() {
                    ", LDRO active"
                } else {
                    ""
                }
            ))
            .small()
            .color(MUTED),
        );

        if self.settings.lora != before && self.settings.lora.validate().is_ok() {
            self.send(Command::Lora(self.settings.lora.clone()));
        }
    }

    fn socket_section(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;
        let socket = &mut self.settings.socket;
        egui::Grid::new("socket").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
            ui.label("Server");
            changed |= ui.checkbox(&mut socket.enabled, "Accept connections").changed();
            ui.end_row();

            ui.label("Address").on_hover_text("127.0.0.1 is this computer only. 0.0.0.0 accepts connections from the network.");
            let address = ui.add(egui::TextEdit::singleline(&mut socket.bind).id_salt("bind").desired_width(170.0));
            changed |= address.lost_focus();
            ui.end_row();

            ui.label("TCP port");
            let port = ui.add(egui::DragValue::new(&mut socket.port).range(1..=65535).speed(1.0));
            changed |= port.lost_focus() || port.drag_stopped();
            ui.end_row();

            ui.label("Format");
            egui::ComboBox::from_id_salt("output")
                .width(170.0)
                .selected_text(socket.format.name())
                .show_ui(ui, |ui| {
                    for format in OutputFormat::ALL {
                        changed |= ui.selectable_value(&mut socket.format, format, format.name()).changed();
                    }
                });
            ui.end_row();

            ui.label("");
            changed |= ui.checkbox(&mut socket.send_bad_frames, "Include CRC failures").changed();
            ui.end_row();
        });
        if changed {
            self.sync_socket();
        }
        if let Some(e) = &self.socket_error {
            ui.colored_label(BAD, e);
        }
        if let Some(server) = self.socket.lock().unwrap().as_ref() {
            let (sent, dropped) = server.counters();
            ui.label(
                RichText::new(format!("{sent} sent, {dropped} lost to slow clients"))
                    .small()
                    .color(MUTED),
            );
        }
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.add_space(6.0);
                egui::CollapsingHeader::new(RichText::new("Radio").strong())
                    .default_open(true)
                    .show(ui, |ui| {
                        self.radio_section(ui);
                    });
                egui::CollapsingHeader::new(RichText::new("LoRa").strong())
                    .default_open(true)
                    .show(ui, |ui| {
                        self.lora_section(ui);
                    });
                egui::CollapsingHeader::new(RichText::new("Socket output").strong())
                    .default_open(true)
                    .show(ui, |ui| {
                        self.socket_section(ui);
                    });
                egui::CollapsingHeader::new(RichText::new("Waterfall").strong())
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.checkbox(&mut self.settings.waterfall_auto, "Follow the noise floor");
                        ui.add_enabled_ui(!self.settings.waterfall_auto, |ui| {
                            ui.add(
                                egui::Slider::new(
                                    &mut self.settings.waterfall_floor,
                                    -120.0..=-20.0,
                                )
                                .text("Floor dB"),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.settings.waterfall_top, -80.0..=10.0)
                                    .text("Top dB"),
                            );
                        });
                    });

                if self.settings.source == SourceKind::RtlSdr {
                    if let Some(tuner) = &self.tuner {
                        ui.label(
                            RichText::new(format!("Tuner: {tuner}"))
                                .small()
                                .color(MUTED),
                        );
                    }
                }
            });
    }

    /// The most important thing the user should know right now.
    fn banner(&self) -> (Color32, String) {
        if let Some(e) = &self.error {
            return (BAD, e.clone());
        }
        let lora = &self.settings.lora;
        let summary = format!(
            "{:.3} MHz, SF{}, {}, sync word {}",
            self.settings.rf_mhz,
            lora.sf,
            bandwidth_label(lora.bw_hz),
            if lora.any_sync_word {
                "any".to_string()
            } else {
                format!("0x{:02X}", lora.sync_word)
            },
        );
        if self.engine.is_some() {
            return (OK, format!("Receiving on {summary}"));
        }
        if let Some(problem) = self.settings.problem() {
            return (WARN, problem);
        }
        if self.settings.source == SourceKind::RtlSdr {
            if let Some(e) = &self.device_error {
                return (BAD, e.clone());
            }
            if self.devices.is_empty() {
                return (
                    WARN,
                    "No RTL-SDR dongle found. Plug one in and press Rescan.".into(),
                );
            }
        }
        match self.settings.advice() {
            Some(advice) => (WARN, advice),
            None => (MUTED, format!("Ready for {summary}")),
        }
    }

    fn waterfall_view(&mut self, ui: &mut egui::Ui) {
        let height = (ui.available_height() * 0.30).clamp(130.0, 260.0);
        let (rect, response) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), height), Sense::hover());
        let texture_id = self.waterfall.texture(ui.ctx()).id();
        let painter = ui.painter_at(rect);
        let uv = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
        painter.image(texture_id, rect, uv, Color32::WHITE);

        let rate = self.settings.sample_rate();
        let tuned = self.settings.rf_mhz * 1e6 - self.settings.offset_khz * 1e3;
        let x_of = |hz: f64| rect.left() + ((hz - tuned) / rate + 0.5) as f32 * rect.width();

        // The LoRa channel.
        let bw = self.settings.lora.bw();
        let centre = self.settings.rf_mhz * 1e6;
        let band = egui::Rect::from_x_y_ranges(
            x_of(centre - bw / 2.0)..=x_of(centre + bw / 2.0),
            rect.y_range(),
        );
        painter.rect_filled(band, 0.0, Color32::from_rgba_unmultiplied(90, 190, 220, 16));
        let line = Stroke::new(1.0, ACCENT.gamma_multiply(0.7));
        painter.vline(band.left(), rect.y_range(), line);
        painter.vline(band.right(), rect.y_range(), line);

        // Frequency scale along the bottom.
        let font = egui::FontId::monospace(11.0);
        let divisions = 8;
        for i in 0..=divisions {
            let hz = tuned - rate / 2.0 + rate * i as f64 / divisions as f64;
            let x = x_of(hz);
            let y = rect.bottom();
            painter.vline(
                x,
                (y - 6.0)..=y,
                Stroke::new(1.0, Color32::from_white_alpha(120)),
            );
            if i % 2 == 0 {
                let anchor = match i {
                    0 => egui::Align2::LEFT_BOTTOM,
                    8 => egui::Align2::RIGHT_BOTTOM,
                    _ => egui::Align2::CENTER_BOTTOM,
                };
                let label = format!("{:.3}", hz / 1e6);
                let inset = match i {
                    0 => 3.0,
                    8 => -3.0,
                    _ => 0.0,
                };
                let shadow = egui::pos2(x + inset + 1.0, y - 7.0);
                painter.text(shadow, anchor, &label, font.clone(), Color32::BLACK);
                painter.text(
                    egui::pos2(x + inset, y - 8.0),
                    anchor,
                    &label,
                    font.clone(),
                    Color32::WHITE,
                );
            }
        }

        // Frequency under the pointer.
        if let Some(pos) = response.hover_pos() {
            let fraction = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64;
            let hz = tuned - rate / 2.0 + fraction * rate;
            painter.vline(
                pos.x,
                rect.y_range(),
                Stroke::new(1.0, Color32::from_white_alpha(170)),
            );

            let offset_khz = (hz - centre) / 1e3;
            let text = format!("{:.4} MHz   {:+.1} kHz", hz / 1e6, offset_khz);
            let galley =
                painter.layout_no_wrap(text, egui::FontId::monospace(12.0), Color32::WHITE);
            let padding = egui::vec2(6.0, 3.0);
            let mut min = egui::pos2(pos.x + 10.0, rect.top() + 6.0);
            // Flip to the other side of the cursor rather than spill off the edge.
            if min.x + galley.size().x + 2.0 * padding.x > rect.right() - 4.0 {
                min.x = pos.x - 10.0 - galley.size().x - 2.0 * padding.x;
            }
            min.x = min.x.max(rect.left() + 4.0);
            let box_rect = egui::Rect::from_min_size(min, galley.size() + padding * 2.0);
            painter.rect_filled(box_rect, 4.0, Color32::from_black_alpha(205));
            painter.galley(min + padding, galley, Color32::WHITE);
        }
    }

    fn frame_table(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Decoded frames").strong());
            ui.checkbox(&mut self.follow, "Follow newest");
            ui.checkbox(&mut self.payload_as_hex, "Hex");
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui.small_button("Clear").clicked() {
                    self.frames.clear();
                    self.selected = None;
                    self.good_frames = 0;
                    self.bad_frames = 0;
                }
            });
        });
        if self.follow {
            self.selected = self.frames.back().map(|f| f.seq);
        }
        let mut clicked = None;
        let frames = &self.frames;
        let selected = self.selected;
        let as_hex = self.payload_as_hex;
        let mono = |text: String| RichText::new(text).monospace();
        let mut table = TableBuilder::new(ui)
            .striped(true)
            .sense(Sense::click())
            .cell_layout(Layout::left_to_right(Align::Center))
            .column(Column::exact(52.0))
            .column(Column::exact(96.0))
            .column(Column::exact(44.0))
            .column(Column::exact(44.0))
            .column(Column::exact(84.0))
            .column(Column::exact(62.0))
            .column(Column::exact(72.0))
            .column(Column::exact(78.0))
            .column(Column::remainder().clip(true))
            .min_scrolled_height(0.0);
        if self.follow && !frames.is_empty() {
            table = table.scroll_to_row(frames.len() - 1, None);
        }
        table
            .header(22.0, |mut header| {
                for title in [
                    "#", "Time", "Bytes", "CRC", "Mode", "SNR dB", "RSSI dB", "CFO kHz", "Payload",
                ] {
                    header.col(|ui| {
                        ui.label(RichText::new(title).small().color(MUTED));
                    });
                }
            })
            .body(|body| {
                body.rows(22.0, frames.len(), |mut row| {
                    let record = &frames[row.index()];
                    let f = &record.frame;
                    row.set_selected(selected == Some(record.seq));
                    row.col(|ui| {
                        ui.label(mono(record.seq.to_string()));
                    });
                    row.col(|ui| {
                        let time = record.time.with_timezone(&chrono::Local);
                        ui.label(mono(time.format("%H:%M:%S%.3f").to_string()));
                    });
                    row.col(|ui| {
                        ui.label(mono(f.payload.len().to_string()));
                    });
                    row.col(|ui| {
                        let (text, colour) = match f.crc_ok {
                            Some(true) => ("ok", OK),
                            Some(false) => ("BAD", BAD),
                            None => ("-", MUTED),
                        };
                        ui.label(mono(text.into()).color(colour));
                    });
                    row.col(|ui| {
                        ui.label(mono(format!("SF{} 4/{}", f.sf, f.cr + 4)));
                    });
                    row.col(|ui| {
                        ui.label(mono(format!("{:.1}", f.snr_db)));
                    });
                    row.col(|ui| {
                        ui.label(mono(format!("{:.1}", f.rssi_dbfs)));
                    });
                    row.col(|ui| {
                        ui.label(mono(format!("{:+.2}", f.cfo_hz / 1e3)));
                    });
                    row.col(|ui| {
                        let text = if as_hex {
                            hex(&f.payload)
                        } else {
                            printable(&f.payload)
                        };
                        ui.label(mono(text));
                    });
                    if row.response().clicked() {
                        clicked = Some(record.seq);
                    }
                });
            });
        if let Some(seq) = clicked {
            self.selected = Some(seq);
            self.follow = false;
        }
    }

    fn detail_panel(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("events")
            .default_size(290.0)
            .size_range(160.0..=600.0)
            .show(ui, |ui| {
                ui.label(RichText::new("Events").strong());
                egui::ScrollArea::vertical()
                    .id_salt("log")
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        for line in &self.log {
                            ui.label(RichText::new(line).monospace().small().color(MUTED));
                        }
                    });
            });
        let record = self
            .selected
            .and_then(|seq| self.frames.iter().rev().find(|f| f.seq == seq));
        let Some(record) = record else {
            ui.label(RichText::new("Select a frame to see its bytes.").color(MUTED));
            return;
        };
        let f = &record.frame;
        ui.horizontal(|ui| {
            ui.label(RichText::new(format!("Frame {}", record.seq)).strong());
            ui.label(
                RichText::new(format!(
                    "sync 0x{:02X}  clock {:+.1} ppm  FEC {} fixed, {} detected{}",
                    f.sync_word,
                    f.clock_ppm,
                    f.fec_corrected,
                    f.fec_detected,
                    if f.ldro { "  LDRO" } else { "" }
                ))
                .small()
                .color(MUTED),
            );
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui.small_button("Copy hex").clicked() {
                    ui.ctx().copy_text(hex(&f.payload));
                }
            });
        });
        egui::ScrollArea::both()
            .id_salt("hex")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.add(egui::Label::new(RichText::new(hexdump(&f.payload)).monospace()).extend());
            });
    }
}

fn printable(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|&b| {
            if (0x20..0x7f).contains(&b) {
                b as char
            } else {
                '.'
            }
        })
        .collect()
}

fn hexdump(bytes: &[u8]) -> String {
    let mut text = String::with_capacity(bytes.len() * 5);
    for (i, row) in bytes.chunks(16).enumerate() {
        text.push_str(&format!("{:04x}  ", i * 16));
        for column in 0..16 {
            match row.get(column) {
                Some(b) => text.push_str(&format!("{b:02x} ")),
                None => text.push_str("   "),
            }
            if column == 7 {
                text.push(' ');
            }
        }
        text.push(' ');
        text.push_str(&printable(row));
        text.push('\n');
    }
    text
}

impl eframe::App for App {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_engine();
        if self.engine.is_some() {
            ctx.request_repaint_after(Duration::from_millis(33));
        } else {
            // Keep the client count current while idle.
            ctx.request_repaint_after(Duration::from_millis(500));
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("top").show(ui, |ui| self.top_bar(ui));
        // This strip is always there, so messages coming and going never move
        // the controls underneath, even while someone is typing in one.
        egui::Panel::top("banner").show(ui, |ui| {
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                let (colour, text) = self.banner();
                ui.colored_label(colour, text);
                if self.error.is_some() {
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if ui.small_button("Dismiss").clicked() {
                            self.error = None;
                        }
                    });
                }
            });
            ui.add_space(2.0);
        });
        egui::Panel::left("settings")
            .default_size(338.0)
            .size_range(320.0..=440.0)
            .show(ui, |ui| self.side_panel(ui));
        egui::Panel::bottom("detail")
            .default_size(180.0)
            .size_range(110.0..=400.0)
            .show(ui, |ui| {
                ui.add_space(4.0);
                self.detail_panel(ui)
            });
        egui::CentralPanel::default().show(ui, |ui| {
            self.waterfall_view(ui);
            ui.add_space(4.0);
            self.frame_table(ui);
        });
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        if let Ok(text) = serde_json::to_string(&self.settings) {
            storage.set_string(SETTINGS_KEY, text);
        }
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.engine = None;
    }
}

//! Tests of the whole receiver: source thread, processing thread, TCP clients.

use chrono::Utc;
use lora_rtl_rx::engine::{Command, Engine, EngineConfig, EngineEvent, FrameRecord, SharedSocket, SourceConfig};
use lora_rtl_rx::lora::modulator::{modulate, TxImpairments};
use lora_rtl_rx::lora::{Frame, LoraParams};
use lora_rtl_rx::socket::{OutputFormat, SocketServer, SocketSettings};
use lora_rtl_rx::source::{FileSettings, IqFormat, RadioSettings, SimSettings};
use num_complex::Complex32;
use std::io::{BufRead, BufReader, Read};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const RATE: f64 = 1_000_000.0;
const OFFSET: f64 = 200_000.0;

fn temp_path(name: &str) -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!("lora-rtl-rx-{}-{name}", std::process::id()));
    path
}

/// Writes frames as 8 bit samples, as a dongle would deliver them.
fn write_capture(path: &Path, p: &LoraParams, payloads: &[&[u8]]) {
    let gap = vec![Complex32::default(); (RATE * 0.05) as usize];
    let mut samples = gap.clone();
    for payload in payloads {
        samples.extend(modulate(p, payload, 2, true, RATE, TxImpairments { cfo_hz: 6000.0, sfo_ppm: 6.5, ..Default::default() }));
        samples.extend_from_slice(&gap);
    }
    let step = std::f64::consts::TAU * OFFSET / RATE;
    let mut bytes = Vec::with_capacity(samples.len() * 2);
    for (i, s) in samples.iter().enumerate() {
        let phase = step * i as f64;
        let v = s * Complex32::new(phase.cos() as f32, phase.sin() as f32) * 0.3;
        bytes.push((v.re * 128.0 + 127.4).round() as u8);
        bytes.push((v.im * 128.0 + 127.4).round() as u8);
    }
    std::fs::write(path, bytes).unwrap();
}

fn server(format: OutputFormat) -> (SharedSocket, u16) {
    let settings = SocketSettings { port: 0, format, ..Default::default() };
    let server = SocketServer::start(settings).unwrap();
    let port = server.port();
    (Arc::new(Mutex::new(Some(server))), port)
}

fn wait_for_clients(socket: &SharedSocket, count: usize) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while socket.lock().unwrap().as_ref().unwrap().client_count() != count {
        assert!(Instant::now() < deadline, "clients did not connect");
        std::thread::sleep(Duration::from_millis(10));
    }
}

fn file_config(path: &Path, p: &LoraParams) -> EngineConfig {
    EngineConfig {
        source: SourceConfig::File(FileSettings {
            path: path.to_path_buf(),
            format: IqFormat::Cu8,
            sample_rate: RATE,
            realtime: false,
            looped: false,
        }),
        lora: p.clone(),
        rf_hz: 915e6,
        offset_hz: OFFSET,
    }
}

fn wait_until_stopped(engine: &Engine) -> Vec<FrameRecord> {
    let mut frames = Vec::new();
    loop {
        match engine.events.recv_timeout(Duration::from_secs(30)).expect("the engine went quiet") {
            EngineEvent::Frame(f) => frames.push(f),
            EngineEvent::Stopped(reason) => {
                assert_eq!(reason, None);
                return frames;
            }
            _ => {}
        }
    }
}

#[test]
fn frames_reach_every_client_in_every_format() {
    let p = LoraParams { sf: 8, ..Default::default() };
    let payloads: [&[u8]; 3] = [b"first frame", &[0, 1, 2, 0xfe, 0xff], b"third"];
    let path = temp_path("formats.cu8");
    write_capture(&path, &p, &payloads);

    for format in OutputFormat::ALL {
        let (socket, port) = server(format);
        let clients: Vec<TcpStream> = (0..2).map(|_| TcpStream::connect(("127.0.0.1", port)).unwrap()).collect();
        wait_for_clients(&socket, 2);

        let engine = Engine::start(file_config(&path, &p), Arc::clone(&socket)).unwrap();
        let frames = wait_until_stopped(&engine);
        assert_eq!(frames.len(), 3);
        for (frame, payload) in frames.iter().zip(&payloads) {
            assert_eq!(frame.frame.payload, *payload);
            assert_eq!(frame.frame.crc_ok, Some(true));
            assert_eq!(frame.frame.cr, 2);
            assert!((frame.frame.cfo_hz - 6000.0).abs() < 50.0, "{}", frame.frame.cfo_hz);
        }

        // Closing the server ends each stream, so clients can read to the end.
        drop(engine);
        *socket.lock().unwrap() = None;
        for mut client in clients {
            client.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
            let mut received = Vec::new();
            client.read_to_end(&mut received).unwrap();
            check_stream(format, &received, &payloads);
        }
    }
    std::fs::remove_file(path).unwrap();
}

fn check_stream(format: OutputFormat, received: &[u8], payloads: &[&[u8]]) {
    match format {
        OutputFormat::RawBytes => assert_eq!(received, payloads.concat()),
        OutputFormat::HexLines => {
            let text = String::from_utf8(received.to_vec()).unwrap();
            let lines: Vec<&str> = text.lines().collect();
            assert_eq!(lines[1], "000102feff");
            assert_eq!(lines.len(), 3);
        }
        OutputFormat::JsonLines => {
            let lines: Vec<serde_json::Value> = BufReader::new(received)
                .lines()
                .map(|line| serde_json::from_str(&line.unwrap()).unwrap())
                .collect();
            assert_eq!(lines.len(), 3);
            assert_eq!(lines[0]["payload_hex"], "6669727374206672616d65");
            assert_eq!(lines[0]["crc"], "ok");
            assert_eq!(lines[0]["cr"], "4/6");
            assert_eq!(lines[0]["seq"], 1);
            assert_eq!(lines[2]["seq"], 3);
            assert_eq!(lines[0]["freq_hz"], 915_000_000u64);
        }
        OutputFormat::Binary => {
            let mut at = 0;
            for (i, payload) in payloads.iter().enumerate() {
                assert_eq!(&received[at..at + 4], b"LORA");
                let flags = received[at + 4];
                assert_eq!(flags, 3, "CRC present and good");
                assert_eq!(received[at + 5], 8);
                assert_eq!(received[at + 6], 2);
                assert_eq!(received[at + 7], 0x12);
                let bw = u32::from_le_bytes(received[at + 8..at + 12].try_into().unwrap());
                assert_eq!(bw, 125_000);
                let seq = u64::from_le_bytes(received[at + 12..at + 20].try_into().unwrap());
                assert_eq!(seq, i as u64 + 1);
                let length = u16::from_le_bytes(received[at + 24..at + 26].try_into().unwrap()) as usize;
                assert_eq!(&received[at + 26..at + 26 + length], *payload);
                at += 26 + length;
            }
            assert_eq!(at, received.len());
        }
    }
}

fn record(payload: Vec<u8>, crc_ok: Option<bool>) -> FrameRecord {
    FrameRecord {
        seq: 1,
        time: Utc::now(),
        rf_hz: 915e6,
        frame: Frame {
            sample_index: 0,
            sf: 7,
            bw_hz: 125_000,
            cr: 1,
            has_crc: true,
            implicit_header: false,
            ldro: false,
            sync_word: 0x12,
            payload,
            crc_ok,
            snr_db: 1.0,
            rssi_dbfs: -30.0,
            cfo_hz: 0.0,
            clock_ppm: 0.0,
            fec_corrected: 0,
            fec_detected: 0,
        },
    }
}

#[test]
fn a_stalled_client_cannot_block_the_receiver() {
    let (socket, port) = server(OutputFormat::RawBytes);
    let stalled = TcpStream::connect(("127.0.0.1", port)).unwrap();
    wait_for_clients(&socket, 1);
    let guard = socket.lock().unwrap();
    let server = guard.as_ref().unwrap();
    let frame = record(vec![0x55; 255], Some(true));
    let started = Instant::now();
    for _ in 0..50_000 {
        server.publish(&frame);
    }
    assert!(started.elapsed() < Duration::from_secs(2), "publishing blocked for {:?}", started.elapsed());
    let (_, lost) = server.counters();
    assert!(lost > 0, "the unread client's queue should have overflowed");
    drop(stalled);
}

#[test]
fn bad_frames_are_filtered_unless_asked_for() {
    let (socket, port) = server(OutputFormat::HexLines);
    let mut client = TcpStream::connect(("127.0.0.1", port)).unwrap();
    client.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    wait_for_clients(&socket, 1);
    {
        let mut guard = socket.lock().unwrap();
        let server = guard.as_mut().unwrap();
        server.publish(&record(vec![0xaa], Some(false)));
        server.publish(&record(vec![0xbb], None));
        server.set_output(OutputFormat::HexLines, true);
        server.publish(&record(vec![0xcc], Some(false)));
    }
    let mut text = [0u8; 6];
    client.read_exact(&mut text).unwrap();
    assert_eq!(&text, b"bb\ncc\n");
}

#[test]
fn clients_that_leave_are_forgotten() {
    let (socket, port) = server(OutputFormat::JsonLines);
    let client = TcpStream::connect(("127.0.0.1", port)).unwrap();
    wait_for_clients(&socket, 1);
    drop(client);
    wait_for_clients(&socket, 0);
}

#[test]
fn parameters_change_while_running() {
    let p = LoraParams { sf: 7, ..Default::default() };
    let (socket, _) = server(OutputFormat::JsonLines);
    let config = EngineConfig {
        source: SourceConfig::Simulator(SimSettings { snr_db: 20.0, interval_s: 0.15, ..Default::default() }),
        lora: p.clone(),
        rf_hz: 868.1e6,
        offset_hz: OFFSET,
    };
    let engine = Engine::start(config, socket).unwrap();
    let frame_at = |sf: u8| {
        let deadline = Instant::now() + Duration::from_secs(8);
        while Instant::now() < deadline {
            if let Ok(EngineEvent::Frame(f)) = engine.events.recv_timeout(Duration::from_millis(200)) {
                if f.frame.sf == sf && f.frame.crc_ok == Some(true) {
                    return f;
                }
            }
        }
        panic!("no good frame at SF{sf}");
    };
    frame_at(7);
    engine.send(Command::Lora(LoraParams { sf: 9, sync_word: 0x34, ..p.clone() }));
    let f = frame_at(9);
    assert_eq!(f.frame.sync_word, 0x34);
    engine.send(Command::Tune { rf_hz: 915e6, offset_hz: OFFSET });
    let f = frame_at(9);
    assert_eq!(f.rf_hz, 915e6);
}

#[test]
fn a_missing_file_is_reported() {
    let (socket, _) = server(OutputFormat::JsonLines);
    let p = LoraParams::default();
    let error = Engine::start(file_config(&temp_path("absent"), &p), socket).err().unwrap();
    assert!(error.contains("cannot open"), "{error}");
}

#[test]
fn an_impossible_rate_is_reported() {
    let (socket, _) = server(OutputFormat::JsonLines);
    let p = LoraParams { bw_hz: 500_000, ..Default::default() };
    let path = temp_path("rate.cu8");
    std::fs::write(&path, [127u8; 1024]).unwrap();
    let mut config = file_config(&path, &p);
    if let SourceConfig::File(f) = &mut config.source {
        f.sample_rate = 800_000.0;
    }
    config.offset_hz = 0.0;
    let error = Engine::start(config, socket).err().unwrap();
    assert!(error.contains("too low"), "{error}");
    std::fs::remove_file(path).unwrap();
}

/// Runs the real radio code against tools/fake_rtlsdr.c. See tools/check_rtlsdr.sh.
#[test]
#[ignore = "needs LIBRTLSDR_PATH to point at the fake library"]
fn rtlsdr_path_through_the_library_abi() {
    let p = LoraParams { sf: 8, ..Default::default() };
    let path = temp_path("radio.cu8");
    write_capture(&path, &p, &[b"over the radio path", b"second", b"third"]);
    let log = temp_path("radio.log");
    std::env::set_var("FAKE_RTLSDR_IQ", &path);
    std::env::set_var("FAKE_RTLSDR_LOG", &log);

    let devices = lora_rtl_rx::source::rtlsdr::list_devices().unwrap();
    assert_eq!(devices.len(), 1);
    assert_eq!(devices[0].serial, "00000001");

    let (socket, port) = server(OutputFormat::HexLines);
    let client = TcpStream::connect(("127.0.0.1", port)).unwrap();
    wait_for_clients(&socket, 1);
    let radio = RadioSettings { gain_tenths: 400, ppm: 3, sample_rate: RATE as u32, bias_tee: true, ..Default::default() };
    let config = EngineConfig {
        source: SourceConfig::Rtl(radio),
        lora: p,
        rf_hz: 915e6,
        offset_hz: OFFSET,
    };
    let engine = Engine::start(config, socket).unwrap();
    let mut frames = Vec::new();
    let mut ready = None;
    let deadline = Instant::now() + Duration::from_secs(10);
    while frames.len() < 3 && Instant::now() < deadline {
        match engine.events.recv_timeout(Duration::from_millis(200)) {
            Ok(EngineEvent::Frame(f)) => frames.push(f),
            Ok(EngineEvent::DeviceReady { tuner, gains, sample_rate }) => ready = Some((tuner, gains, sample_rate)),
            _ => {}
        }
    }
    assert_eq!(frames.len(), 3);
    assert_eq!(frames[0].frame.payload, b"over the radio path");
    let (tuner, gains, sample_rate) = ready.expect("the device reports itself");
    assert_eq!(tuner, "R820T/R820T2");
    assert_eq!(gains.len(), 29);
    assert_eq!(sample_rate, RATE);

    // Live changes, then a clean stop while the stream is running.
    engine.send(Command::Gain { auto: false, tenths: 300 });
    engine.send(Command::Tune { rf_hz: 916e6, offset_hz: OFFSET });
    engine.send(Command::Ppm(-2));
    std::thread::sleep(Duration::from_millis(200));
    drop(engine);
    drop(client);

    let calls = std::fs::read_to_string(&log).unwrap();
    for expected in [
        "open 0",
        "rate 1000000",
        "ppm 3",
        "freq 914800000", // 915 MHz channel, 200 kHz offset
        "gain_mode 1",
        "gain 402", // nearest gain the tuner supports to 40.0 dB
        "bias_tee 1",
        "reset_buffer",
        "read_async buf_num=12 buf_len=65536",
        "gain 297", // nearest to 30.0 dB, changed while running
        "freq 915800000",
        "ppm -2",
        "cancel_async",
        "read_async returned",
        "close streaming=0",
    ] {
        assert!(calls.contains(expected), "missing `{expected}` in:\n{calls}");
    }
    std::fs::remove_file(path).unwrap();
    std::fs::remove_file(log).unwrap();
}

//! TCP output for post processors.
//!
//! Any number of clients may connect. Each has its own queue and writer thread,
//! so a slow client loses frames rather than holding up the radio.

use crate::engine::FrameRecord;
use crossbeam_channel::{bounded, Sender, TrySendError};
use serde::{Deserialize, Serialize};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// What goes down the socket for each frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputFormat {
    /// One JSON object per line, with the payload and everything known about it.
    JsonLines,
    /// One line per frame: the payload in hex.
    HexLines,
    /// Payload bytes only, with nothing to mark where frames end.
    RawBytes,
    /// A binary header followed by the payload. See the README.
    Binary,
}

impl OutputFormat {
    pub const ALL: [OutputFormat; 4] =
        [OutputFormat::JsonLines, OutputFormat::HexLines, OutputFormat::RawBytes, OutputFormat::Binary];

    pub fn name(self) -> &'static str {
        match self {
            OutputFormat::JsonLines => "JSON lines",
            OutputFormat::HexLines => "Hex lines",
            OutputFormat::RawBytes => "Raw bytes",
            OutputFormat::Binary => "Binary frames",
        }
    }
}

/// Socket settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SocketSettings {
    pub enabled: bool,
    /// Address to listen on. `127.0.0.1` keeps the port private to this machine.
    pub bind: String,
    pub port: u16,
    pub format: OutputFormat,
    /// Forward frames that fail their CRC as well.
    pub send_bad_frames: bool,
}

impl Default for SocketSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            bind: "127.0.0.1".into(),
            port: 5005,
            format: OutputFormat::JsonLines,
            send_bad_frames: false,
        }
    }
}

/// Encodes a frame for the wire.
pub fn encode(record: &FrameRecord, format: OutputFormat) -> Vec<u8> {
    let frame = &record.frame;
    match format {
        OutputFormat::RawBytes => frame.payload.clone(),
        OutputFormat::HexLines => {
            let mut line = hex(&frame.payload);
            line.push('\n');
            line.into_bytes()
        }
        OutputFormat::JsonLines => {
            let mut line = record.to_json().to_string();
            line.push('\n');
            line.into_bytes()
        }
        OutputFormat::Binary => {
            let mut flags = 0u8;
            if frame.has_crc {
                flags |= 1;
            }
            if frame.crc_ok == Some(true) {
                flags |= 2;
            }
            let mut out = Vec::with_capacity(frame.payload.len() + 28);
            out.extend_from_slice(b"LORA");
            out.push(flags);
            out.push(frame.sf);
            out.push(frame.cr);
            out.push(frame.sync_word);
            out.extend_from_slice(&frame.bw_hz.to_le_bytes());
            out.extend_from_slice(&record.seq.to_le_bytes());
            out.extend_from_slice(&frame.snr_db.to_le_bytes());
            out.extend_from_slice(&(frame.payload.len() as u16).to_le_bytes());
            out.extend_from_slice(&frame.payload);
            out
        }
    }
}

pub fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write as _;
    let mut text = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        let _ = write!(text, "{b:02x}");
    }
    text
}

struct Client {
    queue: Sender<Arc<Vec<u8>>>,
    alive: Arc<AtomicBool>,
    stream: TcpStream,
}

#[derive(Default)]
struct Shared {
    clients: Mutex<Vec<Client>>,
    count: AtomicUsize,
    sent: AtomicU64,
    dropped: AtomicU64,
    stop: AtomicBool,
}

/// A running TCP server. Stops when dropped.
pub struct SocketServer {
    shared: Arc<Shared>,
    settings: SocketSettings,
    thread: Option<std::thread::JoinHandle<()>>,
    local_port: u16,
}

impl SocketServer {
    pub fn start(settings: SocketSettings) -> Result<Self, String> {
        let address = format!("{}:{}", settings.bind, settings.port);
        let listener = TcpListener::bind(&address).map_err(|e| format!("cannot listen on {address}: {e}"))?;
        let local_port = listener.local_addr().map_err(|e| e.to_string())?.port();
        listener.set_nonblocking(true).map_err(|e| e.to_string())?;
        let shared = Arc::new(Shared::default());
        let for_thread = Arc::clone(&shared);
        let thread = std::thread::Builder::new()
            .name("socket-accept".into())
            .spawn(move || accept_loop(listener, for_thread))
            .map_err(|e| e.to_string())?;
        Ok(Self { shared, settings, thread: Some(thread), local_port })
    }

    pub fn settings(&self) -> &SocketSettings {
        &self.settings
    }

    /// Changes what is sent without dropping connected clients.
    pub fn set_output(&mut self, format: OutputFormat, send_bad_frames: bool) {
        self.settings.format = format;
        self.settings.send_bad_frames = send_bad_frames;
    }

    pub fn port(&self) -> u16 {
        self.local_port
    }

    pub fn client_count(&self) -> usize {
        self.shared.count.load(Ordering::Relaxed)
    }

    /// Frames delivered to clients' queues, and frames lost to full queues.
    pub fn counters(&self) -> (u64, u64) {
        (self.shared.sent.load(Ordering::Relaxed), self.shared.dropped.load(Ordering::Relaxed))
    }

    /// Queues a frame for every client. Never blocks.
    pub fn publish(&self, record: &FrameRecord) {
        if record.frame.crc_ok == Some(false) && !self.settings.send_bad_frames {
            return;
        }
        let clients = self.shared.clients.lock().unwrap();
        if clients.is_empty() {
            return;
        }
        let bytes = Arc::new(encode(record, self.settings.format));
        for client in clients.iter() {
            match client.queue.try_send(Arc::clone(&bytes)) {
                Ok(()) => {
                    self.shared.sent.fetch_add(1, Ordering::Relaxed);
                }
                Err(TrySendError::Full(_)) => {
                    self.shared.dropped.fetch_add(1, Ordering::Relaxed);
                }
                Err(TrySendError::Disconnected(_)) => {}
            }
        }
    }
}

impl Drop for SocketServer {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn accept_loop(listener: TcpListener, shared: Arc<Shared>) {
    while !shared.stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => add_client(stream, &shared),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(_) => {}
        }
        reap(&shared);
        std::thread::sleep(Duration::from_millis(50));
    }
    let mut clients = shared.clients.lock().unwrap();
    for client in clients.iter() {
        let _ = client.stream.shutdown(std::net::Shutdown::Both);
    }
    clients.clear();
    shared.count.store(0, Ordering::Relaxed);
}

fn add_client(stream: TcpStream, shared: &Arc<Shared>) {
    let _ = stream.set_nonblocking(false);
    let _ = stream.set_nodelay(true);
    let _ = stream.set_write_timeout(Some(Duration::from_secs(5)));
    let _ = stream.set_read_timeout(Some(Duration::from_millis(1)));
    let Ok(mut writer) = stream.try_clone() else { return };
    let (queue, receiver) = bounded::<Arc<Vec<u8>>>(1024);
    let alive = Arc::new(AtomicBool::new(true));
    let alive_for_writer = Arc::clone(&alive);
    let spawned = std::thread::Builder::new().name("socket-client".into()).spawn(move || {
        while let Ok(bytes) = receiver.recv() {
            if writer.write_all(&bytes).is_err() {
                break;
            }
        }
        alive_for_writer.store(false, Ordering::Relaxed);
    });
    if spawned.is_ok() {
        let mut clients = shared.clients.lock().unwrap();
        clients.push(Client { queue, alive, stream });
        shared.count.store(clients.len(), Ordering::Relaxed);
    }
}

/// Removes clients that have hung up, which a writer only learns when it next
/// has something to send. Whatever a client sends us is discarded.
fn reap(shared: &Arc<Shared>) {
    let mut clients = shared.clients.lock().unwrap();
    clients.retain_mut(|client| {
        if !client.alive.load(Ordering::Relaxed) {
            return false;
        }
        let mut sink = [0u8; 256];
        match client.stream.read(&mut sink) {
            Ok(0) => false,
            Ok(_) => true,
            Err(e) => matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut),
        }
    });
    shared.count.store(clients.len(), Ordering::Relaxed);
}

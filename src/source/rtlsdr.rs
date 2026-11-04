//! RTL-SDR input through librtlsdr.
//!
//! The library is loaded when first needed rather than linked at build time.
//! That keeps the build free of C dependencies, and the program still runs
//! without it for replaying recordings. Set `LIBRTLSDR_PATH` to use a library in
//! an unusual place.

use super::SourceMessage;
use crossbeam_channel::{Sender, TrySendError};
use libloading::Library;
use num_complex::Complex32;
use serde::{Deserialize, Serialize};
use std::ffi::{c_char, c_int, c_void, CStr};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::JoinHandle;

/// Radio settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RadioSettings {
    pub device_index: u32,
    pub sample_rate: u32,
    /// Let the tuner choose its gain. Poor for bursts: it reacts to each one.
    pub auto_gain: bool,
    /// Tuner gain in tenths of a dB.
    pub gain_tenths: i32,
    /// Crystal correction in parts per million.
    pub ppm: i32,
    /// The RTL2832's own digital AGC.
    pub digital_agc: bool,
    pub bias_tee: bool,
}

impl Default for RadioSettings {
    fn default() -> Self {
        Self {
            device_index: 0,
            sample_rate: 1_000_000,
            auto_gain: false,
            gain_tenths: 402,
            ppm: 0,
            digital_agc: false,
            bias_tee: false,
        }
    }
}

/// One dongle on the USB bus.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    pub index: u32,
    pub name: String,
    pub serial: String,
}

type Dev = *mut c_void;
type AsyncCallback = unsafe extern "C" fn(*mut u8, u32, *mut c_void);

/// Function pointers from librtlsdr. `_lib` keeps them valid.
struct Api {
    get_device_count: unsafe extern "C" fn() -> u32,
    get_device_name: unsafe extern "C" fn(u32) -> *const c_char,
    get_device_usb_strings: unsafe extern "C" fn(u32, *mut c_char, *mut c_char, *mut c_char) -> c_int,
    open: unsafe extern "C" fn(*mut Dev, u32) -> c_int,
    close: unsafe extern "C" fn(Dev) -> c_int,
    set_center_freq: unsafe extern "C" fn(Dev, u32) -> c_int,
    set_freq_correction: unsafe extern "C" fn(Dev, c_int) -> c_int,
    get_tuner_type: unsafe extern "C" fn(Dev) -> c_int,
    get_tuner_gains: unsafe extern "C" fn(Dev, *mut c_int) -> c_int,
    set_tuner_gain: unsafe extern "C" fn(Dev, c_int) -> c_int,
    set_tuner_gain_mode: unsafe extern "C" fn(Dev, c_int) -> c_int,
    set_sample_rate: unsafe extern "C" fn(Dev, u32) -> c_int,
    get_sample_rate: unsafe extern "C" fn(Dev) -> u32,
    set_agc_mode: unsafe extern "C" fn(Dev, c_int) -> c_int,
    reset_buffer: unsafe extern "C" fn(Dev) -> c_int,
    read_async: unsafe extern "C" fn(Dev, AsyncCallback, *mut c_void, u32, u32) -> c_int,
    cancel_async: unsafe extern "C" fn(Dev) -> c_int,
    /// Missing from older librtlsdr builds.
    set_bias_tee: Option<unsafe extern "C" fn(Dev, c_int) -> c_int>,
    _lib: Library,
}

fn library_names() -> Vec<String> {
    let mut names = Vec::new();
    if let Ok(path) = std::env::var("LIBRTLSDR_PATH") {
        names.push(path);
    }
    if cfg!(target_os = "windows") {
        names.extend(["rtlsdr.dll", "librtlsdr.dll"].map(String::from));
    } else if cfg!(target_os = "macos") {
        names.extend(
            [
                "librtlsdr.dylib",
                "/opt/homebrew/lib/librtlsdr.dylib",
                "/usr/local/lib/librtlsdr.dylib",
            ]
            .map(String::from),
        );
    } else {
        names.extend(["librtlsdr.so.2", "librtlsdr.so.0", "librtlsdr.so"].map(String::from));
    }
    names
}

fn load_api() -> Result<Api, String> {
    let mut lib = None;
    let names = library_names();
    for name in &names {
        // SAFETY: loading a library runs its initialisers. librtlsdr has none
        // beyond the C runtime's, and the user chooses LIBRTLSDR_PATH.
        if let Ok(l) = unsafe { Library::new(name) } {
            lib = Some(l);
            break;
        }
    }
    let lib = lib.ok_or_else(|| {
        format!(
            "librtlsdr was not found (tried {}). Install it, for example with `sudo apt install librtlsdr0`.",
            names.join(", ")
        )
    })?;

    macro_rules! sym {
        ($name:literal) => {
            // SAFETY: each field's type matches the prototype in rtl-sdr.h.
            *unsafe { lib.get($name.as_bytes()) }.map_err(|e| format!("librtlsdr: {e}"))?
        };
    }
    // SAFETY: as above.
    let set_bias_tee = unsafe { lib.get(b"rtlsdr_set_bias_tee") }.ok().map(|s| *s);
    Ok(Api {
        get_device_count: sym!("rtlsdr_get_device_count"),
        get_device_name: sym!("rtlsdr_get_device_name"),
        get_device_usb_strings: sym!("rtlsdr_get_device_usb_strings"),
        open: sym!("rtlsdr_open"),
        close: sym!("rtlsdr_close"),
        set_center_freq: sym!("rtlsdr_set_center_freq"),
        set_freq_correction: sym!("rtlsdr_set_freq_correction"),
        get_tuner_type: sym!("rtlsdr_get_tuner_type"),
        get_tuner_gains: sym!("rtlsdr_get_tuner_gains"),
        set_tuner_gain: sym!("rtlsdr_set_tuner_gain"),
        set_tuner_gain_mode: sym!("rtlsdr_set_tuner_gain_mode"),
        set_sample_rate: sym!("rtlsdr_set_sample_rate"),
        get_sample_rate: sym!("rtlsdr_get_sample_rate"),
        set_agc_mode: sym!("rtlsdr_set_agc_mode"),
        reset_buffer: sym!("rtlsdr_reset_buffer"),
        read_async: sym!("rtlsdr_read_async"),
        cancel_async: sym!("rtlsdr_cancel_async"),
        set_bias_tee,
        _lib: lib,
    })
}

fn api() -> Result<&'static Api, String> {
    static API: OnceLock<Result<Api, String>> = OnceLock::new();
    API.get_or_init(load_api).as_ref().map_err(Clone::clone)
}

fn c_string(buffer: &[c_char]) -> String {
    // SAFETY: the buffer is zero initialised and longer than librtlsdr writes.
    unsafe { CStr::from_ptr(buffer.as_ptr()) }.to_string_lossy().trim().to_string()
}

/// Lists the dongles present.
pub fn list_devices() -> Result<Vec<DeviceInfo>, String> {
    let api = api()?;
    // SAFETY: these calls take plain values and write within the 256 byte
    // buffers that rtl-sdr.h specifies.
    let devices = unsafe {
        let count = (api.get_device_count)();
        (0..count)
            .map(|index| {
                let name = CStr::from_ptr((api.get_device_name)(index)).to_string_lossy().into_owned();
                let mut manufacturer = [0 as c_char; 256];
                let mut product = [0 as c_char; 256];
                let mut serial = [0 as c_char; 256];
                (api.get_device_usb_strings)(
                    index,
                    manufacturer.as_mut_ptr(),
                    product.as_mut_ptr(),
                    serial.as_mut_ptr(),
                );
                DeviceInfo { index, name, serial: c_string(&serial) }
            })
            .collect()
    };
    Ok(devices)
}

fn open_error(code: c_int, index: u32) -> String {
    match code {
        -1 | -2 | -4 | -6 if cfg!(target_os = "linux") => format!(
            "could not open dongle {index} (error {code}). On Linux this usually means the \
             dvb_usb_rtl28xxu kernel driver has claimed it, or a udev rule is missing. See the \
             README's troubleshooting section."
        ),
        -3 => format!("no permission to open dongle {index}. Install the rtl-sdr udev rules."),
        _ => format!("could not open dongle {index} (error {code})"),
    }
}

fn check(code: c_int, action: &str) -> Result<(), String> {
    if code < 0 {
        Err(format!("{action} failed (error {code})"))
    } else {
        Ok(())
    }
}

/// An open dongle.
pub struct RtlDevice {
    api: &'static Api,
    dev: Dev,
    gains: Vec<i32>,
    tuner: i32,
    sample_rate: f64,
    streaming: Mutex<Option<JoinHandle<()>>>,
    /// Samples lost because the processing thread fell behind.
    pub dropped: Arc<AtomicU64>,
}

// SAFETY: librtlsdr serialises access to the device for the calls used here:
// rtl_tcp and others retune from a second thread while `read_async` runs, and
// `cancel_async` exists to be called from another thread. All other mutable
// state here is behind a Mutex.
unsafe impl Send for RtlDevice {}
unsafe impl Sync for RtlDevice {}

const TUNERS: [&str; 7] = ["unknown", "E4000", "FC0012", "FC0013", "FC2580", "R820T/R820T2", "R828D"];

impl RtlDevice {
    /// Opens and configures a dongle, without starting the stream.
    pub fn open(settings: &RadioSettings, tuned_hz: f64) -> Result<Self, String> {
        let api = api()?;
        // SAFETY: takes no arguments.
        if unsafe { (api.get_device_count)() } == 0 {
            return Err("no RTL-SDR dongle found".into());
        }
        let mut dev: Dev = std::ptr::null_mut();
        // SAFETY: `dev` is a valid out pointer for rtlsdr_open.
        let code = unsafe { (api.open)(&mut dev, settings.device_index) };
        if code < 0 || dev.is_null() {
            return Err(open_error(code, settings.device_index));
        }
        let mut device = Self {
            api,
            dev,
            gains: Vec::new(),
            tuner: 0,
            sample_rate: 0.0,
            streaming: Mutex::new(None),
            dropped: Arc::new(AtomicU64::new(0)),
        };
        device.configure(settings, tuned_hz)?;
        Ok(device)
    }

    fn configure(&mut self, s: &RadioSettings, tuned_hz: f64) -> Result<(), String> {
        let (api, dev) = (self.api, self.dev);
        // SAFETY: `dev` came from rtlsdr_open and stays open until Drop. The
        // gain buffer has the length rtlsdr_get_tuner_gains reported.
        unsafe {
            self.tuner = (api.get_tuner_type)(dev);
            let count = (api.get_tuner_gains)(dev, std::ptr::null_mut());
            if count > 0 {
                self.gains = vec![0; count as usize];
                (api.get_tuner_gains)(dev, self.gains.as_mut_ptr());
            }
            check((api.set_sample_rate)(dev, s.sample_rate), "setting the sample rate")?;
            self.sample_rate = (api.get_sample_rate)(dev) as f64;
            if s.ppm != 0 {
                check((api.set_freq_correction)(dev, s.ppm), "setting the ppm correction")?;
            }
            check((api.set_center_freq)(dev, tuned_hz.round() as u32), "tuning")?;
            check((api.set_agc_mode)(dev, s.digital_agc as c_int), "setting the digital AGC")?;
        }
        self.set_gain(s.auto_gain, s.gain_tenths)?;
        self.set_bias_tee(s.bias_tee)?;
        Ok(())
    }

    /// Gains the tuner supports, in tenths of a dB.
    pub fn gains(&self) -> &[i32] {
        &self.gains
    }

    pub fn tuner_name(&self) -> &'static str {
        TUNERS.get(self.tuner as usize).copied().unwrap_or("unknown")
    }

    /// The rate the hardware actually runs at.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    pub fn set_frequency(&self, hz: f64) -> Result<(), String> {
        // SAFETY: the device is open.
        check(unsafe { (self.api.set_center_freq)(self.dev, hz.round() as u32) }, "tuning")
    }

    pub fn set_ppm(&self, ppm: i32) -> Result<(), String> {
        // SAFETY: the device is open.
        let code = unsafe { (self.api.set_freq_correction)(self.dev, ppm) };
        // -2 means the value did not change.
        if code == -2 {
            return Ok(());
        }
        check(code, "setting the ppm correction")
    }

    pub fn set_gain(&self, auto: bool, tenths: i32) -> Result<(), String> {
        let wanted = self
            .gains
            .iter()
            .copied()
            .min_by_key(|g| (g - tenths).abs())
            .unwrap_or(tenths);
        // SAFETY: the device is open.
        unsafe {
            check((self.api.set_tuner_gain_mode)(self.dev, !auto as c_int), "setting the gain mode")?;
            if !auto {
                check((self.api.set_tuner_gain)(self.dev, wanted), "setting the gain")?;
            }
        }
        Ok(())
    }

    pub fn set_bias_tee(&self, on: bool) -> Result<(), String> {
        match self.api.set_bias_tee {
            // SAFETY: the device is open.
            Some(f) => check(unsafe { f(self.dev, on as c_int) }, "setting the bias tee"),
            None if on => Err("this librtlsdr has no bias tee support".into()),
            None => Ok(()),
        }
    }

    /// Starts streaming on a dedicated thread.
    pub fn start(self: &Arc<Self>, tx: Sender<SourceMessage>) -> Result<(), String> {
        // SAFETY: the device is open and not streaming.
        check(unsafe { (self.api.reset_buffer)(self.dev) }, "resetting the buffer")?;
        let device = Arc::clone(self);
        let handle = std::thread::Builder::new()
            .name("rtlsdr".into())
            .spawn(move || device.stream(tx))
            .map_err(|e| e.to_string())?;
        *self.streaming.lock().unwrap() = Some(handle);
        Ok(())
    }

    fn stream(&self, tx: Sender<SourceMessage>) {
        let mut context = StreamContext {
            lut: std::array::from_fn(|i| (i as f32 - 127.4) / 128.0),
            tx: tx.clone(),
            dropped: Arc::clone(&self.dropped),
        };
        // SAFETY: `context` outlives read_async, which only returns once the
        // stream has ended and the callback can no longer run. The buffer length
        // is a multiple of 512 as required.
        let code = unsafe {
            (self.api.read_async)(
                self.dev,
                on_samples,
                &mut context as *mut StreamContext as *mut c_void,
                12,
                65_536,
            )
        };
        let message = if code < 0 {
            SourceMessage::Failed(format!(
                "the dongle stopped streaming (error {code}). Was it unplugged?"
            ))
        } else {
            SourceMessage::End
        };
        let _ = tx.send(message);
    }

    /// Stops the stream and waits for it to finish.
    pub fn stop(&self) {
        let handle = self.streaming.lock().unwrap().take();
        if let Some(handle) = handle {
            // SAFETY: the device is open; cancelling is safe from another thread.
            unsafe { (self.api.cancel_async)(self.dev) };
            let _ = handle.join();
        }
    }
}

impl Drop for RtlDevice {
    fn drop(&mut self) {
        self.stop();
        // SAFETY: the stream has ended and nothing else can reach `dev`.
        unsafe { (self.api.close)(self.dev) };
    }
}

struct StreamContext {
    lut: [f32; 256],
    tx: Sender<SourceMessage>,
    dropped: Arc<AtomicU64>,
}

/// Runs on librtlsdr's USB event loop, so it must not block.
unsafe extern "C" fn on_samples(buf: *mut u8, len: u32, ctx: *mut c_void) {
    // SAFETY: `ctx` is the StreamContext passed to read_async, and `buf` holds
    // `len` bytes for the duration of this call.
    let context = unsafe { &*(ctx as *const StreamContext) };
    let bytes = unsafe { std::slice::from_raw_parts(buf, len as usize) };
    let samples: Vec<Complex32> = bytes
        .chunks_exact(2)
        .map(|iq| Complex32::new(context.lut[iq[0] as usize], context.lut[iq[1] as usize]))
        .collect();
    let count = samples.len() as u64;
    if let Err(TrySendError::Full(_)) = context.tx.try_send(SourceMessage::Samples(samples)) {
        context.dropped.fetch_add(count, Ordering::Relaxed);
    }
}

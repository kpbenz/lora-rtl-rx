//! Where samples come from.

pub mod file;
pub mod rtlsdr;
pub mod simulator;

pub use file::{FileSettings, IqFormat};
pub use rtlsdr::{DeviceInfo, RadioSettings, RtlDevice};
pub use simulator::SimSettings;

use num_complex::Complex32;
use serde::{Deserialize, Serialize};

/// A block of samples, or the end of a finite source.
pub enum SourceMessage {
    Samples(Vec<Complex32>),
    End,
    Failed(String),
}

/// Which source the receiver uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SourceKind {
    RtlSdr,
    File,
    Simulator,
}

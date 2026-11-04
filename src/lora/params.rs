//! LoRa PHY parameters, shared by the decoder, the simulator and the GUI.

use serde::{Deserialize, Serialize};

/// Bandwidths supported by Semtech radios, in hertz.
pub const BANDWIDTHS_HZ: [u32; 10] = [
    7_812, 10_417, 15_625, 20_833, 31_250, 41_667, 62_500, 125_000, 250_000, 500_000,
];

/// The exact value of a nominal bandwidth. Semtech quotes 7.8, 10.4, 20.8 and
/// 41.7 kHz, which are really 500 kHz / 64, / 48, / 24 and / 12.
pub fn exact_bandwidth_hz(bw: u32) -> f64 {
    match bw {
        7_812 | 7_800 => 500_000.0 / 64.0,
        10_417 | 10_400 => 500_000.0 / 48.0,
        20_833 | 20_800 => 500_000.0 / 24.0,
        41_667 | 41_700 => 500_000.0 / 12.0,
        other => other as f64,
    }
}

/// Low data rate optimisation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Ldro {
    /// On when a symbol lasts longer than 16 ms, as Semtech recommend.
    Auto,
    On,
    Off,
}

/// Every parameter that defines a LoRa waveform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoraParams {
    /// Spreading factor, 5 to 12.
    pub sf: u8,
    /// Bandwidth in hertz (one of [`BANDWIDTHS_HZ`]).
    pub bw_hz: u32,
    /// Coding rate as 1..=4 for 4/5..=4/8. Only used in implicit header mode;
    /// an explicit header carries its own.
    pub cr: u8,
    /// Sync word, 0x12 for private networks and 0x34 for LoRaWAN.
    pub sync_word: u8,
    /// Accept any sync word instead of checking `sync_word`.
    pub any_sync_word: bool,
    /// Number of preamble up-chirps the transmitter sends.
    pub preamble_len: u16,
    /// Implicit header mode: no PHY header on air.
    pub implicit_header: bool,
    /// Payload length in bytes, implicit header mode only.
    pub implicit_len: u8,
    /// Whether a payload CRC follows, implicit header mode only.
    pub implicit_crc: bool,
    pub ldro: Ldro,
    /// Receive inverted IQ (for example LoRaWAN downlinks).
    pub invert_iq: bool,
}

impl Default for LoraParams {
    fn default() -> Self {
        Self {
            sf: 7,
            bw_hz: 125_000,
            cr: 1,
            sync_word: 0x12,
            any_sync_word: false,
            preamble_len: 8,
            implicit_header: false,
            implicit_len: 16,
            implicit_crc: true,
            ldro: Ldro::Auto,
            invert_iq: false,
        }
    }
}

impl LoraParams {
    pub fn bins(&self) -> usize {
        1 << self.sf
    }

    pub fn bw(&self) -> f64 {
        exact_bandwidth_hz(self.bw_hz)
    }

    pub fn symbol_time_s(&self) -> f64 {
        self.bins() as f64 / self.bw()
    }

    pub fn ldro_active(&self) -> bool {
        match self.ldro {
            Ldro::On => true,
            Ldro::Off => false,
            Ldro::Auto => self.symbol_time_s() > 0.016,
        }
    }

    /// The two symbol values the sync word is sent as.
    pub fn sync_symbols(&self) -> (usize, usize) {
        let n = self.bins();
        (
            (((self.sync_word >> 4) as usize) << 3) % n,
            (((self.sync_word & 0x0f) as usize) << 3) % n,
        )
    }

    /// Explains why these parameters cannot be decoded, if they cannot.
    pub fn validate(&self) -> Result<(), String> {
        if !(5..=12).contains(&self.sf) {
            return Err(format!("spreading factor {} is outside 5 to 12", self.sf));
        }
        if !(1..=4).contains(&self.cr) {
            return Err(format!("coding rate 4/{} is outside 4/5 to 4/8", self.cr + 4));
        }
        if !BANDWIDTHS_HZ.contains(&self.bw_hz) {
            return Err(format!("{} Hz is not a LoRa bandwidth", self.bw_hz));
        }
        if self.preamble_len < 5 {
            return Err("the preamble must be at least 5 symbols".into());
        }
        if self.sf < 7 && !self.implicit_header {
            return Err(format!(
                "SF{} needs implicit header mode: the SX127x has no explicit header at these spreading factors",
                self.sf
            ));
        }
        if self.implicit_header {
            if self.implicit_len == 0 {
                return Err("the implicit payload length cannot be zero".into());
            }
            if self.implicit_crc && self.implicit_len < 2 {
                return Err("a payload CRC needs at least 2 payload bytes".into());
            }
        }
        Ok(())
    }
}

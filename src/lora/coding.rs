//! The LoRa coding chain: Gray mapping, interleaving, Hamming coding,
//! whitening, the payload CRC and the PHY header.
//!
//! Semtech do not publish this, so it follows the reverse engineering in
//! gr-lora_sdr (Tapparel et al., EPFL), which interoperates with the SX1276,
//! RFM95 and SX1262. `tests/reference_vectors.rs` checks this module symbol for
//! symbol against gr-lora_sdr itself.

use super::params::LoraParams;

/// Fields of the explicit PHY header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub payload_len: u8,
    pub cr: u8,
    pub has_crc: bool,
}

/// Payload bytes with their CRC check result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedPayload {
    pub data: Vec<u8>,
    /// `None` when the frame has no CRC or is too short for one.
    pub crc_ok: Option<bool>,
    /// Codewords the Hamming code repaired.
    pub corrected: u32,
    /// Codewords with an error it could only detect.
    pub detected: u32,
}

pub fn gray_encode(x: u16) -> u16 {
    x ^ (x >> 1)
}

pub fn gray_decode(mut x: u16) -> u16 {
    let mut out = x;
    while x != 0 {
        x >>= 1;
        out ^= x;
    }
    out
}

/// The whitening sequence: an LFSR with polynomial x^8 + x^6 + x^5 + x^4 + 1.
pub fn whitening_sequence() -> [u8; 255] {
    let mut seq = [0u8; 255];
    let mut s: u8 = 0xff;
    for v in seq.iter_mut() {
        *v = s;
        let feedback = ((s >> 7) ^ (s >> 5) ^ (s >> 4) ^ (s >> 3)) & 1;
        s = (s << 1) | feedback;
    }
    seq
}

/// CCITT CRC (polynomial 0x1021, init 0) over all but the last two bytes,
/// XORed with those two bytes. `None` for payloads under two bytes, where the
/// value is undefined.
pub fn payload_crc(data: &[u8]) -> Option<u16> {
    if data.len() < 2 {
        return None;
    }
    let (head, tail) = data.split_at(data.len() - 2);
    let mut crc: u16 = 0;
    for &byte in head {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    Some(crc ^ (((tail[0] as u16) << 8) | tail[1] as u16))
}

/// The 5 bit header checksum over the three nibbles `n`.
pub fn header_checksum(n: [u8; 3]) -> u8 {
    let bit = |nibble: usize, b: u8| (n[nibble] >> b) & 1;
    let c4 = bit(0, 3) ^ bit(0, 2) ^ bit(0, 1) ^ bit(0, 0);
    let c3 = bit(0, 3) ^ bit(1, 3) ^ bit(1, 2) ^ bit(1, 1) ^ bit(2, 0);
    let c2 = bit(0, 2) ^ bit(1, 3) ^ bit(1, 0) ^ bit(2, 3) ^ bit(2, 1);
    let c1 = bit(0, 1) ^ bit(1, 2) ^ bit(1, 0) ^ bit(2, 2) ^ bit(2, 1) ^ bit(2, 0);
    let c0 = bit(0, 0) ^ bit(1, 1) ^ bit(2, 3) ^ bit(2, 2) ^ bit(2, 1) ^ bit(2, 0);
    (c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0
}

/// Encodes a nibble as a `4 + cr` bit codeword, first bit in the MSB.
pub fn hamming_encode(nibble: u8, cr: u8) -> u8 {
    let d = |b: u8| (nibble >> b) & 1;
    let (d0, d1, d2, d3) = (d(0), d(1), d(2), d(3));
    if cr == 1 {
        return (d0 << 4) | (d1 << 3) | (d2 << 2) | (d3 << 1) | (d0 ^ d1 ^ d2 ^ d3);
    }
    let p0 = d0 ^ d1 ^ d2;
    let p1 = d1 ^ d2 ^ d3;
    let p2 = d0 ^ d1 ^ d3;
    let p3 = d0 ^ d2 ^ d3;
    let full = (d0 << 7) | (d1 << 6) | (d2 << 5) | (d3 << 4) | (p0 << 3) | (p1 << 2) | (p2 << 1) | p3;
    full >> (4 - cr)
}

/// Outcome of decoding one codeword.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FecResult {
    Clean,
    Corrected,
    Detected,
}

/// Maximum likelihood decoding: picks the nearest valid codeword. When two are
/// equally near the error can only be detected, so the data bits are kept as
/// they were received.
pub fn hamming_decode(cw: u8, cr: u8) -> (u8, FecResult) {
    let mut best = 0u8;
    let mut best_dist = u32::MAX;
    let mut ties = 0;
    for nibble in 0..16u8 {
        let dist = (hamming_encode(nibble, cr) ^ cw).count_ones();
        if dist < best_dist {
            best_dist = dist;
            best = nibble;
            ties = 1;
        } else if dist == best_dist {
            ties += 1;
        }
    }
    if best_dist == 0 {
        return (best, FecResult::Clean);
    }
    if ties == 1 {
        return (best, FecResult::Corrected);
    }
    // The four data bits sit at the top of the codeword, least significant first.
    let top = cw >> cr;
    let raw = ((top & 1) << 3) | ((top & 2) << 1) | ((top & 4) >> 1) | ((top & 8) >> 3);
    (raw, FecResult::Detected)
}

/// The nibbles carried by a frame in the order they enter the channel coder.
fn frame_nibbles(p: &LoraParams, payload: &[u8], has_crc: bool, cr: u8) -> Vec<u8> {
    let whiten = whitening_sequence();
    let mut nibbles = Vec::with_capacity(payload.len() * 2 + 9);
    if !p.implicit_header {
        let len = payload.len() as u8;
        let n = [len >> 4, len & 0x0f, (cr << 1) | has_crc as u8];
        let check = header_checksum(n);
        nibbles.extend_from_slice(&n);
        nibbles.push(check >> 4);
        nibbles.push(check & 0x0f);
    }
    for (i, &byte) in payload.iter().enumerate() {
        let w = byte ^ whiten[i % 255];
        nibbles.push(w & 0x0f);
        nibbles.push(w >> 4);
    }
    if has_crc {
        if let Some(crc) = payload_crc(payload) {
            for shift in [0, 4, 8, 12] {
                nibbles.push(((crc >> shift) & 0x0f) as u8);
            }
        }
    }
    nibbles
}

/// Encodes a payload into chirp symbol values: the whole transmit coding chain.
/// `cr` and `has_crc` go in the header, or apply as-is in implicit mode.
pub fn encode_symbols(p: &LoraParams, payload: &[u8], cr: u8, has_crc: bool) -> Vec<u16> {
    let sf = p.sf as usize;
    let ldro = p.ldro_active();
    let nibbles = frame_nibbles(p, payload, has_crc, cr);
    let mut symbols = Vec::new();
    let mut pos = 0;
    let mut first = true;
    while first || pos < nibbles.len() {
        let reduced = first || ldro;
        let sf_app = if reduced { sf - 2 } else { sf };
        let cr_app = if first { 4 } else { cr };
        let cw_len = 4 + cr_app as usize;
        let mut cw = vec![0u8; sf_app];
        for slot in cw.iter_mut() {
            if pos < nibbles.len() {
                *slot = hamming_encode(nibbles[pos], cr_app);
                pos += 1;
            }
        }
        for i in 0..cw_len {
            let mut bits = vec![0u8; sf];
            for (j, bit) in bits[..sf_app].iter_mut().enumerate() {
                let row = (i + 2 * sf_app - j - 1) % sf_app;
                *bit = (cw[row] >> (cw_len - 1 - i)) & 1;
            }
            if reduced {
                bits[sf_app] = bits[..sf_app].iter().fold(0, |a, b| a ^ b);
            }
            let value = bits.iter().fold(0u16, |acc, &b| (acc << 1) | b as u16);
            symbols.push((gray_decode(value) + 1) & ((1 << sf) - 1) as u16);
        }
        first = false;
    }
    symbols
}

/// Undoes the +1 shift, the rate reduction and the Gray mapping of one symbol.
fn symbol_to_bits(sym: u16, sf: usize, reduced: bool) -> u16 {
    let n = 1u16 << sf;
    if reduced {
        // Reduced rate symbols sit every 4 bins, so round to the nearest one
        // rather than truncating: this is what makes them tolerate drift.
        let x = ((sym + 1) % n) / 4;
        gray_encode(x)
    } else {
        gray_encode((sym + n - 1) % n)
    }
}

/// Deinterleaves `cw_len` symbols into `sf_app` codewords.
fn deinterleave(symbols: &[u16], sf: usize, sf_app: usize, reduced: bool) -> Vec<u8> {
    let cw_len = symbols.len();
    let mut cw = vec![0u8; sf_app];
    for (i, &sym) in symbols.iter().enumerate() {
        let value = symbol_to_bits(sym, sf, reduced);
        for j in 0..sf_app {
            let bit = ((value >> (sf_app - 1 - j)) & 1) as u8;
            let row = (i + 2 * sf_app - j - 1) % sf_app;
            cw[row] |= bit << (cw_len - 1 - i);
        }
    }
    cw
}

/// Hamming-decoded nibbles of a block, with the running error counts.
fn decode_block(
    symbols: &[u16],
    sf: usize,
    reduced: bool,
    cr: u8,
    out: &mut Vec<u8>,
    corrected: &mut u32,
    detected: &mut u32,
) {
    let sf_app = if reduced { sf - 2 } else { sf };
    for cw in deinterleave(symbols, sf, sf_app, reduced) {
        let (nibble, result) = hamming_decode(cw, cr);
        match result {
            FecResult::Clean => {}
            FecResult::Corrected => *corrected += 1,
            FecResult::Detected => *detected += 1,
        }
        out.push(nibble);
    }
}

/// Decodes the first 8 symbols. Returns the header (if explicit and valid) and
/// the nibbles of that block.
pub fn decode_first_block(
    p: &LoraParams,
    symbols: &[u16],
) -> (Result<Option<Header>, String>, Vec<u8>) {
    let sf = p.sf as usize;
    let mut nibbles = Vec::new();
    let (mut corrected, mut detected) = (0, 0);
    decode_block(&symbols[..8], sf, true, 4, &mut nibbles, &mut corrected, &mut detected);
    if p.implicit_header {
        return (Ok(None), nibbles);
    }
    let n = [nibbles[0], nibbles[1], nibbles[2]];
    let check = ((nibbles[3] & 1) << 4) | nibbles[4];
    if check != header_checksum(n) & 0x1f {
        return (Err("header checksum mismatch".into()), nibbles);
    }
    let header = Header {
        payload_len: (n[0] << 4) | n[1],
        cr: n[2] >> 1,
        has_crc: n[2] & 1 != 0,
    };
    if header.payload_len == 0 {
        return (Err("header declares an empty payload".into()), nibbles);
    }
    if !(1..=4).contains(&header.cr) {
        return (Err(format!("header declares coding rate {}", header.cr)), nibbles);
    }
    (Ok(Some(header)), nibbles)
}

/// Total symbols after the sync sequence for a frame of this shape.
pub fn frame_symbols(p: &LoraParams, payload_len: usize, cr: u8, has_crc: bool) -> usize {
    let sf = p.sf as usize;
    let sf_app = if p.ldro_active() { sf - 2 } else { sf };
    let header = if p.implicit_header { 0 } else { 5 };
    let crc = if has_crc { 4 } else { 0 };
    let nibbles = (2 * payload_len + header + crc) as isize;
    let rest = (nibbles - (sf as isize - 2)).max(0) as usize;
    8 + rest.div_ceil(sf_app) * (4 + cr as usize)
}

/// Decodes all symbols of a frame into its payload.
pub fn decode_payload(
    p: &LoraParams,
    symbols: &[u16],
    payload_len: usize,
    cr: u8,
    has_crc: bool,
) -> DecodedPayload {
    let sf = p.sf as usize;
    let ldro = p.ldro_active();
    let (mut corrected, mut detected) = (0, 0);
    let mut nibbles = Vec::with_capacity(2 * payload_len + 16);
    decode_block(&symbols[..8], sf, true, 4, &mut nibbles, &mut corrected, &mut detected);
    let cw_len = 4 + cr as usize;
    for block in symbols[8..].chunks_exact(cw_len) {
        decode_block(block, sf, ldro, cr, &mut nibbles, &mut corrected, &mut detected);
    }
    let skip = if p.implicit_header { 0 } else { 5 };
    let body = &nibbles[skip.min(nibbles.len())..];

    let whiten = whitening_sequence();
    let mut data = Vec::with_capacity(payload_len);
    for i in 0..payload_len {
        let low = body.get(2 * i).copied().unwrap_or(0);
        let high = body.get(2 * i + 1).copied().unwrap_or(0);
        data.push(((high << 4) | low) ^ whiten[i % 255]);
    }

    let mut crc_ok = None;
    if has_crc && payload_len >= 2 {
        let mut received = 0u16;
        for k in 0..4 {
            let nibble = body.get(2 * payload_len + k).copied().unwrap_or(0) as u16;
            received |= nibble << (4 * k);
        }
        crc_ok = payload_crc(&data).map(|crc| crc == received);
    }
    DecodedPayload { data, crc_ok, corrected, detected }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gray_round_trip() {
        for x in 0..4096u16 {
            assert_eq!(gray_decode(gray_encode(x)), x);
        }
    }

    #[test]
    fn whitening_matches_published_start() {
        let seq = whitening_sequence();
        assert_eq!(seq[..8], [0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85]);
    }

    #[test]
    fn hamming_corrects_single_errors_at_rates_4_7_and_4_8() {
        for cr in [3u8, 4] {
            for nibble in 0..16u8 {
                let cw = hamming_encode(nibble, cr);
                assert_eq!(hamming_decode(cw, cr), (nibble, FecResult::Clean));
                for bit in 0..(4 + cr) {
                    let (n, r) = hamming_decode(cw ^ (1 << bit), cr);
                    assert_eq!(n, nibble, "cr {cr} nibble {nibble} bit {bit}");
                    assert_eq!(r, FecResult::Corrected);
                }
            }
        }
    }

    #[test]
    fn hamming_detects_but_keeps_data_at_rate_4_5() {
        for nibble in 0..16u8 {
            let cw = hamming_encode(nibble, 1);
            let (n, r) = hamming_decode(cw ^ 1, 1);
            assert_eq!(n, nibble);
            assert_eq!(r, FecResult::Detected);
        }
    }

    #[test]
    fn round_trip_all_shapes() {
        for sf in 5..=12u8 {
            for cr in 1..=4u8 {
                for &ldro in &[super::super::params::Ldro::Off, super::super::params::Ldro::On] {
                    for len in [2usize, 3, 7, 16, 51, 255] {
                        let p = LoraParams {
                            sf,
                            cr,
                            ldro,
                            implicit_header: sf < 7,
                            implicit_len: len as u8,
                            ..Default::default()
                        };
                        let payload: Vec<u8> = (0..len).map(|i| (i * 37 + 11) as u8).collect();
                        let syms = encode_symbols(&p, &payload, cr, true);
                        assert_eq!(syms.len(), frame_symbols(&p, len, cr, true), "sf{sf} cr{cr} len{len}");
                        let out = decode_payload(&p, &syms, len, cr, true);
                        assert_eq!(out.data, payload, "sf{sf} cr{cr} len{len} {ldro:?}");
                        assert_eq!(out.crc_ok, Some(true));
                        assert_eq!((out.corrected, out.detected), (0, 0));
                    }
                }
            }
        }
    }

    #[test]
    fn explicit_header_round_trip() {
        for sf in 7..=12u8 {
            let p = LoraParams { sf, ..Default::default() };
            let payload = b"hello lora";
            let syms = encode_symbols(&p, payload, 3, true);
            let (header, _) = decode_first_block(&p, &syms[..8]);
            assert_eq!(
                header.unwrap(),
                Some(Header { payload_len: 10, cr: 3, has_crc: true })
            );
        }
    }
}

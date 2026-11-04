//! Checks the coding chain symbol for symbol against gr-lora_sdr, an independent
//! implementation tested with SX1276, RFM95 and SX1262 radios.
//! `tools/gr_reference.py vectors` produced the data.

use lora_rtl_rx::lora::coding::{decode_first_block, decode_payload, encode_symbols, Header};
use lora_rtl_rx::lora::{Ldro, LoraParams};
use serde::Deserialize;

#[derive(Deserialize)]
struct Vector {
    sf: u8,
    cr: u8,
    bw: u32,
    ldro: u8,
    implicit: bool,
    has_crc: bool,
    payload: String,
    stream: Vec<u16>,
}

fn unhex(text: &str) -> Vec<u8> {
    (0..text.len() / 2)
        .map(|i| u8::from_str_radix(&text[2 * i..2 * i + 2], 16).unwrap())
        .collect()
}

#[test]
fn matches_gr_lora_sdr() {
    let text = include_str!("vectors/gr_lora_sdr_symbols.json");
    let vectors: Vec<Vector> = serde_json::from_str(text).unwrap();
    assert!(vectors.len() >= 1760, "expected the full set, found {}", vectors.len());
    let mut crc_checked = 0;
    for v in &vectors {
        let payload = unhex(&v.payload);
        let p = LoraParams {
            sf: v.sf,
            cr: v.cr,
            bw_hz: v.bw,
            ldro: [Ldro::Off, Ldro::On, Ldro::Auto][v.ldro as usize],
            implicit_header: v.implicit,
            implicit_len: payload.len() as u8,
            implicit_crc: v.has_crc,
            ..Default::default()
        };
        let label = format!(
            "sf{} cr{} bw{} ldro{} implicit={} crc={} len={}",
            v.sf, v.cr, v.bw, v.ldro, v.implicit, v.has_crc, payload.len()
        );

        // The encoder reproduces the reference's symbols exactly...
        let mine = encode_symbols(&p, &payload, v.cr, v.has_crc);
        assert_eq!(mine, v.stream, "{label}");

        // ...and the decoder recovers the frame from them.
        if !v.implicit {
            let (header, _) = decode_first_block(&p, &v.stream);
            let expected = Header { payload_len: payload.len() as u8, cr: v.cr, has_crc: v.has_crc };
            assert_eq!(header.unwrap(), Some(expected), "{label}");
        }
        let out = decode_payload(&p, &v.stream, payload.len(), v.cr, v.has_crc);
        assert_eq!(out.data, payload, "{label}");
        assert_eq!((out.corrected, out.detected), (0, 0), "{label}");
        if v.has_crc {
            assert_eq!(out.crc_ok, Some(true), "{label}");
            crc_checked += 1;
        }
    }
    assert_eq!(crc_checked, vectors.len() / 2);
}

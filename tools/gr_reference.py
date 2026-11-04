#!/usr/bin/env python3
"""Generates reference data with gr-lora_sdr (https://github.com/tapparelj/gr-lora_sdr).

  gr_reference.py vectors OUT.json          symbol vectors for tests/reference_vectors.rs
  gr_reference.py iq OUT.cf32 SF CR BW OS LDRO FRAMES PAYLOAD_HEX [SYNC [PADDING]]
                                            a recording with several frames, as cf32

Needs GNU Radio 3.10 and gr-lora_sdr. These signals come from an implementation
that has been tested with SX1276, RFM95 and SX1262 radios and shares no code
with this project, which is why they are worth comparing against.
"""
import json
import math
import os
import random
import sys
import time

import pmt
from gnuradio import blocks, gr, lora_sdr


class Tx(gr.top_block):
    def __init__(self, sf, cr, bw, os_factor, ldro, has_crc, implicit, sync, padding, preamble=8, path=None):
        gr.top_block.__init__(self, "reference tx")
        self.path = path
        rate = int(bw * os_factor)
        self.whitening = lora_sdr.whitening(True, False, ",", "packet_len")
        header = lora_sdr.header(implicit, has_crc, cr)
        crc = lora_sdr.add_crc(has_crc)
        hamming = lora_sdr.hamming_enc(cr, sf)
        interleaver = lora_sdr.interleaver(cr, sf, ldro, bw)
        gray = lora_sdr.gray_demap(sf)
        self.modulate = lora_sdr.modulate(sf, rate, bw, [sync], padding, preamble)
        self.symbols = blocks.vector_sink_i()
        self.iq = blocks.file_sink(gr.sizeof_gr_complex, path) if path else blocks.vector_sink_c()
        self.chain = (self.whitening, header, crc, hamming, interleaver, gray, self.modulate)
        for a, b in zip(self.chain, self.chain[1:]):
            self.connect(a, b)
        self.connect(gray, self.symbols)

    def run_frames(self, payloads, samples, symbols=0):
        """Sends the payloads, stopping once enough has been generated."""
        self.connect(self.modulate, self.iq)
        self.start()
        port = pmt.intern("msg")
        for payload in payloads:
            self.whitening.to_basic_block()._post(port, pmt.intern(payload.hex()))
        deadline = time.time() + 120
        while time.time() < deadline:
            time.sleep(0.01)
            done = os.path.getsize(self.path) // 8 if self.path else len(self.iq.data())
            if len(self.symbols.data()) >= symbols and done >= samples:
                break
        self.stop()
        self.wait()


def frame_symbols(sf, cr, bw, ldro, has_crc, implicit, size):
    active = ldro == 1 or (ldro == 2 and (1 << sf) * 1e3 / bw > 16)
    bits = 2 * size - sf + 2 + (0 if implicit else 5) + (4 if has_crc else 0)
    return 8 + max(math.ceil(bits / (sf - 2 * active)), 0) * (4 + cr)


def symbols_for(sf, cr, bw, ldro, has_crc, implicit, payload):
    # A second frame pushes the first through, then the head block stops the graph.
    count = frame_symbols(sf, cr, bw, ldro, has_crc, implicit, len(payload))
    tx = Tx(sf, cr, bw, 1, ldro, has_crc, implicit, 0x12, 0)
    tx.run_frames([payload], int((8 + 4.25 + count) * (1 << sf)), count)
    return list(tx.symbols.data())[:count]


def vectors(path):
    rng = random.Random(1234)
    out = []
    shapes = [(sf, False) for sf in range(7, 13)] + [(5, True), (6, True), (7, True), (9, True), (12, True)]
    for sf, implicit in shapes:
        for cr in range(1, 5):
            for ldro in (0, 1, 2):
                for bw in (125000, 500000) if ldro == 2 else (125000,):
                    for size in (2, 5, 16, 33, 255 if sf >= 9 else 64):
                        for has_crc in (True, False):
                            payload = bytes(rng.randrange(256) for _ in range(size))
                            syms = symbols_for(sf, cr, bw, ldro, has_crc, implicit, payload)
                            out.append(dict(sf=sf, cr=cr, bw=bw, ldro=ldro, implicit=implicit, has_crc=has_crc,
                                            payload=payload.hex(), stream=syms))
            print(f"sf{sf} implicit={implicit} cr{cr}: {len(out)} vectors", flush=True)
    with open(path, "w") as f:
        json.dump(out, f, separators=(",", ":"))


def iq(args):
    path = args[0]
    sf, cr, bw, os_factor, ldro, frames = (int(v) for v in args[1:7])
    payload = bytes.fromhex(args[7])
    sync = int(args[8], 0) if len(args) > 8 else 0x12
    padding = int(args[9]) if len(args) > 9 else (20 << sf) * os_factor
    tx = Tx(sf, cr, bw, os_factor, ldro, True, False, sync, padding, path=path)
    count = frame_symbols(sf, cr, bw, ldro, True, False, len(payload))
    length = int((8 + 4.25 + count) * (1 << sf) * os_factor) + padding
    tx.run_frames([payload] * frames, frames * length)
    print(f"{frames * length} samples")


if __name__ == "__main__":
    if sys.argv[1] == "vectors":
        vectors(sys.argv[2])
    else:
        iq(sys.argv[2:])

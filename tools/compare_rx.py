#!/usr/bin/env python3
"""Compares this decoder with gr-lora_sdr's receiver on identical noisy signals.

  compare_rx.py [--bin path/to/lora-decode] [--frames N] SF:SNR[,SNR...] ...

The transmitter is gr-lora_sdr. gr-lora_sdr's receiver has no channel filter of
its own, so it is given the noisy signal low-pass filtered to the LoRa band, as
hardware ahead of it would do. See gr_reference.py for the environment needed.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gr_reference  # noqa: E402

BW = 125000
OS = 4
PAYLOAD = bytes(range(0x30, 0x40))


GR_RX = """
import sys
from gnuradio import blocks, filter as gr_filter, gr, lora_sdr
from gnuradio.filter import firdes
path, sf, soft = sys.argv[1], int(sys.argv[2]), sys.argv[3] == "soft"
rate = {rate}
tb = gr.top_block()
src = blocks.file_source(gr.sizeof_gr_complex, path, False)
lpf = gr_filter.fir_filter_ccf(1, firdes.low_pass(1, rate, 0.55 * {bw}, 0.1 * {bw}))
rx = lora_sdr.lora_sdr_lora_rx(center_freq=915000000, bw={bw}, cr=1, has_crc=True, impl_head=False, pay_len=255,
                               samp_rate=rate, sf=sf, sync_word=[0x12], soft_decoding=soft, ldro_mode=2,
                               print_rx=[False, True])
tb.connect(src, lpf, rx)
tb.run()
""".format(rate=BW * OS, bw=BW)


def reference_signal(sf, cr, frames, folder):
    path = os.path.join(folder, f"tx_sf{sf}.cf32")
    padding = (12 << sf) * OS + 1234
    subprocess.run(
        [sys.executable, os.path.join(HERE, "gr_reference.py"), "iq", path, str(sf), str(cr), str(BW), str(OS), "2",
         str(frames), PAYLOAD.hex(), "0x12", str(padding)],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return np.fromfile(path, dtype=np.complex64)


def noisy(signal, snr_db, cfo_bins, sf, seed):
    rng = np.random.default_rng(seed)
    n = np.arange(len(signal))
    rotated = signal * np.exp(2j * np.pi * cfo_bins * n / ((1 << sf) * OS))
    sigma = np.sqrt(OS * 10 ** (-snr_db / 10) / 2)
    noise = sigma * (rng.standard_normal(len(signal)) + 1j * rng.standard_normal(len(signal)))
    return (rotated + noise).astype(np.complex64)


def run_mine(binary, path, sf, cr):
    out = subprocess.run(
        [binary, path, "--format", "cf32", "--rate", str(BW * OS), "--sf", str(sf), "--cr", str(cr)],
        capture_output=True, text=True, check=True).stdout
    good = 0
    for line in out.splitlines():
        frame = json.loads(line)
        good += frame["crc"] == "ok" and frame["payload_hex"] == PAYLOAD.hex()
    return good


def run_gr(path, sf, soft):
    out = subprocess.run([sys.executable, "-c", GR_RX, path, str(sf), "soft" if soft else "hard"],
                         capture_output=True, text=True, timeout=900).stdout
    return out.count("CRC valid!")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bin", default=os.path.join(HERE, "../target/release/lora-decode"))
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--cr", type=int, default=4)
    parser.add_argument("--cfo", type=float, default=2.3, help="carrier offset in bins")
    parser.add_argument("cases", nargs="+")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as folder:
        print(f"frames decoded out of {args.frames}, CR 4/{args.cr + 4}")
        print(f"{'':>6} {'SNR dB':>7} {'this':>6} {'gr hard':>8} {'gr soft':>8}")
        for case in args.cases:
            sf, snrs = case.split(":")
            sf = int(sf)
            signal = reference_signal(sf, args.cr, args.frames, folder)
            for i, snr in enumerate(float(v) for v in snrs.split(",")):
                path = os.path.join(folder, "noisy.cf32")
                noisy(signal, snr, args.cfo, sf, 100 * sf + i).tofile(path)
                mine = run_mine(args.bin, path, sf, args.cr)
                hard = run_gr(path, sf, False)
                soft = run_gr(path, sf, True)
                print(f"SF{sf:<4} {snr:>7.1f} {mine:>6} {hard:>8} {soft:>8}", flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Feeds decoded LoRa frames to Wireshark, which dissects LoRaWAN.

Reads the receiver's JSON lines (socket format "JSON lines") and writes a pcap
stream in which each frame carries a LoRaTap header. Wireshark's LoRaTap
dissector passes frames with sync word 0x34 to its LoRaWAN dissector.

Live, straight into Wireshark:

    python3 lorawan_to_pcap.py | wireshark -k -i -

To a file:

    python3 lorawan_to_pcap.py -w capture.pcap

From a recording, without the GUI:

    lora-decode capture.cu8 --sync 0x34 --offset 200000 | python3 lorawan_to_pcap.py --stdin -w out.pcap

Only frames that passed their CRC are written unless --include-bad is given (the
receiver's own "Include CRC failures" setting must also be on for the socket).

The LoRaTap RSSI fields are left at zero: the receiver measures power relative to
the dongle's full scale, not in dBm, so it has no RSSI to put there. Wireshark
will show them as -139 dBm; ignore that. SNR is filled in.
"""
import argparse
import datetime
import json
import socket
import struct
import sys
import time

LINKTYPE_LORATAP = 270


def pcap_header():
    return struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, LINKTYPE_LORATAP)


def loratap_header(frame):
    """LoRaTap version 0, as Wireshark's dissector reads it (all big endian)."""
    bandwidth = {125000: 1, 250000: 2, 500000: 4}.get(frame["bw_hz"], 0)
    snr = max(-128, min(127, round(frame["snr_db"] * 4)))
    return struct.pack(
        ">BBHIBBBBBbB",
        0,  # version
        0,  # padding
        15,  # header length
        frame["freq_hz"],
        bandwidth,  # in steps of 125 kHz; 0 for narrower bandwidths
        frame["sf"],
        0,  # packet RSSI, unknown
        0,  # max RSSI, unknown
        0,  # current RSSI, unknown
        snr,  # SNR in quarter dB
        int(frame["sync_word"], 16),
    )


def timestamp(frame):
    try:
        when = datetime.datetime.fromisoformat(frame["time"].replace("Z", "+00:00"))
        return when.timestamp()
    except (KeyError, ValueError):
        return time.time()


def record(frame):
    data = loratap_header(frame) + bytes.fromhex(frame["payload_hex"])
    t = timestamp(frame)
    seconds = int(t)
    micros = int(round((t - seconds) * 1e6))
    if micros == 1_000_000:
        seconds, micros = seconds + 1, 0
    return struct.pack("<IIII", seconds, micros, len(data), len(data)) + data


def lines_from_socket(host, port):
    """Yields JSON lines, reconnecting whenever the receiver goes away."""
    while True:
        try:
            with socket.create_connection((host, port)) as connection:
                print(f"connected to {host}:{port}", file=sys.stderr, flush=True)
                yield from connection.makefile("r", encoding="utf-8")
            print("receiver closed the connection, retrying", file=sys.stderr, flush=True)
        except OSError as error:
            print(f"{host}:{port}: {error}, retrying", file=sys.stderr, flush=True)
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--stdin", action="store_true", help="read JSON lines from stdin instead of the socket")
    parser.add_argument("-w", "--write", metavar="FILE", help="write to FILE instead of stdout")
    parser.add_argument("--include-bad", action="store_true", help="also write frames that failed their CRC")
    args = parser.parse_args()

    out = open(args.write, "wb") if args.write else sys.stdout.buffer
    lines = sys.stdin if args.stdin else lines_from_socket(args.host, args.port)
    written = 0
    try:
        out.write(pcap_header())
        out.flush()
        for line in lines:
            line = line.strip()
            if not line:
                continue
            frame = json.loads(line)
            if frame.get("crc") == "bad" and not args.include_bad:
                continue
            out.write(record(frame))
            out.flush()
            written += 1
    except (BrokenPipeError, KeyboardInterrupt):
        pass  # Wireshark closed, or Ctrl-C
    finally:
        if args.write:
            out.close()
            print(f"{written} frames written to {args.write}", file=sys.stderr)


if __name__ == "__main__":
    main()

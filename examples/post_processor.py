#!/usr/bin/env python3
"""Minimal post processor: connects to the receiver and handles each frame.

Set the receiver's socket format to "JSON lines". Reconnects if the receiver goes
away, so it can be started in either order.

    python3 post_processor.py [host] [port]
"""
import json
import socket
import sys
import time


def handle_frame(frame):
    """Replace this with your own processing."""
    payload = bytes.fromhex(frame["payload_hex"])
    text = payload.decode("ascii", errors="replace")
    print(f'#{frame["seq"]} SF{frame["sf"]} {frame["snr_db"]:+.1f} dB crc={frame["crc"]} {len(payload)} bytes: {text!r}',
          flush=True)


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 5005
    while True:
        try:
            with socket.create_connection((host, port)) as connection:
                print(f"connected to {host}:{port}", flush=True)
                for line in connection.makefile("r", encoding="utf-8"):
                    handle_frame(json.loads(line))
        except OSError as error:
            print(f"{error}, retrying", flush=True)
        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

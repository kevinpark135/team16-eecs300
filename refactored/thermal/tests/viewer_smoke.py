#!/usr/bin/env python3
"""Exercise the real viewer through a pseudo-terminal, without a sensor."""
import os
from pathlib import Path
import pty
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
BINARY = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "build/thermaltest"
LOG = ROOT / "build/viewer-smoke.log"


def wait_for(text, process, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if text in LOG.read_text():
            return
        if process.poll() is not None:
            raise RuntimeError("Viewer stopped early:\n" + LOG.read_text())
        time.sleep(0.02)
    raise RuntimeError("Missing viewer output: " + text + "\n" + LOG.read_text())


def send_frame(port, values):
    payload = b"\xaa\xbb" + struct.pack("<768f", *values)
    # Split both the header and payload to exercise partial serial reads.
    for chunk in [payload[:1], payload[1:71], payload[71:]]:
        while chunk:
            chunk = chunk[os.write(port, chunk):]
        time.sleep(0.01)


def main():
    LOG.parent.mkdir(parents=True, exist_ok=True)
    master, slave = pty.openpty()
    environment = os.environ.copy()
    environment["SDL_VIDEODRIVER"] = "dummy"
    environment["SDL_RENDER_DRIVER"] = "software"
    process = None
    try:
        with LOG.open("w") as output:
            process = subprocess.Popen([str(BINARY), os.ttyname(slave)],
                                       stdout=output, stderr=output, env=environment)
            wait_for("opened at 115200 baud", process)
            for i in range(10):
                send_frame(master, [20.0] * 768)
                wait_for("[thermal] frame=" + str(i + 1) + " ", process)
            warm = [20.0] * 768
            for row in range(3, 5):
                for column in range(2, 4):
                    warm[row * 32 + column] = 25.0
            send_frame(master, warm)
            wait_for("status=READY blobCount=1", process)
            send_frame(master, [20.0] * 768)
            wait_for("status=READY blobCount=0", process)
            send_frame(master, [float("nan")] * 768)
            wait_for("status=INVALID blobCount=unavailable", process)
            wait_for("detection=STALE", process)
            os.close(master)
            master = None
            process.wait(timeout=8)
            assert process.returncode == 1
            assert "device disconnected or unavailable" in LOG.read_text()
        print("PASS viewer: fragmented packets, calibration, blob/zero, invalid, stale, disconnect")
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=8)
        if master is not None:
            os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()

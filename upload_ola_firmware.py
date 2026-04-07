#!/usr/bin/env python3
"""
Upload firmware to the OpenLog Artemis (OLA) via arduino-cli.
Target board: SparkFun RedBoard Artemis ATP (SparkFun:apollo3:redboard_artemis_atp)
Sketch:       OpenLog_Artemis_GNSS_Logging_Modified/
"""

import subprocess
import glob
import sys
import os

FQBN = "SparkFun:apollo3:sfe_artemis_atp"
SKETCH_PATH = os.path.join(os.path.dirname(__file__), "OpenLog_Artemis_GNSS_Logging_Modified")


def find_port():
    ports = glob.glob("/dev/cu.usbserial*")
    if not ports:
        print("ERROR: No USB serial port found. Is the OLA plugged in?")
        sys.exit(1)
    if len(ports) > 1:
        print("Multiple ports found:")
        for i, p in enumerate(ports):
            print(f"  {i}) {p}")
        choice = input("Select port number: ").strip()
        return ports[int(choice)]
    print(f"Found port: {ports[0]}")
    return ports[0]


def run(cmd, label):
    print(f"\n--- {label} ---")
    print(f"$ {' '.join(cmd)}\n")
    result = subprocess.run(cmd, text=True)
    if result.returncode != 0:
        print(f"\nERROR: {label} failed (exit code {result.returncode})")
        sys.exit(result.returncode)


def main():
    port = find_port()

    run(
        ["arduino-cli", "compile", "--fqbn", FQBN, SKETCH_PATH],
        "Compiling sketch"
    )

    run(
        ["arduino-cli", "upload", "-p", port, "--fqbn", FQBN, SKETCH_PATH],
        "Uploading firmware"
    )

    print("\nDone. Firmware uploaded successfully.")
    print(f"Monitor: python3 -c \""
          f"import serial,time; s=serial.Serial('{port}',115200,timeout=1); "
          f"[print(s.read(256).decode('utf-8','replace'),end='',flush=True) or time.sleep(0) "
          f"for _ in iter(int,1)]\"")


if __name__ == "__main__":
    main()

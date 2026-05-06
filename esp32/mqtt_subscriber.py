#!/usr/bin/env python3
"""
MQTT subscriber for the RTK wave buoy.

Connects to a local Mosquitto broker, subscribes to all buoy/* topics,
and prints live GPS and status data to the terminal. Optionally appends
GPS fixes to a CSV file for later analysis.

Usage:
    python3 mqtt_subscriber.py                     # broker = localhost
    python3 mqtt_subscriber.py 192.168.1.42        # custom broker IP
    python3 mqtt_subscriber.py --csv gps_log.csv   # also write to CSV
    python3 mqtt_subscriber.py 192.168.1.42 --csv gps_log.csv

Requirements:
    pip install paho-mqtt
"""

import sys
import json
import csv
import datetime
import argparse

import paho.mqtt.client as mqtt

# ── Fix-quality labels (GGA convention used by the ESP32 sketch) ─────────────
FIX_LABELS = {
    0: "No fix",
    1: "GPS",
    4: "RTK Fixed  ✓",
    5: "RTK Float  ~",
}

# ── Callback: broker connection result ───────────────────────────────────────
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print(f"[mqtt] Connected to broker. Listening on buoy/#\n")
        client.subscribe("buoy/#")
    else:
        print(f"[mqtt] Connection refused, reason_code={reason_code}")


# ── Callback: incoming message ────────────────────────────────────────────────
def on_message(client, userdata, msg):
    topic = msg.topic
    local_ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    try:
        data = json.loads(msg.payload.decode())
    except (json.JSONDecodeError, UnicodeDecodeError):
        print(f"[{local_ts}] [{topic}] (non-JSON) {msg.payload!r}")
        return

    if topic == "buoy/gps":
        _handle_gps(local_ts, data, userdata)
    elif topic == "buoy/status":
        _handle_status(local_ts, data)
    else:
        # Forward-compatible: print any future topics generically
        print(f"[{local_ts}] [{topic}] {data}")


def _handle_gps(ts, d, userdata):
    fix      = d.get("fix", 0)
    fix_label = FIX_LABELS.get(fix, f"fix={fix}")
    lat      = d.get("lat", float("nan"))
    lon      = d.get("lon", float("nan"))
    alt      = d.get("alt", float("nan"))
    siv      = d.get("siv", "?")
    hacc     = d.get("hacc", float("nan"))
    utc      = d.get("utc", "?")

    print(
        f"[{ts}]  GPS  "
        f"lat={lat:>12.7f}  lon={lon:>13.7f}  alt={alt:>7.2f} m  "
        f"{fix_label:<16}  SIV={siv:>2}  hAcc={hacc:.3f} m  UTC {utc}"
    )

    writer = userdata.get("csv_writer")
    if writer:
        writer.writerow({
            "local_time": ts,
            "utc":        utc,
            "lat":        lat,
            "lon":        lon,
            "alt_m":      alt,
            "fix":        FIX_LABELS.get(d.get("fix", 0), d.get("fix", "")),
            "siv":        siv,
            "hacc_m":     hacc,
        })
        userdata["csv_file"].flush()


def _handle_status(ts, d):
    carrier   = d.get("carrier", "?")
    siv       = d.get("siv", "?")
    rtcm_b    = d.get("rtcm_bytes", "?")
    reconnects = d.get("reconnects", "?")

    carrier_label = FIX_LABELS.get(carrier, f"carrier={carrier}")
    print(
        f"[{ts}]  STATUS  {carrier_label:<16}  "
        f"SIV={siv}  RTCM={rtcm_b} B  reconnects={reconnects}"
    )


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Live GPS/IMU subscriber for RTK wave buoy"
    )
    parser.add_argument(
        "broker",
        nargs="?",
        default="localhost",
        help="MQTT broker IP address (default: localhost)",
    )
    parser.add_argument(
        "--csv",
        metavar="FILE",
        help="Append GPS fixes to this CSV file",
    )
    args = parser.parse_args()

    userdata: dict = {}

    if args.csv:
        csv_file = open(args.csv, "a", newline="")
        fieldnames = ["local_time", "utc", "lat", "lon", "alt_m", "fix", "siv", "hacc_m"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if csv_file.tell() == 0:
            writer.writeheader()
        userdata["csv_writer"] = writer
        userdata["csv_file"]   = csv_file
        print(f"[csv]  Appending GPS fixes to {args.csv}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.user_data_set(userdata)
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"[mqtt] Connecting to {args.broker}:{1883} ...")
    try:
        client.connect(args.broker, 1883, keepalive=60)
    except OSError as e:
        sys.exit(f"[mqtt] Could not reach broker at {args.broker}:1883 — {e}\n"
                 f"       Is Mosquitto running?  Try: mosquitto -v")

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[mqtt] Stopped by user.")
    finally:
        if "csv_file" in userdata:
            userdata["csv_file"].close()
            print(f"[csv]  Closed {args.csv}")


if __name__ == "__main__":
    main()

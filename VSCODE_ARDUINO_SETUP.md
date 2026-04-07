# VS Code + Arduino CLI Setup Guide

A guide for developing Arduino sketches using VS Code and the Arduino CLI — replacing the Arduino IDE while keeping full Git integration.

---

## Prerequisites

- [VS Code](https://code.visualstudio.com/) installed
- [Arduino CLI](https://arduino.github.io/arduino-cli/latest/installation/) installed
- [Arduino for VS Code extension](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino) by Microsoft

---

## 1. Install Arduino CLI

Using Homebrew (macOS):

```bash
brew install arduino-cli
```

Initialize the config file:

```bash
arduino-cli config init
```

---

## 2. Add ESP32 Board Support

Edit the config to add the ESP32 board URL:

```bash
arduino-cli config add board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Update the index and install the ESP32 core:

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

Verify it installed:

```bash
arduino-cli board listall | grep -i esp32
```

---

## 3. Install the VS Code Extension

In VS Code, open the Extensions panel (`Cmd+Shift+X`) and search for:

```
Arduino
```

Install **Arduino** by Microsoft (`vsciot-vscode.vscode-arduino`).

---

## 4. Configure the Extension

Open VS Code settings (`Cmd+,`) and search for `arduino`. Set:

| Setting | Value |
|---|---|
| `arduino.path` | Path to Arduino CLI (find with `which arduino-cli`) |
| `arduino.useArduinoCli` | `true` |
| `arduino.logLevel` | `info` |

Or add directly to `settings.json`:

```json
{
  "arduino.useArduinoCli": true,
  "arduino.path": "/opt/homebrew/bin",
  "arduino.logLevel": "info"
}
```

---

## 5. Open a Sketch and Select Board/Port

1. Open your `.ino` file in VS Code
2. Use the status bar at the bottom to set:
   - **Board**: `esp32:esp32:esp32` (ESP32 Dev Module)
   - **Port**: `/dev/cu.usbserial-*` (your device's port)

Or use the Command Palette (`Cmd+Shift+P`):
- `Arduino: Select Board`
- `Arduino: Select Serial Port`

---

## 6. Install Libraries

Via Arduino CLI in terminal:

```bash
# Search for a library
arduino-cli lib search SparkFun_u-blox

# Install a library
arduino-cli lib install "SparkFun u-blox GNSS Arduino Library"

# List installed libraries
arduino-cli lib list
```

---

## 7. Compile and Upload

**From the Command Palette** (`Cmd+Shift+P`):
- `Arduino: Verify` — compile only
- `Arduino: Upload` — compile and flash to board

**From terminal using Arduino CLI directly:**

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 esp32_rtk_wifi/

# Upload (replace /dev/cu.usbserial-XXXXX with your port)
arduino-cli upload -p /dev/cu.usbserial-XXXXX --fqbn esp32:esp32:esp32 esp32_rtk_wifi/
```

---

## 8. Serial Monitor

Open the serial monitor from the Command Palette:
- `Arduino: Open Serial Monitor`

Or use the VS Code Arduino extension's built-in monitor in the bottom panel.

Alternatively, from terminal:

```bash
arduino-cli monitor -p /dev/cu.usbserial-XXXXX --config baudrate=115200
```

---

## 9. Git Workflow

Since you're now in VS Code, Git is fully integrated:

- **Source Control panel** (`Cmd+Shift+G`): stage, commit, push, pull
- **Terminal** (`Ctrl+\``): run any git command directly

```bash
git add esp32_rtk_wifi.ino
git commit -m "Update NTRIP client logic"
git push
```

---

## Useful Arduino CLI Commands

```bash
# List connected boards and ports
arduino-cli board list

# List installed cores
arduino-cli core list

# Update everything
arduino-cli core update-index
arduino-cli core upgrade

# Get board FQBN (Fully Qualified Board Name)
arduino-cli board listall esp32
```

---

## Project-Specific Notes

- **OLA port**: `/dev/cu.usbserial-210` @ 115200 baud
- **ESP32 FQBN**: `esp32:esp32:esp32`
- **Secrets**: never commit `secrets.h` — it is gitignored
- **Apollo3 core**: use version specified in hardware warnings (see `README.md`)

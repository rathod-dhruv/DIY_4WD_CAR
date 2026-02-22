# ACEBOTT Car BLE Controller

Control a 4-wheel robot car from your browser using **Web Bluetooth** and an **ESP32**.

## Files

| File | Description |
|---|---|
| `car_ble.ino` | Arduino sketch — runs BLE server on ESP32 |
| `car_controller.html` | Web controller — open in Chrome/Edge |

## Requirements

- **Arduino**: ESP32 board + ESP32 Arduino core v3.x (no extra libraries needed)
- **Browser**: Chrome or Edge (Firefox does not support Web Bluetooth)
- **Page must be served over HTTPS or localhost** — not `file://`

## Quick Start

1. Flash `car_ble.ino` to your ESP32
2. Serve `car_controller.html` via a local server:
   ```
   python -m http.server 8000
   ```
3. Open `http://localhost:8000/car_controller.html` in Chrome
4. Click **Connect to Car** and select `ACEBOTT_CAR_BT`

## Commands

| Command | Effect |
|---|---|
| `F` | Forward |
| `B` | Backward |
| `L` | Spin Left |
| `R` | Spin Right |
| `S` | Stop |
| `V180` | Set speed to 180 (0–255) |
| `T` | Run motor test sequence |
| `X128` | Raw byte to shift register (debug) |

## Pin Wiring (ESP32)

| Pin | GPIO | Purpose |
|---|---|---|
| SHCP | 18 | Shift register clock |
| EN | 16 | Motor enable (active LOW) |
| DATA | 5 | Serial data |
| STCP | 17 | Latch |
| PWM | 19 | Motor speed |

## BLE UUIDs

- **Service**: `0000FFE0-0000-1000-8000-00805F9B34FB`
- **Characteristic**: `0000FFE1-0000-1000-8000-00805F9B34FB`

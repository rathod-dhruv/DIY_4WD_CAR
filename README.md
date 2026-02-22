# ACEBOTT Car BLE Dashboard (ESP32 + Web Bluetooth)

Control an ACEBOTT mecanum car from your browser using **Web Bluetooth (BLE)** and an **ESP32**.  
This version includes a **full dashboard**: driving + strafe, servo, LEDs, buzzer, and **realtime sensor streaming** (ultrasonic obstacle, trace/line sensors, IR raw, and analog temperature).

---

## Files

| File | Description |
|---|---|
| `car_ble.ino` | ESP32 firmware (BLE server + motor control + sensor notify) |
| `car_ble.html` | Browser dashboard (Web Bluetooth controller + sensor UI + console logs) |

> If your HTML file is named differently (e.g. `car_controller_full.html`), rename it to `car_ble.html` or update the URL you open.

---

## Requirements

- **Arduino IDE** with **ESP32 Arduino core v3.x**
- **Browser:** Chrome or Edge (Firefox does not support Web Bluetooth)
- **Serve the HTML over `https://` or `http://localhost`** (Web Bluetooth will not work from `file://`)

---

## Quick Start

1. **Flash firmware**
   - Open `car_ble.ino` in Arduino IDE
   - Select your ESP32 board + correct COM port
   - Upload

2. **Serve the HTML**
   ```bash
   python -m http.server 8000
   ```
3. Open in Chrome/Edge:
   - `http://localhost:8000/car_ble.html`
4. Click **Connect** → choose `ACEBOTT_CAR_BT`

---

## Controls (Commands sent over BLE)

| Command | Effect |
|---|---|
| `F` `B` `L` `R` | Forward / Backward / Spin Left / Spin Right |
| `SL` `SR` | Strafe Left / Strafe Right (mecanum) |
| `S` | Stop |
| `V180` | Set speed (0–255) |
| `P90` | Servo angle (0–180) |
| `D1` / `D0` | LEDs ON / OFF |
| `H1` | Honk (short beep) |
| `X###` | Raw shift-register byte (motor debug) |

---

## Realtime Sensor Streaming (BLE Notify)

The ESP32 pushes JSON to the browser using **notifications** on the **FFE2** characteristic.

### JSON payload
Example:
```json
{"d":23,"l":1,"m":0,"r":1,"ir":1,"t":26.4,"obs":1,"zone":"WARN"}
```

Fields:
- `d` : ultrasonic distance in cm (`999` = no echo)
- `zone`: `SAFE` / `WARN` / `DANGER` / `NO_ECHO`
- `obs`: 1 if obstacle present (WARN/DANGER), else 0
- `l,m,r`: trace sensors (digital)
- `ir`: IR receiver **raw** level (usually `1` when idle)
- `t`: temperature °C (analog sensor on GPIO 27)

---

## Pin Map (current working setup)

### Motors (shift register)
| GPIO | Purpose |
|---:|---|
| 18 | SHCP (shift clock) |
| 16 | EN (motor enable, active LOW) |
| 5  | DATA (serial data) |
| 17 | STCP (latch) |
| 19 | PWM (motor speed) |

### Sensors / peripherals
| GPIO | Device |
|---:|---|
| 13 | Ultrasonic TRIG |
| 14 | Ultrasonic ECHO (**found by multi-echo scan**) |
| 25 | Servo |
| 33 | Buzzer |
| 12, 2 | LEDs |
| 39, 36, 35 | Trace sensors L/M/R |
| 4 | IR receiver (raw) |
| 27 | Temperature (analog) |

---

## BLE UUIDs

- Service: `0000FFE0-0000-1000-8000-00805F9B34FB`
- Command RX (Write No Response): `0000FFE1-0000-1000-8000-00805F9B34FB`
- Sensor TX (Notify): `0000FFE2-0000-1000-8000-00805F9B34FB`

---

## Troubleshooting

### “GATT Error: Not supported.”
- Firmware and HTML UUIDs do not match, **or**
- Firmware did not create the `FFE2` notify characteristic.
Flash the latest `car_ble.ino` and ensure the HTML uses FFE0/FFE1/FFE2.

### Ultrasonic shows `999` / `NO_ECHO`
- Common cause: **loose connector** on the ultrasonic header (works only when pressing the plug).
- Fix: push connector fully, swap cable, add strain relief (tape/hot glue), or re-solder header.

### IR value doesn’t “change”
- Many IR receivers output **HIGH (1)** when idle.
- It changes only when you point an IR remote and press buttons.
- For decoded button codes, we’d add IR decoding (protocol handling).

### Web Bluetooth doesn’t connect
- Must use **Chrome/Edge**
- Must use **https://** or **localhost**
- Ensure Bluetooth is enabled and site permissions allow Bluetooth

---

## Notes

- Mecanum strafing (`SL`/`SR`) only works with mecanum wheels.
- Temperature reading assumes an **analog** temperature sensor. If you used DHT11/DHT22/DS18B20, the firmware must change.

# esp_presence_zigbee

ESP32-C6 Zigbee end device combining a 24 GHz mmWave radar and time-of-flight sensor to detect presence, motion, and range — reports live to Zigbee2MQTT via three Zigbee endpoints.

## Key Technologies

- **MCU:** ESP32-C6 (native IEEE 802.15.4 radio)
- **Sensors:** HLK-LD2410C mmWave radar (UART), VL53L0X ToF ranging (I2C)
- **Protocol:** Zigbee Home Automation (0x0104), Analog Input cluster (0x000C)
- **Stack:** ESP-IDF + Espressif Zigbee SDK (ZBOSS)
- **Integration:** Zigbee2MQTT with custom converter

## Zigbee Endpoints

| Endpoint | Measurement | Values |
|---|---|---|
| 1 | Presence (mmWave) | 0.0 / 1.0 |
| 2 | Range (ToF) | distance in cm |
| 3 | Static energy (mmWave) | 0–100 |

Reports on value change with 1 s minimum and 300 s maximum intervals.

## Getting Started

**Prerequisites:** ESP-IDF, ESP32-C6 devkit, HLK-LD2410C on UART, VL53L0X on I2C

```bash
idf.py set-target esp32c6
idf.py build flash monitor
```

Pair the device to your Zigbee network and add the included `presence-node.js` converter to Zigbee2MQTT. A pre-built binary is available in [Releases](../../releases).

---

Built by Owen O'Hehir — embedded Linux, IoT, Matter & Rust consulting at [electronicsconsult.com](https://electronicsconsult.com). Available for contract and consulting work.

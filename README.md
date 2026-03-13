# Rufilla Presence Node

ESP32-C6 Zigbee end device combining an HLK-LD2410C 24 GHz mmWave radar and a VL53L0X time-of-flight ranging sensor. Currently reports binary presence over Zigbee using the standard Analog Input cluster (`genAnalogInput` / `0x000C`).

## Status

- LD2410C presence detection: **working end-to-end** (ESP32 -> Zigbee -> Z2M -> MQTT)
- VL53L0X ToF ranging: driver working, Zigbee reporting **placeholder** (to be added as endpoint 2)

## Hardware

### Target
ESP32-C6 (QFN40) with native IEEE 802.15.4 radio. Uses USB-Serial/JTAG for flashing and console output.

### Wiring

| Signal | GPIO | Notes |
|---|---|---|
| LD2410C TX -> ESP RX | GPIO5 | UART1, 256000 baud |
| LD2410C RX <- ESP TX | GPIO4 | UART1 |
| VL53L0X SDA | GPIO6 | I2C0, 400 kHz |
| VL53L0X SCL | GPIO7 | I2C0 |

Power: mains via USB (no battery constraints).

## Zigbee Architecture

| Item | Value |
|---|---|
| Device type | End Device (ZED) |
| Profile | Home Automation (`0x0104`) |
| Endpoint 1 | Analog Input cluster (`0x000C`), server role |
| Attribute | `present_value` (`0x0055`) — `1.0` = presence, `0.0` = absent |
| Reporting | Automatic on attribute change via ZBOSS stack (no explicit bindings or reporting config needed) |
| Report interval | ~5 s (sensor polling rate) |
| Channel | 15 |
| Manufacturer | `Rufilla` |
| Model | `presence-node-v1` |

### How reporting works

The firmware calls `esp_zb_zcl_set_attribute_val()` every 5 seconds. The ZBOSS stack automatically sends unsolicited attribute reports to the coordinator — no explicit `report_attr_cmd_req()` or binding table entries are needed. This matches the pattern used by the [sound level monitor](https://github.com/OOHehir/esp_sound_lvl_zigbee).

## Build and Flash

Requires ESP-IDF v5.5.x with Zigbee support.

```bash
# One-time setup
idf.py set-target esp32c6

# Build
idf.py build

# Flash (USB-Serial/JTAG — no stub required)
python -m esptool --chip esp32c6 --no-stub -p /dev/ttyACM0 -b 115200 \
  --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_size 2MB --flash_freq 80m \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/rufilla-presence-node.bin
```

Or if `idf.py flash` works with your setup:
```bash
idf.py -p /dev/ttyACM0 flash monitor
```

## Zigbee2MQTT Setup

### External Converter

Copy `z2m/rufilla-presence-node.js` to your Z2M `data/` directory (or a subdirectory like `converters/`) and reference it in `configuration.yaml`:

```yaml
external_converters:
  - rufilla-presence-node.js
```

Restart Z2M to load the converter.

### Pairing

1. Enable permit join in Z2M (frontend or MQTT: `zigbee2mqtt/bridge/request/permit_join` with payload `{"value": true, "time": 254}`)
2. Power the device (or erase ZB storage and reset for a fresh join)
3. Device appears as `presence-node-v1` by Rufilla
4. Once joined, MQTT messages publish to `zigbee2mqtt/<device_name>`

### MQTT Output

```json
{"presence": true, "linkquality": 255}
```

| Field | Type | Description |
|---|---|---|
| `presence` | boolean | `true` if moving or stationary target detected |
| `linkquality` | numeric (0-255) | Zigbee link quality indicator |

## Factory Reset

Erase the Zigbee storage partition to force a fresh network join:

```bash
python -m esptool --chip esp32c6 --no-stub -p /dev/ttyACM0 -b 115200 \
  --before default_reset --after hard_reset erase_region 0x190000 0x10000
```

Or erase the entire flash:
```bash
idf.py erase-flash
```

## Partition Layout

| Name | Offset | Size | Purpose |
|---|---|---|---|
| nvs | 0x9000 | 24 KB | NVS key-value store |
| phy_init | 0xF000 | 4 KB | PHY calibration |
| factory | 0x10000 | 1536 KB | Application firmware |
| zb_storage | 0x190000 | 64 KB | Zigbee network state |
| zb_fct | 0x1A0000 | 4 KB | Zigbee factory data |

## Project Structure

```
├── CMakeLists.txt
├── sdkconfig.defaults          # ZB_ENABLED, native radio, USB console
├── partitions.csv
├── main/
│   ├── main.c                  # Init sensors, start Zigbee main loop
│   └── idf_component.yml       # esp-zigbee-lib + esp-zboss-lib
├── components/
│   ├── ld2410c/                # HLK-LD2410C mmWave radar UART driver
│   │   ├── include/ld2410c.h
│   │   ├── ld2410c.c
│   │   └── test/test_ld2410c.c
│   ├── vl53l0x/                # VL53L0X ToF I2C driver
│   │   ├── include/vl53l0x.h
│   │   ├── vl53l0x.c
│   │   └── test/test_vl53l0x.c
│   └── zigbee_node/            # Zigbee end device + Analog Input cluster
│       ├── include/zigbee_node.h
│       └── zigbee_node.c
├── z2m/
│   └── rufilla-presence-node.js  # Z2M external converter
└── test/
    └── main/test_main.c
```

## Unit Tests

```bash
cd test
idf.py set-target esp32c6
idf.py build flash monitor
```

Tests cover LD2410C frame parsing and VL53L0X status mapping.

## Next Steps

- Add VL53L0X range reporting as endpoint 2 (second Analog Input cluster)
- Evaluate encoding strategy to keep endpoint count minimal

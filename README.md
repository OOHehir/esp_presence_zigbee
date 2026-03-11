# Rufilla Presence Node

ESP32-C6 Zigbee sensor node combining an HLK-LD2410C mmWave radar and a VL53L0X time-of-flight ranging sensor. Reports presence and ranging data over Zigbee to a Zigbee2MQTT coordinator.

## Hardware Wiring

| Signal | GPIO |
|---|---|
| LD2410C UART TX → ESP RX | GPIO17 |
| LD2410C UART RX ← ESP TX | GPIO16 |
| VL53L0X SDA | GPIO6 |
| VL53L0X SCL | GPIO7 |

Power: mains (no battery constraints).

## Build and Flash

```bash
source ~/.espressif/tools/activate_idf_v5.5.3.sh
idf.py set-target esp32c6
idf.py build
idf.py flash monitor
```

## First-Time Zigbee Pairing

1. Put Zigbee2MQTT into permit join mode (via the Z2M frontend or `zigbee2mqtt/bridge/request/permit_join` MQTT topic).
2. Power the device. It will automatically attempt to join the network.
3. The device should appear in the Z2M device list as `presence-node-v1`.
4. Once joined, the device reports sensor data every 5 seconds.

## Zigbee2MQTT External Converter

Copy `z2m/rufilla-presence-node.js` to your Zigbee2MQTT data directory and add to `configuration.yaml`:

```yaml
external_converters:
  - rufilla-presence-node.js
```

Restart Zigbee2MQTT to load the converter.

### Exposed MQTT Data

| Feature | Type | Source |
|---|---|---|
| `occupancy` | binary | LD2410C presence |
| `moving_target` | binary | LD2410C |
| `stationary_target` | binary | LD2410C |
| `move_energy` | numeric (0–100) | LD2410C |
| `static_energy` | numeric (0–100) | LD2410C |
| `target_distance_cm` | numeric (cm) | LD2410C |
| `range_mm` | numeric (mm) | VL53L0X |
| `range_status` | numeric | VL53L0X (0=valid, 1=sigma fail, 2=signal fail, 255=no target) |

## Running Unit Tests

```bash
cd test
idf.py set-target esp32c6
idf.py build flash monitor
```

Tests cover LD2410C frame parsing and VL53L0X status mapping. Hardware-dependent tests are noted in comments and should be skipped in CI.

## Factory Reset

To erase the Zigbee network state and force a fresh join:

```bash
idf.py erase-flash
```

Or erase only the Zigbee storage partition:

```bash
parttool.py erase_partition --partition-name zb_storage
```

Then power-cycle the device.

A serial UART command `reset\n` sent to the ESP32-C6 console UART will also erase the Zigbee storage and restart the device.

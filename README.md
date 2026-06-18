# Landis+Gyr E450 Cyprus Meter

ESPHome external component for decoding Landis+Gyr E450 Cyprus meter push frames received over an M-Bus-to-UART interface.

The component parses HDLC, GBT, DLMS notification, and A-XDR payloads, then publishes decoded values as ESPHome sensors and text sensors.

## Hardware

- ESP32 board, tested with `Waveshare ESP32-C6 1.47inch Display Development Board`.
- M-Bus-to-UART adapter, for example a `Mikroe M-Bus Slave Click`.
- Meter adapter TX/data output connected to ESP32 GPIO0.

## ESPHome Usage

Use this repository as an external component:

```yaml
external_components:
  - source: github://vanyasvl/e450_cyprus_meter@master
    components: [e450_cyprus_meter]
```

Minimal component setup:

```yaml
uart:
  id: mbus_uart
  rx_pin: GPIO0
  baud_rate: 2400
  data_bits: 8
  parity: NONE
  stop_bits: 1
  rx_buffer_size: 2048

e450_cyprus_meter:
  id: meter
  uart_id: mbus_uart
  frame_timeout: 1500ms
  validate_crc: true

  push_timestamp:
    name: Push Timestamp
  active_power_import_total:
    name: Active Power Import Total
  current_l1:
    name: Current L1
  power_factor:
    name: Power Factor
  frequency:
    name: Frequency
```

See [esphome-e450-cyprus-meter.yaml](esphome-e450-cyprus-meter.yaml) for a full example with all exposed sensors.

## Local Tests

Run the native C++ decoder fixture test from the repository root:

```bash
tests/run_cpp_tests.sh
```

Expected output:

```text
C++ mbus_frames test passed: 117 HDLC frames, 23 push groups, 467 published events
```

The test compiles a small C++ harness with ESPHome stubs, feeds [tests/fixtures/mbus_frames.log](tests/fixtures/mbus_frames.log) through the real component, and compares the redacted decoded event dump with [tests/fixtures/mbus_frames.redacted.dump](tests/fixtures/mbus_frames.redacted.dump).

## ESPHome Compile

With ESPHome installed:

```bash
esphome run esphome-e450-cyprus-meter.yaml
```

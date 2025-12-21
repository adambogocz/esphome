TDMA485 (RS‑485 SYNC + TDMA minislot protocol)
==============================================

Overview
--------
This component implements a simple event‑driven protocol for half‑duplex RS‑485 at robust low baud rates (e.g., 9600 8N1). A monitor node periodically sends a short SYNC and scans N fixed minislots for 1‑byte claims from nodes. Nodes that have events claim their slot, then the monitor grants transmission sequentially; each granted node sends its event frame and receives an ACK.

Key properties:
- Deterministic, low latency at 9600 bps with ~10 nodes.
- No continuous polling. Empty slots pass quickly (~2 char times).
- CRC16‑CCITT on all frames; short ACK for reliability.

Protocol (default)
------------------
- SYNC: 2 bytes [0xF0, 0x0F]
- Claim (node i): 1 byte 0xC0 | (i & 0x1F)
- GRANT: [0xA5, 'G'(0x47), node_id, seq, CRC16LE]
- EVENT: [0xA5, 'E'(0x45), node_id, seq, event_type, len, payload[len], CRC16LE]
- ACK:   [0xA5, 0x06, node_id, seq, status(0=OK), CRC16LE]

Timing
------
- 8N1 character time at 9600 bps: ~1.04 ms
- Guard time: configurable (default 1.0 chars)
- Claim window: ~1.2 chars
- Slot ≈ guard + claim ≈ ~2.2 chars → ~2.3 ms
- 10 nodes: ~23 ms base cycle + payload handling for claimers

ESPHome configuration
---------------------

1) Monitor (coordinator)

```yaml
esphome:
  name: tdma_monitor
  platform: ESP32
  board: esp32dev

logger:
  level: INFO

uart:
  id: bus_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600

tdma485:
  id: tdma_bus
  uart_id: bus_uart
  role: monitor
  num_nodes: 10
  guard_chars: 1.0
  direction_pin: GPIO4   # DE/RE pin (HIGH=TX, LOW=RX). Optional if transceiver auto‑direction
  # Optional tuning:
  # baud: 9600
  # sync_byte_a: 0xF0
  # sync_byte_b: 0x0F
  # grant_timeout_ms: 100
  # ack_timeout_ms: 50
  on_event:
    - lambda: |-
        // Variables available: node_id (uint8_t), event_type (uint8_t), payload (std::vector<uint8_t>)
        ESP_LOGI("tdma", "Evt from %u type %u len %u", node_id, event_type, (unsigned)payload.size());
        // Example: if type==1 and payload is 2 bytes little-endian number:
        if (event_type == 1 && payload.size() >= 2) {
          uint16_t val = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
          ESP_LOGI("tdma", "Value=%u", val);
        }
```

2) Node (event producer)

```yaml
esphome:
  name: tdma_node3
  platform: ESP32
  board: esp32dev

logger:
  level: INFO

uart:
  id: bus_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600

tdma485:
  id: tdma_bus
  uart_id: bus_uart
  role: node
  node_id: 3
  guard_chars: 1.0
  direction_pin: GPIO4
  # baud, sync bytes, timeouts as needed

# Example: send an event with type=1 and a 16‑bit value when a button is pressed
button:
  - platform: gpio
    name: "Send TDMA Event"
    pin:
      number: GPIO0
      mode: INPUT_PULLUP
      inverted: true
    on_press:
      - lambda: |-
          id(tdma_bus).queue_simple_event(1, 1234);
```

Notes
-----
- The component toggles the RS‑485 direction pin (if provided): HIGH to transmit, LOW to receive, with automatic guard delays.
- For best robustness: terminate both bus ends (120 Ω), add biasing resistors, keep stubs short, use twisted pair/shield where appropriate.
- If many slots are empty, this design keeps cycles short; payload time is only added for claimers.
- The monitor exposes an `on_event:` automation with `node_id`, `event_type`, and `payload` arguments.

Extending
---------
- Map specific `event_type` values to sensors (numeric) or text sensors (hex payload).
```

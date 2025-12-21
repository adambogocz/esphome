// Copyright 2025
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include <vector>

namespace esphome {
namespace tdma485 {

static const char *const TAG = "tdma485";

// Protocol constants
static const uint8_t SOF = 0xA5;         // Start of frame for GRANT, EVENT, ACK frames
static const uint8_t TYPE_GRANT = 0x47;  // 'G'
static const uint8_t TYPE_EVENT = 0x45;  // 'E'
static const uint8_t TYPE_ACK = 0x06;    // ACK

// Claim byte template: 0xC0 | node_id (low 5 bits)
inline uint8_t make_claim_byte(uint8_t node_id) { return uint8_t(0xC0 | (node_id & 0x1F)); }

class TDMA485 : public Component, public uart::UARTDevice {
 public:
  void set_is_monitor(bool is_monitor) { this->is_monitor_ = is_monitor; }
  void set_node_id(uint8_t node_id) { this->node_id_ = node_id; }
  void set_num_nodes(uint8_t num_nodes) { this->num_nodes_ = num_nodes; }
  void set_baud(uint32_t baud) { this->baud_ = baud; }
  void set_guard_chars(float guard_chars) { this->guard_chars_ = guard_chars; }
  void set_sync_bytes(uint8_t a, uint8_t b) {
    this->sync_a_ = a;
    this->sync_b_ = b;
  }
  void set_timeouts_ms(uint16_t grant_ms, uint16_t ack_ms) {
    this->grant_timeout_ms_ = grant_ms;
    this->ack_timeout_ms_ = ack_ms;
  }
  void set_direction_pin(GPIOPin *pin) { this->dir_pin_ = pin; }

  // Node-side API: simple event helpers callable from lambda
  void queue_simple_event(uint8_t type, uint16_t value);
  void queue_bytes_event(uint8_t type, const std::vector<uint8_t> &data);

  void setup() override;
  void dump_config() override;
  void loop() override;

  // Monitor-side trigger for received events
  esphome::Trigger<uint8_t, uint8_t, std::vector<uint8_t>> *get_on_event_trigger() { return &this->on_event_; }

 protected:
  // Common
  bool is_monitor_{false};
  uint8_t node_id_{0};
  uint8_t num_nodes_{10};
  uint32_t baud_{9600};
  float guard_chars_{1.0f};
  uint8_t sync_a_{0xF0};
  uint8_t sync_b_{0x0F};
  uint16_t grant_timeout_ms_{100};
  uint16_t ack_timeout_ms_{50};

  GPIOPin *dir_pin_{nullptr};

  // Timing derived
  uint32_t char_time_us_{0};
  uint32_t guard_time_us_{0};
  uint32_t claim_window_us_{0};

  // Direction control
  void set_tx_mode_(bool tx);
  void write_bytes_blocking_(const uint8_t *data, size_t len);
  void write_byte_blocking_(uint8_t b);
  bool read_byte_with_timeout_(uint32_t timeout_us, uint8_t &out);
  bool read_exact_with_timeout_(uint8_t *buf, size_t len, uint32_t timeout_us);
  void clear_rx_();

  // CRC
  uint16_t crc16_ccitt_(const uint8_t *data, size_t len, uint16_t seed = 0xFFFF);
  void append_crc16_(std::vector<uint8_t> &frame);
  bool verify_crc16_(const std::vector<uint8_t> &frame);

  // Frames
  void send_sync_();
  void send_grant_(uint8_t node_id, uint8_t seq);
  bool read_event_frame_(uint8_t &node_id, uint8_t &event_type, std::vector<uint8_t> &payload, uint8_t &seq);
  void send_ack_(uint8_t node_id, uint8_t seq, uint8_t status);

  // Monitor state
  void monitor_cycle_();

  // Node state
  struct PendingEvent {
    uint8_t type{0};
    std::vector<uint8_t> data;
    uint8_t seq{0};
  };
  std::vector<PendingEvent> queue_;
  uint8_t seq_counter_{0};

  void node_cycle_();
  bool wait_for_sync_(uint32_t timeout_ms);
  void send_claim_();
  bool wait_for_grant_(uint8_t &seq);
  void send_event_(uint8_t seq, const PendingEvent &ev);
  bool wait_for_ack_(uint8_t expected_seq, uint32_t timeout_ms);

  // Automations
  esphome::Trigger<uint8_t, uint8_t, std::vector<uint8_t>> on_event_;
};

}  // namespace tdma485
}  // namespace esphome

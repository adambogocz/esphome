#include "tdma485.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tdma485 {

void TDMA485::setup() {
  // Pre-compute timing
  const float bits_per_char = 10.0f;  // 8N1
  this->char_time_us_ = static_cast<uint32_t>((bits_per_char * 1e6f) / float(this->baud_));
  this->guard_time_us_ = static_cast<uint32_t>(this->guard_chars_ * float(this->char_time_us_));
  this->claim_window_us_ = static_cast<uint32_t>(1.2f * float(this->char_time_us_));

  if (this->dir_pin_ != nullptr) {
    this->dir_pin_->setup();
    this->dir_pin_->pin_mode(gpio::FLAG_OUTPUT);
    this->set_tx_mode_(false);
  }
}

void TDMA485::dump_config() {
  ESP_LOGCONFIG(TAG, "TDMA485:");
  ESP_LOGCONFIG(TAG, "  Role: %s", this->is_monitor_ ? "MONITOR" : "NODE");
  if (!this->is_monitor_) {
    ESP_LOGCONFIG(TAG, "  Node ID: %u", this->node_id_);
  }
  ESP_LOGCONFIG(TAG, "  Num nodes: %u", this->num_nodes_);
  ESP_LOGCONFIG(TAG, "  Baud: %u", this->baud_);
  ESP_LOGCONFIG(TAG, "  Char time (us): %u", this->char_time_us_);
  ESP_LOGCONFIG(TAG, "  Guard chars: %.2f (=> %u us)", this->guard_chars_, this->guard_time_us_);
  ESP_LOGCONFIG(TAG, "  Claim window (us): %u", this->claim_window_us_);
  ESP_LOGCONFIG(TAG, "  Sync bytes: 0x%02X 0x%02X", this->sync_a_, this->sync_b_);
  ESP_LOGCONFIG(TAG, "  Timeouts: grant %u ms, ack %u ms", this->grant_timeout_ms_, this->ack_timeout_ms_);
  LOG_PIN("  Direction pin: ", this->dir_pin_);
}

void TDMA485::loop() {
  if (this->is_monitor_) {
    this->monitor_cycle_();
  } else {
    this->node_cycle_();
  }
}

// ---- Public Node API ----

void TDMA485::queue_simple_event(uint8_t type, uint16_t value) {
  PendingEvent ev;
  ev.type = type;
  ev.data.reserve(2);
  ev.data.push_back(uint8_t(value & 0xFF));
  ev.data.push_back(uint8_t((value >> 8) & 0xFF));
  this->queue_.push_back(std::move(ev));
}

void TDMA485::queue_bytes_event(uint8_t type, const std::vector<uint8_t> &data) {
  PendingEvent ev;
  ev.type = type;
  ev.data = data;
  this->queue_.push_back(std::move(ev));
}

// ---- Low-level helpers ----

void TDMA485::set_tx_mode_(bool tx) {
  if (this->dir_pin_ != nullptr) {
    this->dir_pin_->digital_write(tx);
    // allow bus driver turn-around ~1 char-time
    delayMicroseconds(this->char_time_us_);
  }
}

void TDMA485::write_bytes_blocking_(const uint8_t *data, size_t len) {
  this->set_tx_mode_(true);
  for (size_t i = 0; i < len; i++) {
    this->write_byte(data[i]);
  }
  this->flush();
  // guard after TX before releasing bus
  delayMicroseconds(this->guard_time_us_);
  this->set_tx_mode_(false);
}

void TDMA485::write_byte_blocking_(uint8_t b) {
  this->set_tx_mode_(true);
  this->write_byte(b);
  this->flush();
  delayMicroseconds(this->guard_time_us_);
  this->set_tx_mode_(false);
}

bool TDMA485::read_byte_with_timeout_(uint32_t timeout_us, uint8_t &out) {
  const uint32_t start = micros();
  while (true) {
    if (this->available()) {
      int c = this->read();
      if (c >= 0) {
        out = static_cast<uint8_t>(c & 0xFF);
        return true;
      }
    }
    if (micros() - start >= timeout_us) {
      return false;
    }
    // Short yield
    delayMicroseconds(50);
  }
}

bool TDMA485::read_exact_with_timeout_(uint8_t *buf, size_t len, uint32_t timeout_us) {
  const uint32_t start = micros();
  size_t got = 0;
  while (got < len) {
    uint8_t b;
    if (this->read_byte_with_timeout_(50, b)) {
      buf[got++] = b;
      continue;
    }
    if (micros() - start >= timeout_us) {
      return false;
    }
  }
  return true;
}

void TDMA485::clear_rx_() {
  while (this->available()) {
    (void) this->read();
  }
}

// ---- CRC ----

uint16_t TDMA485::crc16_ccitt_(const uint8_t *data, size_t len, uint16_t seed) {
  uint16_t crc = seed;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t) data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void TDMA485::append_crc16_(std::vector<uint8_t> &frame) {
  uint16_t crc = this->crc16_ccitt_(frame.data(), frame.size());
  frame.push_back(uint8_t(crc & 0xFF));
  frame.push_back(uint8_t((crc >> 8) & 0xFF));
}

bool TDMA485::verify_crc16_(const std::vector<uint8_t> &frame) {
  if (frame.size() < 3)
    return false;
  uint16_t calc = this->crc16_ccitt_(frame.data(), frame.size() - 2);
  uint16_t rx = uint16_t(frame[frame.size() - 2]) | (uint16_t(frame[frame.size() - 1]) << 8);
  return calc == rx;
}

// ---- Frames ----

void TDMA485::send_sync_() {
  uint8_t sync[2] = {this->sync_a_, this->sync_b_};
  this->write_bytes_blocking_(sync, sizeof(sync));
}

void TDMA485::send_grant_(uint8_t node_id, uint8_t seq) {
  std::vector<uint8_t> f;
  f.reserve(6);
  f.push_back(SOF);
  f.push_back(TYPE_GRANT);
  f.push_back(node_id);
  f.push_back(seq);
  this->append_crc16_(f);
  this->write_bytes_blocking_(f.data(), f.size());
}

bool TDMA485::read_event_frame_(uint8_t &node_id, uint8_t &event_type, std::vector<uint8_t> &payload, uint8_t &seq) {
  // Wait for SOF
  uint8_t b;
  if (!this->read_byte_with_timeout_(uint32_t(this->grant_timeout_ms_) * 1000U, b))
    return false;
  if (b != SOF)
    return false;

  uint8_t header[5];  // TYPE, node_id, seq, event_type, len
  if (!this->read_exact_with_timeout_(header, sizeof(header), uint32_t(this->grant_timeout_ms_) * 1000U))
    return false;
  if (header[0] != TYPE_EVENT)
    return false;
  node_id = header[1];
  seq = header[2];
  event_type = header[3];
  uint8_t len = header[4];

  payload.resize(len);
  if (len > 0) {
    if (!this->read_exact_with_timeout_(payload.data(), len, uint32_t(this->grant_timeout_ms_) * 1000U))
      return false;
  }

  uint8_t crc_buf[2];
  if (!this->read_exact_with_timeout_(crc_buf, 2, uint32_t(this->grant_timeout_ms_) * 1000U))
    return false;

  std::vector<uint8_t> frame;
  frame.reserve(1 + sizeof(header) + len + 2);
  frame.push_back(SOF);
  frame.insert(frame.end(), header, header + sizeof(header));
  if (len > 0)
    frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(crc_buf[0]);
  frame.push_back(crc_buf[1]);

  if (!this->verify_crc16_(frame))
    return false;
  return true;
}

void TDMA485::send_ack_(uint8_t node_id, uint8_t seq, uint8_t status) {
  std::vector<uint8_t> f;
  f.reserve(7);
  f.push_back(SOF);
  f.push_back(TYPE_ACK);
  f.push_back(node_id);
  f.push_back(seq);
  f.push_back(status);
  this->append_crc16_(f);
  this->write_bytes_blocking_(f.data(), f.size());
}

// ---- Monitor ----

void TDMA485::monitor_cycle_() {
  static uint8_t grant_seq = 0;

  // Broadcast SYNC
  this->send_sync_();

  // Scan claims
  uint32_t claim_mask = 0;
  for (uint8_t i = 0; i < this->num_nodes_; i++) {
    delayMicroseconds(this->guard_time_us_);
    uint8_t b;
    if (this->read_byte_with_timeout_(this->claim_window_us_, b)) {
      if (b == make_claim_byte(i)) {
        claim_mask |= (1UL << i);
      } else {
        // Unexpected byte, clear RX to avoid cascading errors
        ESP_LOGW(TAG, "Unexpected byte 0x%02X during slot %u", b, i);
        this->clear_rx_();
      }
    }
  }

  // Handle grants and payloads
  for (uint8_t i = 0; i < this->num_nodes_; i++) {
    if ((claim_mask & (1UL << i)) == 0)
      continue;

    uint8_t seq = ++grant_seq;
    this->send_grant_(i, seq);

    uint8_t rx_node = 0, rx_event_type = 0, rx_seq = 0;
    std::vector<uint8_t> payload;
    bool ok = this->read_event_frame_(rx_node, rx_event_type, payload, rx_seq);
    if (ok && rx_node == i && rx_seq == seq) {
      ESP_LOGI(TAG, "Event from node %u type %u seq %u len %u", rx_node, rx_event_type, rx_seq,
               (unsigned) payload.size());
      // Fire automation trigger
      this->on_event_.trigger(rx_node, rx_event_type, payload);
      this->send_ack_(i, seq, 0x00);
    } else {
      ESP_LOGW(TAG, "Failed to get event from node %u (ok=%d, rx_node=%u, rx_seq=%u, exp_seq=%u)", i, (int) ok, rx_node,
               rx_seq, seq);
      this->send_ack_(i, seq, 0x01);
    }
    delayMicroseconds(this->guard_time_us_);
  }
}

// ---- Node ----

bool TDMA485::wait_for_sync_(uint32_t timeout_ms) {
  const uint32_t start = millis();
  bool got_first = false;
  while (millis() - start < timeout_ms) {
    uint8_t b;
    if (!this->read_byte_with_timeout_(500, b)) {
      continue;
    }
    if (!got_first) {
      if (b == this->sync_a_) {
        got_first = true;
      }
      continue;
    } else {
      if (b == this->sync_b_) {
        return true;
      } else {
        got_first = (b == this->sync_a_);
      }
    }
  }
  return false;
}

void TDMA485::send_claim_() {
  const uint8_t claim = make_claim_byte(this->node_id_);
  this->write_byte_blocking_(claim);
}

bool TDMA485::wait_for_grant_(uint8_t &seq) {
  uint8_t b;
  if (!this->read_byte_with_timeout_(uint32_t(this->grant_timeout_ms_) * 1000U, b))
    return false;
  if (b != SOF)
    return false;

  uint8_t header[3];  // TYPE, node_id, seq
  if (!this->read_exact_with_timeout_(header, sizeof(header), uint32_t(this->grant_timeout_ms_) * 1000U))
    return false;
  uint8_t type = header[0];
  uint8_t node = header[1];
  seq = header[2];

  uint8_t crc_buf[2];
  if (!this->read_exact_with_timeout_(crc_buf, 2, uint32_t(this->grant_timeout_ms_) * 1000U))
    return false;

  std::vector<uint8_t> frame;
  frame.reserve(1 + sizeof(header) + 2);
  frame.push_back(SOF);
  frame.insert(frame.end(), header, header + sizeof(header));
  frame.push_back(crc_buf[0]);
  frame.push_back(crc_buf[1]);

  if (!this->verify_crc16_(frame))
    return false;
  if (type != TYPE_GRANT)
    return false;
  if (node != this->node_id_)
    return false;
  return true;
}

void TDMA485::send_event_(uint8_t seq, const PendingEvent &ev) {
  std::vector<uint8_t> f;
  f.reserve(7 + ev.data.size());
  f.push_back(SOF);
  f.push_back(TYPE_EVENT);
  f.push_back(this->node_id_);
  f.push_back(seq);
  f.push_back(ev.type);                  // event type
  f.push_back(uint8_t(ev.data.size()));  // len
  if (!ev.data.empty()) {
    f.insert(f.end(), ev.data.begin(), ev.data.end());
  }
  this->append_crc16_(f);
  this->write_bytes_blocking_(f.data(), f.size());
}

bool TDMA485::wait_for_ack_(uint8_t expected_seq, uint32_t timeout_ms) {
  uint8_t b;
  if (!this->read_byte_with_timeout_(timeout_ms * 1000U, b))
    return false;
  if (b != SOF)
    return false;
  uint8_t header[4];  // TYPE, node_id, seq, status
  if (!this->read_exact_with_timeout_(header, sizeof(header), timeout_ms * 1000U))
    return false;
  uint8_t crc_buf[2];
  if (!this->read_exact_with_timeout_(crc_buf, 2, timeout_ms * 1000U))
    return false;

  std::vector<uint8_t> frame;
  frame.reserve(1 + sizeof(header) + 2);
  frame.push_back(SOF);
  frame.insert(frame.end(), header, header + sizeof(header));
  frame.push_back(crc_buf[0]);
  frame.push_back(crc_buf[1]);
  if (!this->verify_crc16_(frame))
    return false;
  if (header[0] != TYPE_ACK)
    return false;
  if (header[1] != this->node_id_)
    return false;
  if (header[2] != expected_seq)
    return false;
  return header[3] == 0x00;
}

void TDMA485::node_cycle_() {
  // Only act if we need to send an event soon; otherwise just drain any noise and return
  const bool have_event = !this->queue_.empty();
  if (!this->wait_for_sync_(/*timeout_ms=*/200)) {
    return;
  }

  // TDMA slot timing
  const uint32_t slot_us = this->guard_time_us_ + this->claim_window_us_;
  // Wait until our slot
  const uint32_t before = micros();
  const uint32_t target = before + uint32_t(slot_us) * uint32_t(this->node_id_);
  while ((int32_t) (micros() - target) < 0) {
    delayMicroseconds(50);
  }

  // In our slot
  delayMicroseconds(this->guard_time_us_);
  if (have_event) {
    this->send_claim_();
  }

  // If no event to send, we're done for this cycle
  if (!have_event) {
    return;
  }

  // Wait for grant
  uint8_t seq = 0;
  if (!this->wait_for_grant_(seq)) {
    ESP_LOGW(TAG, "No grant received; will retry next cycle");
    return;
  }

  // Send one event
  PendingEvent ev = std::move(this->queue_.front());
  this->queue_.erase(this->queue_.begin());
  this->send_event_(seq, ev);

  // Wait for ACK
  if (!this->wait_for_ack_(seq, this->ack_timeout_ms_)) {
    ESP_LOGW(TAG, "ACK failed; re-queueing event");
    this->queue_.insert(this->queue_.begin(), std::move(ev));
    return;
  }

  ESP_LOGI(TAG, "Event sent OK (type=%u, len=%u)", ev.type, (unsigned) ev.data.size());
}

}  // namespace tdma485
}  // namespace esphome

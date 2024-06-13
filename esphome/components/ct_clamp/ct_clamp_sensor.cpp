#include "ct_clamp_sensor.h"

#include "esphome/core/log.h"
#include <cinttypes>
#include <cmath>

namespace esphome {
namespace ct_clamp {

static const char *const TAG = "ct_clamp";

void CTClampSensor::dump_config() {
  LOG_SENSOR("", "CT Clamp Sensor", this);
  ESP_LOGCONFIG(TAG, "  Sample Duration: %.2fs", this->sample_duration_ / 1e3f);
  LOG_UPDATE_INTERVAL(this);
}

void CTClampSensor::update() {
  // Update only starts the sampling phase, in loop() the actual sampling is happening.

  // Request a high loop() execution interval during sampling phase.
  this->high_freq_.start();

  // Set timeout for ending sampling phase
  this->set_timeout("read", this->sample_duration_, [this]() {
    this->is_sampling_ = false;
    this->high_freq_.stop();

    if (this->num_samples_ == 0) {
      // Shouldn't happen, but let's not crash if it does.
      this->publish_state(NAN);
      return;
    }

    const float rms_ac_dc_squared = this->sample_squared_sum_ / this->num_samples_;
    const float rms_dc = this->sample_sum_ / this->num_samples_;
    const float rms_ac_squared = rms_ac_dc_squared - rms_dc * rms_dc;
    float rms_ac = 0;
    float rms_ac_dc = 0;
    if (rms_ac_squared > 0)
      rms_ac = std::sqrt(rms_ac_squared);
    if (rms_ac_dc_squared > 0)
      rms_ac_dc = std::sqrt(rms_ac_dc_squared);
    ESP_LOGD(TAG, "'%s' - Raw AC Value: %.3fA (%.3fA) after %" PRIu32 " different samples (%" PRIu32 " SPS)",
             this->name_.c_str(), rms_ac, rms_ac_dc, this->num_samples_, 1000 * this->num_samples_ / this->sample_duration_);
    this->publish_state(rms_ac);
  });

  // Set sampling values
  this->last_sample_at = 0;
  this->num_samples_ = 0;
  this->sample_sum_ = 0.0f;
  this->sample_squared_sum_ = 0.0f;
  this->is_sampling_ = true;
}

void CTClampSensor::loop() {
  if (!this->is_sampling_)
    return;

  // Get current time
  unsigned long current_time = micros();

  if((current_time - this->last_sample_at) < 550)
    return; // ADC not ready yet

  // Perform a single sample
  float value = this->source_->sample();
  if (std::isnan(value))
    return;

  this->last_sample_at = current_time;
  this->num_samples_++;
  this->sample_sum_ += value;
  this->sample_squared_sum_ += value * value;
}

}  // namespace ct_clamp
}  // namespace esphome

#include "bang_bang_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bang_bang {

static const char *const TAG = "bang_bang.climate";

void BangBangClimate::setup() {
  this->sensor_->add_on_state_callback([this](float state) {
    this->current_temperature = state;
    // control may have changed, recompute
    this->compute_state_();
    // current temperature changed, publish state
    this->publish_state();
  });
  this->current_temperature = this->sensor_->state;

  // register for humidity values and get initial state
  if (this->humidity_sensor_ != nullptr) {
    this->humidity_sensor_->add_on_state_callback([this](float state) {
      this->current_humidity = state;
      this->publish_state();
    });
    this->current_humidity = this->humidity_sensor_->state;
  }

  // restore set points
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->to_call(this).perform();
  } else {
    // restore from defaults, change_away handles those for us
    if (supports_cool_ && supports_heat_) {
      this->mode = climate::CLIMATE_MODE_HEAT_COOL;
    } else if (supports_cool_) {
      this->mode = climate::CLIMATE_MODE_COOL;
    } else if (supports_heat_) {
      this->mode = climate::CLIMATE_MODE_HEAT;
    }
    this->change_preset_internal_(this->normal_config_);
  }
}
void BangBangClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_target_temperature_low().has_value())
    this->target_temperature_low = *call.get_target_temperature_low();
  if (call.get_target_temperature_high().has_value())
    this->target_temperature_high = *call.get_target_temperature_high();
  if (call.get_preset().has_value()) {
    this->change_preset_(*call.get_preset());
  }
  if (call.get_custom_preset().has_value()) {
    // this->change_custom_preset_(*call.get_custom_preset());
  }

  this->compute_state_();
  this->publish_state();
}
climate::ClimateTraits BangBangClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  if (this->humidity_sensor_ != nullptr)
    traits.set_supports_current_humidity(true);
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
  });
  if (supports_cool_)
    traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  if (supports_heat_)
    traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  if (supports_cool_ && supports_heat_)
    traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
  traits.set_supports_two_point_target_temperature(true);

  for (auto &it : this->preset_config_) {
    traits.add_supported_preset(it.first);
  }
  for (auto &it : this->custom_preset_config_) {
    traits.add_supported_custom_preset(it.first);
  }

  traits.set_supports_action(true);
  return traits;
}
void BangBangClimate::compute_state_() {
  if (this->mode == climate::CLIMATE_MODE_OFF) {
    this->switch_to_action_(climate::CLIMATE_ACTION_OFF);
    return;
  }
  if (std::isnan(this->current_temperature) || std::isnan(this->target_temperature_low) ||
      std::isnan(this->target_temperature_high)) {
    // if any control parameters are nan, go to OFF action (not IDLE!)
    this->switch_to_action_(climate::CLIMATE_ACTION_OFF);
    return;
  }
  const bool too_cold = this->current_temperature < this->target_temperature_low;
  const bool too_hot = this->current_temperature > this->target_temperature_high;

  climate::ClimateAction target_action;
  if (too_cold) {
    // too cold -> enable heating if possible and enabled, else idle
    if (this->supports_heat_ &&
        (this->mode == climate::CLIMATE_MODE_HEAT_COOL || this->mode == climate::CLIMATE_MODE_HEAT)) {
      target_action = climate::CLIMATE_ACTION_HEATING;
    } else {
      target_action = climate::CLIMATE_ACTION_IDLE;
    }
  } else if (too_hot) {
    // too hot -> enable cooling if possible and enabled, else idle
    if (this->supports_cool_ &&
        (this->mode == climate::CLIMATE_MODE_HEAT_COOL || this->mode == climate::CLIMATE_MODE_COOL)) {
      target_action = climate::CLIMATE_ACTION_COOLING;
    } else {
      target_action = climate::CLIMATE_ACTION_IDLE;
    }
  } else {
    // neither too hot nor too cold -> in range
    if (this->supports_cool_ && this->supports_heat_ && this->mode == climate::CLIMATE_MODE_HEAT_COOL) {
      // if supports both ends and both cooling and heating enabled, go to idle action
      target_action = climate::CLIMATE_ACTION_IDLE;
    } else {
      // else use current mode and don't change (hysteresis)
      target_action = this->action;
    }
  }

  this->switch_to_action_(target_action);
}
void BangBangClimate::switch_to_action_(climate::ClimateAction action) {
  if (action == this->action) {
    // already in target mode
    return;
  }

  if ((action == climate::CLIMATE_ACTION_OFF && this->action == climate::CLIMATE_ACTION_IDLE) ||
      (action == climate::CLIMATE_ACTION_IDLE && this->action == climate::CLIMATE_ACTION_OFF)) {
    // switching from OFF to IDLE or vice-versa
    // these only have visual difference. OFF means user manually disabled,
    // IDLE means it's in auto mode but value is in target range.
    this->action = action;
    this->publish_state();
    return;
  }

  if (this->prev_trigger_ != nullptr) {
    this->prev_trigger_->stop_action();
    this->prev_trigger_ = nullptr;
  }
  Trigger<> *trig;
  switch (action) {
    case climate::CLIMATE_ACTION_OFF:
    case climate::CLIMATE_ACTION_IDLE:
      trig = this->idle_trigger_;
      break;
    case climate::CLIMATE_ACTION_COOLING:
      trig = this->cool_trigger_;
      break;
    case climate::CLIMATE_ACTION_HEATING:
      trig = this->heat_trigger_;
      break;
    default:
      trig = nullptr;
  }
  if (trig != nullptr) {
    trig->trigger();
  } else {
    ESP_LOGW(TAG, "trig not set - unsupported action");
  }
  this->action = action;
  this->prev_trigger_ = trig;
  this->publish_state();
}

void BangBangClimate::set_normal_config(const BangBangClimateTargetTempConfig &normal_config) {
  this->normal_config_ = normal_config;
}
void BangBangClimate::set_away_config(const BangBangClimateTargetTempConfig &away_config) {
  this->preset_config_[climate::CLIMATE_PRESET_AWAY] = away_config;
}
void BangBangClimate::change_preset_(climate::ClimatePreset preset) {
  auto config = this->preset_config_.find(preset);

  if (config != this->preset_config_.end()) {
    ESP_LOGI(TAG, "Preset %s requested", LOG_STR_ARG(climate::climate_preset_to_string(preset)));
    if (this->change_preset_internal_(config->second) || (!this->preset.has_value()) ||
        this->preset.value() != preset) {
      // Fire any preset changed trigger if defined
      Trigger<> *trig = this->preset_change_trigger_;
      this->preset = preset;
      if (trig != nullptr) {
        trig->trigger();
      }

      ESP_LOGI(TAG, "Preset %s applied", LOG_STR_ARG(climate::climate_preset_to_string(preset)));
    } else {
      ESP_LOGI(TAG, "No changes required to apply preset %s", LOG_STR_ARG(climate::climate_preset_to_string(preset)));
    }
    this->custom_preset.reset();
    this->preset = preset;
  } else {
    ESP_LOGE(TAG, "Preset %s is not configured, ignoring.", LOG_STR_ARG(climate::climate_preset_to_string(preset)));
  }
}

template<typename Out, typename In = Out> Out exchange(Out &output, In &&newValue) {
  auto old = output;
  output = std::forward<In>(newValue);
  return old;
}

bool BangBangClimate::change_preset_internal_(const BangBangClimateTargetTempConfig &config) {
  const auto old_low = exchange(this->target_temperature_low, config.default_temperature_low);
  const auto old_high = exchange(this->target_temperature_high, config.default_temperature_high);

  return old_low != config.default_temperature_low || old_high != config.default_temperature_high;
}

BangBangClimate::BangBangClimate()
    : idle_trigger_(new Trigger<>()), cool_trigger_(new Trigger<>()), heat_trigger_(new Trigger<>()) {}
void BangBangClimate::set_sensor(sensor::Sensor *sensor) { this->sensor_ = sensor; }
void BangBangClimate::set_humidity_sensor(sensor::Sensor *humidity_sensor) { this->humidity_sensor_ = humidity_sensor; }
Trigger<> *BangBangClimate::get_idle_trigger() const { return this->idle_trigger_; }
Trigger<> *BangBangClimate::get_cool_trigger() const { return this->cool_trigger_; }
void BangBangClimate::set_supports_cool(bool supports_cool) { this->supports_cool_ = supports_cool; }
Trigger<> *BangBangClimate::get_heat_trigger() const { return this->heat_trigger_; }
void BangBangClimate::set_supports_heat(bool supports_heat) { this->supports_heat_ = supports_heat; }
void BangBangClimate::dump_config() {
  LOG_CLIMATE("", "Bang Bang Climate", this);
  ESP_LOGCONFIG(TAG, "  Supports HEAT: %s", YESNO(this->supports_heat_));
  ESP_LOGCONFIG(TAG, "  Supports COOL: %s", YESNO(this->supports_cool_));
  ESP_LOGCONFIG(TAG, "  Supports AWAY mode: %s", YESNO(this->preset_config_.count(climate::CLIMATE_PRESET_AWAY) > 0));
  ESP_LOGCONFIG(TAG, "  Default Target Temperature Low: %.2f°C", this->normal_config_.default_temperature_low);
  ESP_LOGCONFIG(TAG, "  Default Target Temperature High: %.2f°C", this->normal_config_.default_temperature_high);
}

BangBangClimateTargetTempConfig::BangBangClimateTargetTempConfig() = default;
BangBangClimateTargetTempConfig::BangBangClimateTargetTempConfig(float default_temperature_low,
                                                                 float default_temperature_high)
    : default_temperature_low(default_temperature_low), default_temperature_high(default_temperature_high) {}

}  // namespace bang_bang
}  // namespace esphome

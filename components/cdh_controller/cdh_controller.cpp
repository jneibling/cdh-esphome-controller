#include "esphome/components/cdh_controller/cdh_controller.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace cdh_controller {

static const char *TAG = "cdh_controller";

// ============================================================
// Run state and error code string lookups
// ============================================================
const char* run_state_str(uint8_t code) {
  switch (code) {
    case 0: return "Off";
    case 1: return "Starting";
    case 2: return "Preheating";
    case 3: return "Igniting";
    case 4: return "Ignition Confirmed";
    case 5: return "Running";
    case 6: return "Running (Thermostat)";
    case 7: return "Shutting Down";
    case 8: return "Cooldown";
    default: return "Unknown";
  }
}

const char* error_code_str(uint8_t code) {
  switch (code) {
    case 0: return "None";
    case 1: return "E-01 Overheat";
    case 2: return "E-02 Glow Plug";
    case 3: return "E-03 Supply Voltage";
    case 4: return "E-04 Ignition Fail";
    case 5: return "E-05 Comms Timeout";
    case 6: return "E-06 Fan Fault";
    case 7: return "E-07 Overheat 2";
    case 8: return "E-08 Flame Out";
    case 10: return "E-10 Fuel Pump";
    case 11: return "E-11 Sensor Fault";
    case 12: return "E-12 High Temp Diff";
    case 13: return "E-13 Overtemp 3";
    default: return "Unknown Error";
  }
}

// ============================================================
// CRC-16/Modbus
// ============================================================
uint16_t CDHController::calc_crc16_(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ============================================================
// Frame validation
// ============================================================
bool CDHController::validate_frame_(const uint8_t *frame) {
  if (frame[0] != FRAME_START) return false;
  if (frame[1] != FRAME_LENGTH) return false;
  uint16_t calc = calc_crc16_(frame, FRAME_SIZE - 2);
  uint16_t recv = (uint16_t)frame[22] | ((uint16_t)frame[23] << 8);
  return calc == recv;
}

int CDHController::find_frame_start_(const uint8_t *buf, int len) {
  for (int i = 0; i < len - 1; i++) {
    if (buf[i] == FRAME_START && buf[i + 1] == FRAME_LENGTH) {
      return i;
    }
  }
  return -1;
}

// ============================================================
// Setup
// ============================================================
void CDHController::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CDH Controller...");

  // Configure relay pin - default LOW = OEM controller connected
  if (this->relay_pin_ != nullptr) {
    this->relay_pin_->setup();
    this->relay_pin_->digital_write(false);
    ESP_LOGI(TAG, "Relay pin configured - OEM controller active (failsafe)");
  }

  // Initialize TX frame with sane defaults
  build_tx_frame_();

  // Publish initial control state
  if (this->control_active_sensor_ != nullptr) {
    this->control_active_sensor_->publish_state(false);
  }

  ESP_LOGI(TAG, "CDH Controller initialized in PASSIVE mode");
}

// ============================================================
// Main loop - handles timing-critical protocol
// ============================================================
void CDHController::loop() {
  uint32_t now = millis();

  if (this->esp_control_active_) {
    // === ACTIVE CONTROL MODE ===
    // Send command frame every TX_INTERVAL_MS
    if (now - this->last_tx_time_ >= TX_INTERVAL_MS) {
      this->last_tx_time_ = now;

      // Build and send command frame
      build_tx_frame_();
      send_command_frame_();

      // Wait for response
      uint8_t rx_frame[FRAME_SIZE];
      if (read_response_frame_(rx_frame, RX_TIMEOUT_MS)) {
        parse_rx_frame_(rx_frame);
        this->last_valid_rx_time_ = millis();
      } else {
        ESP_LOGW(TAG, "No valid response from heater");
        // If no response for too long, maybe switch back to OEM
        if (millis() - this->last_valid_rx_time_ > COMMS_TIMEOUT_MS &&
            this->last_valid_rx_time_ != 0) {
          ESP_LOGE(TAG, "Comms timeout - reverting to OEM controller!");
          set_esp_control(false);
        }
      }
    }
  }
  // Passive mode reading is handled in update()
}

// ============================================================
// Polling update - passive mode frame reading
// ============================================================
void CDHController::update() {
  if (!this->esp_control_active_) {
    // === PASSIVE MONITORING MODE ===
    // Read whatever is in the UART buffer (OEM controller <-> heater traffic)
    parse_passive_frames_();
  }
}

// ============================================================
// Build the 24-byte TX command frame
// ============================================================
void CDHController::build_tx_frame_() {
  memset(this->tx_frame_, 0, FRAME_SIZE);

  this->tx_frame_[0] = FRAME_START;              // Start byte
  this->tx_frame_[1] = FRAME_LENGTH;             // Payload length (22)
  this->tx_frame_[2] = this->heater_on_cmd_ ? 0x01 : 0x00;  // On/Off
  this->tx_frame_[3] = this->ambient_temp_;       // Current ambient temp
  this->tx_frame_[4] = this->desired_temp_;       // Desired temperature
  this->tx_frame_[5] = this->min_pump_freq_raw_;  // Min pump Hz * 10
  this->tx_frame_[6] = this->max_pump_freq_raw_;  // Max pump Hz * 10
  this->tx_frame_[7] = (this->min_fan_rpm_ >> 8) & 0xFF;  // Min fan RPM high
  this->tx_frame_[8] = this->min_fan_rpm_ & 0xFF;         // Min fan RPM low
  this->tx_frame_[9] = (this->max_fan_rpm_ >> 8) & 0xFF;  // Max fan RPM high
  this->tx_frame_[10] = this->max_fan_rpm_ & 0xFF;        // Max fan RPM low
  this->tx_frame_[11] = this->supply_voltage_raw_;  // Voltage * 10
  this->tx_frame_[12] = this->fan_sensor_pulses_;   // Fan sensor pulses
  this->tx_frame_[13] = this->operating_mode_;       // Mode
  this->tx_frame_[14] = this->min_temp_limit_;       // Min temp limit
  this->tx_frame_[15] = this->max_temp_limit_;       // Max temp limit
  this->tx_frame_[16] = this->glow_plug_power_;      // Glow plug power
  this->tx_frame_[17] = this->pump_mode_;             // Pump mode
  this->tx_frame_[18] = (this->altitude_ >> 8) & 0xFF;  // Altitude high
  this->tx_frame_[19] = this->altitude_ & 0xFF;         // Altitude low
  // Bytes 20-21 reserved (0x00)

  // Calculate and append CRC-16
  uint16_t crc = calc_crc16_(this->tx_frame_, FRAME_SIZE - 2);
  this->tx_frame_[22] = crc & 0xFF;         // CRC low
  this->tx_frame_[23] = (crc >> 8) & 0xFF;  // CRC high
}

// ============================================================
// Send command frame over UART
// ============================================================
void CDHController::send_command_frame_() {
  // Flush any stale data in RX buffer before sending
  while (this->available()) {
    uint8_t dummy;
    this->read_byte(&dummy);
  }

  this->write_array(this->tx_frame_, FRAME_SIZE);
  this->flush();

  ESP_LOGV(TAG, "TX: %s",
    format_hex_pretty(this->tx_frame_, FRAME_SIZE).c_str());
}

// ============================================================
// Read response frame with timeout
// ============================================================
bool CDHController::read_response_frame_(uint8_t *frame, uint32_t timeout_ms) {
  uint32_t start = millis();
  uint8_t buf[64];
  int count = 0;

  // Wait for enough bytes to arrive
  while (millis() - start < timeout_ms) {
    while (this->available() && count < 64) {
      this->read_byte(&buf[count++]);
      if (count >= FRAME_SIZE) {
        // Look for a valid frame in what we've received
        int pos = find_frame_start_(buf, count);
        if (pos >= 0 && (count - pos) >= FRAME_SIZE) {
          memcpy(frame, &buf[pos], FRAME_SIZE);
          if (validate_frame_(frame)) {
            ESP_LOGV(TAG, "RX: %s",
              format_hex_pretty(frame, FRAME_SIZE).c_str());
            return true;
          }
        }
      }
    }
    delay(1);
  }

  // Last chance - check what we have
  if (count >= FRAME_SIZE) {
    int pos = find_frame_start_(buf, count);
    if (pos >= 0 && (count - pos) >= FRAME_SIZE) {
      memcpy(frame, &buf[pos], FRAME_SIZE);
      if (validate_frame_(frame)) {
        ESP_LOGV(TAG, "RX (late): %s",
          format_hex_pretty(frame, FRAME_SIZE).c_str());
        return true;
      }
    }
  }

  return false;
}

// ============================================================
// Parse passive mode frames (48 bytes = TX + RX)
// The daoudeddy component reads both the OEM controller's
// TX frame and the heater's RX response as one 48-byte block.
// We want the HEATER's response (second 24 bytes).
// ============================================================
void CDHController::parse_passive_frames_() {
  int avail = this->available();
  if (avail < FRAME_SIZE) return;

  // Read all available bytes
  uint8_t buf[512];
  int count = 0;
  while (this->available() && count < 512) {
    this->read_byte(&buf[count++]);
  }

  ESP_LOGV(TAG, "Passive: read %d bytes", count);

  // Scan for frame pairs (TX frame + RX frame)
  // We look for two consecutive valid frames
  for (int i = 0; i < count - FULL_FRAME_SIZE + 1; i++) {
    if (buf[i] == FRAME_START && buf[i + 1] == FRAME_LENGTH) {
      // Potential TX frame start
      if (i + FULL_FRAME_SIZE <= count) {
        uint8_t *tx_frame = &buf[i];
        uint8_t *rx_frame = &buf[i + FRAME_SIZE];

        // Validate both frames
        if (validate_frame_(tx_frame) && validate_frame_(rx_frame)) {
          // Parse the TX frame to capture OEM controller settings
          // (useful for learning default values)
          this->ambient_temp_ = tx_frame[3];

          // Parse the heater's response
          parse_rx_frame_(rx_frame);
          this->last_valid_rx_time_ = millis();

          ESP_LOGV(TAG, "Passive TX: %s",
            format_hex_pretty(tx_frame, FRAME_SIZE).c_str());
          ESP_LOGV(TAG, "Passive RX: %s",
            format_hex_pretty(rx_frame, FRAME_SIZE).c_str());
          return;
        }
      }
    }
  }

  // If we didn't find a frame pair, try to find just a single
  // heater response frame (might happen if timing catches partial)
  for (int i = 0; i < count - FRAME_SIZE + 1; i++) {
    if (buf[i] == FRAME_START && buf[i + 1] == FRAME_LENGTH) {
      uint8_t *frame = &buf[i];
      if (validate_frame_(frame)) {
        // Could be either TX or RX - parse it as RX anyway
        // The sensor values will only make sense for the RX frame
        // but this is a best-effort fallback
        parse_rx_frame_(frame);
        this->last_valid_rx_time_ = millis();
        return;
      }
    }
  }
}

// ============================================================
// Parse the heater's 24-byte response frame
// ============================================================
void CDHController::parse_rx_frame_(const uint8_t *frame) {
  // Byte 2: Run state
  uint8_t run_state = frame[2];
  if (this->run_state_sensor_ != nullptr) {
    this->run_state_sensor_->publish_state(run_state_str(run_state));
  }

  // Byte 3: Error code
  uint8_t error = frame[3];
  if (this->error_code_sensor_ != nullptr) {
    this->error_code_sensor_->publish_state(error_code_str(error));
  }

  // Byte 4: Supply voltage (value / 10)
  float voltage = frame[4] / 10.0f;
  if (this->supply_voltage_sensor_ != nullptr) {
    this->supply_voltage_sensor_->publish_state(voltage);
  }

  // Bytes 5-6: Fan speed RPM (big-endian)
  uint16_t fan_rpm = ((uint16_t)frame[5] << 8) | frame[6];
  if (this->fan_speed_sensor_ != nullptr) {
    this->fan_speed_sensor_->publish_state(fan_rpm);
  }

  // Bytes 7-8: Fan voltage (big-endian, / 10)
  float fan_volt = (((uint16_t)frame[7] << 8) | frame[8]) / 10.0f;
  if (this->fan_voltage_sensor_ != nullptr) {
    this->fan_voltage_sensor_->publish_state(fan_volt);
  }

  // Byte 9: Heat exchanger temperature
  uint8_t heat_ex = frame[9];
  if (this->heat_exchanger_sensor_ != nullptr) {
    this->heat_exchanger_sensor_->publish_state(heat_ex);
  }

  // Bytes 10-11: Glow plug voltage (big-endian, / 10)
  float gp_volt = (((uint16_t)frame[10] << 8) | frame[11]) / 10.0f;
  if (this->glow_plug_voltage_sensor_ != nullptr) {
    this->glow_plug_voltage_sensor_->publish_state(gp_volt);
  }

  // Byte 12: Glow plug current (* 10 = mA, or direct A reading
  // depending on firmware - publish raw and let user scale)
  float gp_current = frame[12] / 10.0f;
  if (this->glow_plug_current_sensor_ != nullptr) {
    this->glow_plug_current_sensor_->publish_state(gp_current);
  }

  // Bytes 13-14: Actual pump frequency
  // Encoding varies by firmware. Common: byte 13 = Hz * 10 (single byte)
  // Some use byte 14 as fractional. We'll use byte 13 / 10.0
  float pump_hz = frame[13] / 10.0f;
  if (this->pump_frequency_sensor_ != nullptr) {
    this->pump_frequency_sensor_->publish_state(pump_hz);
  }

  // Byte 15: Stored error code
  // 0xFA = no stored error

  // On/Off state - derived from run state
  bool is_on = (run_state >= 1 && run_state <= 6);
  if (this->on_off_sensor_ != nullptr) {
    this->on_off_sensor_->publish_state(is_on);
  }

  // Update ambient temp from heater's current temp reading if available
  // In passive mode, we already grabbed it from the TX frame.
  // In active mode, the heater doesn't send ambient temp in RX -
  // we may want to get it from an external sensor instead.
  // For now, byte 9 (heat exchanger) is the closest internal reading.

  // Current temperature - this comes from the OEM controller's TX frame
  // (byte 3). In passive mode we captured it above. In active mode,
  // we can use an external DS18B20 or the heat exchanger as a proxy.
  if (this->current_temp_sensor_ != nullptr && !this->esp_control_active_) {
    this->current_temp_sensor_->publish_state(this->ambient_temp_);
  }

  // Desired temperature from TX frame (byte 4) - only in passive mode
  if (this->desired_temp_sensor_ != nullptr && !this->esp_control_active_) {
    // In passive mode, we'd need to parse from TX frame.
    // For active mode, it's what we're commanding.
    if (this->esp_control_active_) {
      this->desired_temp_sensor_->publish_state(this->desired_temp_);
    }
  }
}

// ============================================================
// Control methods - called from HA entities
// ============================================================

void CDHController::set_heater_on(bool on) {
  if (!this->esp_control_active_) {
    ESP_LOGW(TAG, "Cannot control heater - ESP control not active");
    return;
  }
  this->heater_on_cmd_ = on;
  ESP_LOGI(TAG, "Heater %s", on ? "ON" : "OFF");
  if (this->heater_switch_ != nullptr) {
    this->heater_switch_->publish_state(on);
  }
}

void CDHController::set_esp_control(bool active) {
  if (active == this->esp_control_active_) return;

  this->esp_control_active_ = active;

  if (active) {
    // Activate ESP32 control
    ESP_LOGI(TAG, "Switching to ESP32 control mode");

    // Energize relay to disconnect OEM controller, connect ESP32 TX
    if (this->relay_pin_ != nullptr) {
      this->relay_pin_->digital_write(true);
    }

    // Flush UART buffer
    while (this->available()) {
      uint8_t dummy;
      this->read_byte(&dummy);
    }

    // Reset timing
    this->last_tx_time_ = 0;
    this->last_valid_rx_time_ = millis();

  } else {
    // Revert to OEM controller
    ESP_LOGI(TAG, "Reverting to OEM controller (passive monitoring)");

    // SAFETY: Send a final OFF command before releasing control
    // Only if we were running the heater
    if (this->heater_on_cmd_) {
      ESP_LOGW(TAG, "Sending shutdown command before releasing control");
      this->heater_on_cmd_ = false;
      build_tx_frame_();
      send_command_frame_();
      // Give heater time to register the off command
      delay(100);
      send_command_frame_();
      delay(100);
    }

    // De-energize relay = OEM controller reconnected
    if (this->relay_pin_ != nullptr) {
      this->relay_pin_->digital_write(false);
    }
  }

  if (this->control_mode_switch_ != nullptr) {
    this->control_mode_switch_->publish_state(active);
  }
  if (this->control_active_sensor_ != nullptr) {
    this->control_active_sensor_->publish_state(active);
  }
}

void CDHController::set_desired_temp(float value) {
  this->desired_temp_ = (uint8_t)value;
  ESP_LOGI(TAG, "Desired temp: %d°C", this->desired_temp_);
  if (this->desired_temp_number_ != nullptr) {
    this->desired_temp_number_->publish_state(value);
  }
}

void CDHController::set_min_pump_hz(float value) {
  this->min_pump_freq_raw_ = (uint8_t)(value * 10);
  ESP_LOGI(TAG, "Min pump Hz: %.1f", value);
  if (this->min_pump_hz_number_ != nullptr) {
    this->min_pump_hz_number_->publish_state(value);
  }
}

void CDHController::set_max_pump_hz(float value) {
  this->max_pump_freq_raw_ = (uint8_t)(value * 10);
  ESP_LOGI(TAG, "Max pump Hz: %.1f", value);
  if (this->max_pump_hz_number_ != nullptr) {
    this->max_pump_hz_number_->publish_state(value);
  }
}

void CDHController::set_min_fan_rpm(float value) {
  this->min_fan_rpm_ = (uint16_t)value;
  ESP_LOGI(TAG, "Min fan RPM: %u", this->min_fan_rpm_);
  if (this->min_fan_rpm_number_ != nullptr) {
    this->min_fan_rpm_number_->publish_state(value);
  }
}

void CDHController::set_max_fan_rpm(float value) {
  this->max_fan_rpm_ = (uint16_t)value;
  ESP_LOGI(TAG, "Max fan RPM: %u", this->max_fan_rpm_);
  if (this->max_fan_rpm_number_ != nullptr) {
    this->max_fan_rpm_number_->publish_state(value);
  }
}

void CDHController::set_altitude(float value) {
  this->altitude_ = (uint16_t)value;
  ESP_LOGI(TAG, "Altitude: %um", this->altitude_);
  if (this->altitude_number_ != nullptr) {
    this->altitude_number_->publish_state(value);
  }
}

void CDHController::set_supply_voltage_setting(float value) {
  this->supply_voltage_raw_ = (uint8_t)(value * 10);
  ESP_LOGI(TAG, "Supply voltage: %.1fV", value);
  if (this->supply_voltage_number_ != nullptr) {
    this->supply_voltage_number_->publish_state(value);
  }
}

void CDHController::set_glow_plug_power(float value) {
  this->glow_plug_power_ = (uint8_t)value;
  ESP_LOGI(TAG, "Glow plug power: %d", this->glow_plug_power_);
  if (this->glow_plug_power_number_ != nullptr) {
    this->glow_plug_power_number_->publish_state(value);
  }
}

void CDHController::set_min_temp_limit(float value) {
  this->min_temp_limit_ = (uint8_t)value;
  if (this->min_temp_number_ != nullptr) {
    this->min_temp_number_->publish_state(value);
  }
}

void CDHController::set_max_temp_limit(float value) {
  this->max_temp_limit_ = (uint8_t)value;
  if (this->max_temp_number_ != nullptr) {
    this->max_temp_number_->publish_state(value);
  }
}

void CDHController::set_operating_mode(const std::string &mode) {
  if (mode == "Thermostat") {
    this->operating_mode_ = MODE_THERMOSTAT;
  } else if (mode == "Fixed Hz") {
    this->operating_mode_ = MODE_FIXED_HZ;
  }
  ESP_LOGI(TAG, "Operating mode: %s (0x%02X)", mode.c_str(), this->operating_mode_);
  if (this->operating_mode_select_ != nullptr) {
    this->operating_mode_select_->publish_state(mode);
  }
}

// ============================================================
// Dump config
// ============================================================
void CDHController::dump_config() {
  ESP_LOGCONFIG(TAG, "CDH Controller:");
  ESP_LOGCONFIG(TAG, "  Relay pin: %s", this->relay_pin_ ? "configured" : "none");
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->esp_control_active_ ? "ESP32 Control" : "Passive Monitor");
}

// ============================================================
// Switch implementations
// ============================================================
void HeaterSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) {
    this->parent_->set_heater_on(state);
  }
  this->publish_state(state);
}

void ControlModeSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) {
    this->parent_->set_esp_control(state);
  }
  // State published by set_esp_control
}

// ============================================================
// Number implementation
// ============================================================
void CDHNumber::control(float value) {
  if (this->parent_ == nullptr) return;

  switch (this->param_type_) {
    case PARAM_DESIRED_TEMP:     this->parent_->set_desired_temp(value); break;
    case PARAM_MIN_PUMP_HZ:      this->parent_->set_min_pump_hz(value); break;
    case PARAM_MAX_PUMP_HZ:      this->parent_->set_max_pump_hz(value); break;
    case PARAM_MIN_FAN_RPM:      this->parent_->set_min_fan_rpm(value); break;
    case PARAM_MAX_FAN_RPM:      this->parent_->set_max_fan_rpm(value); break;
    case PARAM_ALTITUDE:         this->parent_->set_altitude(value); break;
    case PARAM_SUPPLY_VOLTAGE:   this->parent_->set_supply_voltage_setting(value); break;
    case PARAM_GLOW_PLUG_POWER:  this->parent_->set_glow_plug_power(value); break;
    case PARAM_MIN_TEMP:         this->parent_->set_min_temp_limit(value); break;
    case PARAM_MAX_TEMP:         this->parent_->set_max_temp_limit(value); break;
  }
  this->publish_state(value);
}

// ============================================================
// Select implementation
// ============================================================
void CDHSelect::control(const std::string &value) {
  if (this->parent_ != nullptr) {
    this->parent_->set_operating_mode(value);
  }
  this->publish_state(value);
}

}  // namespace cdh_controller
}  // namespace esphome

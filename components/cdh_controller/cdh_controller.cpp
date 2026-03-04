#include "cdh_controller.h"
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
// Frame validation (CRC is big-endian in this protocol)
// ============================================================
bool CDHController::validate_frame_(const uint8_t *frame) {
  if (frame[0] != FRAME_START) return false;
  if (frame[1] != FRAME_LENGTH) return false;
  uint16_t calc = calc_crc16_(frame, FRAME_SIZE - 2);
  uint16_t recv = ((uint16_t)frame[22] << 8) | (uint16_t)frame[23];
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
// Identify if a frame is TX (controller→heater) or RX (heater→controller)
// TX frames have operating mode byte at [13] = 0xCD or 0xA5
// RX frames will have small values at [13] (pump freq data)
// ============================================================
bool CDHController::is_tx_frame_(const uint8_t *frame) {
  return (frame[13] == MODE_THERMOSTAT || frame[13] == MODE_FIXED_HZ);
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
        // If no response for too long, switch back to OEM
      //  if (millis() - this->last_valid_rx_time_ > COMMS_TIMEOUT_MS &&
      //      this->last_valid_rx_time_ != 0) {
      //    ESP_LOGE(TAG, "Comms timeout - reverting to OEM controller!");
      //    set_esp_control(false);
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
    parse_passive_frames_();
  }
}

// ============================================================
// Build the 24-byte TX command frame
// Byte layout matches OEM controller (verified from captured frames):
//   [0]  0x76 start
//   [1]  0x16 length (22)
//   [2]  On/Off command
//   [3]  0x00 (reserved/padding)
//   [4]  Desired temperature
//   [5]  Min pump frequency (*10)
//   [6]  Max pump frequency (*10)
//   [7-8]  Min fan RPM (big-endian)
//   [9-10] Max fan RPM (big-endian)
//   [11] Supply voltage (*10)
//   [12] Fan sensor pulses
//   [13] Operating mode (0xCD=thermostat, 0xA5=fixed Hz)
//   [14] Min temp limit
//   [15] Max temp limit
//   [16] Glow plug power
//   [17] Pump mode (0=auto)
//   [18-19] Reserved/unknown
//   [20-21] Altitude (big-endian)
//   [22-23] CRC-16 (big-endian)
// ============================================================
void CDHController::build_tx_frame_() {
  memset(this->tx_frame_, 0, FRAME_SIZE);

  this->tx_frame_[0] = FRAME_START;              // Start byte
  this->tx_frame_[1] = FRAME_LENGTH;             // Payload length (22)
  this->tx_frame_[2] = this->heater_on_cmd_ ? 0x01 : 0x00;  // On/Off
  this->tx_frame_[3] = 0x00;                      // Reserved (OEM sends 0x00)
  this->tx_frame_[4] = this->desired_temp_;        // Desired temperature
  this->tx_frame_[5] = this->min_pump_freq_raw_;   // Min pump Hz * 10
  this->tx_frame_[6] = this->max_pump_freq_raw_;   // Max pump Hz * 10
  this->tx_frame_[7] = (this->min_fan_rpm_ >> 8) & 0xFF;  // Min fan RPM high
  this->tx_frame_[8] = this->min_fan_rpm_ & 0xFF;         // Min fan RPM low
  this->tx_frame_[9] = (this->max_fan_rpm_ >> 8) & 0xFF;  // Max fan RPM high
  this->tx_frame_[10] = this->max_fan_rpm_ & 0xFF;        // Max fan RPM low
  this->tx_frame_[11] = this->supply_voltage_raw_;  // Voltage * 10
  this->tx_frame_[12] = this->fan_sensor_pulses_;   // Fan sensor pulses
  this->tx_frame_[13] = this->operating_mode_;       // Mode (0xCD/0xA5)
  this->tx_frame_[14] = this->min_temp_limit_;       // Min temp limit
  this->tx_frame_[15] = this->max_temp_limit_;       // Max temp limit
  this->tx_frame_[16] = this->glow_plug_power_;      // Glow plug power
  this->tx_frame_[17] = this->pump_mode_;             // Pump mode (0=auto)
  this->tx_frame_[18] = 0x00;                         // Reserved
  this->tx_frame_[19] = 0x00;                         // Reserved
  this->tx_frame_[20] = (this->altitude_ >> 8) & 0xFF;  // Altitude high
  this->tx_frame_[21] = this->altitude_ & 0xFF;         // Altitude low

  // Calculate and append CRC-16 (big-endian)
  uint16_t crc = calc_crc16_(this->tx_frame_, FRAME_SIZE - 2);
  this->tx_frame_[22] = (crc >> 8) & 0xFF;  // CRC high
  this->tx_frame_[23] = crc & 0xFF;         // CRC low
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
// Parse passive mode frames
// Reads the continuous stream of OEM controller ↔ heater traffic.
// Identifies TX vs RX frames by content (byte[13] = 0xCD/0xA5
// is unique to TX frames as the operating mode field).
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

  // Debug: dump first 48 bytes
  int dump_len = count < 48 ? count : 48;
  ESP_LOGV(TAG, "Raw: %s", format_hex_pretty(buf, dump_len).c_str());

  // Strategy: Find any two consecutive valid frames, then identify
  // which is TX and which is RX by content (not position).
  for (int i = 0; i < count - FULL_FRAME_SIZE + 1; i++) {
    if (buf[i] == FRAME_START && buf[i + 1] == FRAME_LENGTH) {
      if (i + FULL_FRAME_SIZE <= count) {
        uint8_t *frame_a = &buf[i];
        uint8_t *frame_b = &buf[i + FRAME_SIZE];

        // Validate both frames
        if (validate_frame_(frame_a) && validate_frame_(frame_b)) {
          uint8_t *tx_frame = nullptr;
          uint8_t *rx_frame = nullptr;

          // Identify which is TX and which is RX
          if (is_tx_frame_(frame_a) && !is_tx_frame_(frame_b)) {
            tx_frame = frame_a;
            rx_frame = frame_b;
          } else if (!is_tx_frame_(frame_a) && is_tx_frame_(frame_b)) {
            tx_frame = frame_b;
            rx_frame = frame_a;
          } else {
            // Both look like same type - skip this pair
            ESP_LOGW(TAG, "Could not distinguish TX/RX pair");
            continue;
          }

          // Parse TX frame for OEM controller settings
          parse_tx_frame_(tx_frame);

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

  // Fallback: find a single valid frame
  for (int i = 0; i < count - FRAME_SIZE + 1; i++) {
    if (buf[i] == FRAME_START && buf[i + 1] == FRAME_LENGTH) {
      uint8_t *frame = &buf[i];
      if (validate_frame_(frame)) {
        if (is_tx_frame_(frame)) {
          // It's a TX frame - extract OEM settings
          parse_tx_frame_(frame);
          ESP_LOGV(TAG, "Passive single TX: %s",
            format_hex_pretty(frame, FRAME_SIZE).c_str());
        } else {
          // It's an RX frame - parse heater data
          parse_rx_frame_(frame);
          this->last_valid_rx_time_ = millis();
          ESP_LOGV(TAG, "Passive single RX: %s",
            format_hex_pretty(frame, FRAME_SIZE).c_str());
        }
        return;
      }
    }
  }
}

// ============================================================
// Parse OEM controller TX frame (for learning settings)
// TX byte layout (verified from captured OEM frames):
//   [3]  = 0x00 reserved
//   [4]  = Desired temperature
//   [5]  = Min pump freq (*10)
//   [6]  = Max pump freq (*10)
//   [7-8]  = Min fan RPM
//   [9-10] = Max fan RPM
//   [11] = Supply voltage (*10)
//   [13] = Operating mode
// ============================================================
void CDHController::parse_tx_frame_(const uint8_t *frame) {
  // Capture ambient/desired temp from OEM controller
  this->ambient_temp_ = frame[4];  // OEM sends current reading here

  // Publish desired temperature from OEM controller
  if (this->desired_temp_sensor_ != nullptr && !this->esp_control_active_) {
    this->desired_temp_sensor_->publish_state(frame[4]);
  }
}

// ============================================================
// Parse the heater's 24-byte response frame
// RX byte layout (verified from captured heater responses):
//   [0]  0x76 start
//   [1]  0x16 length
//   [2]  Run state (0=Off, 1=Starting, ..., 5=Running, ...)
//   [3]  On/off acknowledgement
//   [4]  Error code (0=None, 1=E-01, ...)
//   [5]  Supply voltage raw (/10 for volts)
//   [6-7]  Fan speed RPM (big-endian)
//   [8-9]  Fan voltage (big-endian, /10)
//   [10] Unknown
//   [11] Heat exchanger temperature (°C)
//   [12-13] Glow plug voltage (big-endian, /10)
//   [14] Glow plug current (/100 for amps)
//   [15] Unknown
//   [16] Pump frequency (/10 for Hz)
//   [17-18] Unknown
//   [19] Body/ambient temperature (°C)
//   [20-21] Unknown
//   [22-23] CRC-16 (big-endian)
// ============================================================
void CDHController::parse_rx_frame_(const uint8_t *frame) {
  // Byte 2: Run state
  uint8_t run_state = frame[2];
  if (this->run_state_sensor_ != nullptr) {
    this->run_state_sensor_->publish_state(run_state_str(run_state));
  }

  // Byte 4: Error code (byte 3 is on/off ack, NOT error)
  uint8_t error = frame[4];
  if (this->error_code_sensor_ != nullptr) {
    this->error_code_sensor_->publish_state(error_code_str(error));
  }

  // Byte 5: Supply voltage (value / 10)
  float voltage = frame[5] / 10.0f;
  if (this->supply_voltage_sensor_ != nullptr) {
    this->supply_voltage_sensor_->publish_state(voltage);
  }

  // Bytes 6-7: Fan speed RPM (big-endian)
  uint16_t fan_rpm = ((uint16_t)frame[6] << 8) | frame[7];
  if (this->fan_speed_sensor_ != nullptr) {
    this->fan_speed_sensor_->publish_state(fan_rpm);
  }

  // Bytes 8-9: Fan voltage (big-endian, / 10)
  float fan_volt = (((uint16_t)frame[8] << 8) | frame[9]) / 10.0f;
  if (this->fan_voltage_sensor_ != nullptr) {
    this->fan_voltage_sensor_->publish_state(fan_volt);
  }

  // Byte 11: Heat exchanger temperature
  uint8_t heat_ex = frame[11];
  if (this->heat_exchanger_sensor_ != nullptr) {
    this->heat_exchanger_sensor_->publish_state(heat_ex);
  }

  // Bytes 12-13: Glow plug voltage (big-endian, / 10)
  float gp_volt = (((uint16_t)frame[12] << 8) | frame[13]) / 10.0f;
  if (this->glow_plug_voltage_sensor_ != nullptr) {
    this->glow_plug_voltage_sensor_->publish_state(gp_volt);
  }

  // Byte 14: Glow plug current (/ 100 for amps)
  float gp_current = frame[14] / 100.0f;
  if (this->glow_plug_current_sensor_ != nullptr) {
    this->glow_plug_current_sensor_->publish_state(gp_current);
  }

  // Byte 16: Actual pump frequency (/ 10 for Hz)
  float pump_hz = frame[16] / 10.0f;
  if (this->pump_frequency_sensor_ != nullptr) {
    this->pump_frequency_sensor_->publish_state(pump_hz);
  }

  // On/Off state - derived from run state
  bool is_on = (run_state >= 1 && run_state <= 6);
  if (this->on_off_sensor_ != nullptr) {
    this->on_off_sensor_->publish_state(is_on);
  }

  // Current temperature (body/ambient from heater)
  // Byte 19 appears to be body temperature reading
  if (this->current_temp_sensor_ != nullptr) {
    this->current_temp_sensor_->publish_state(frame[19]);
  }

  // Desired temperature - published from TX frame in parse_tx_frame_()
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
    if (this->heater_on_cmd_) {
      ESP_LOGW(TAG, "Sending shutdown command before releasing control");
      this->heater_on_cmd_ = false;
      build_tx_frame_();
      send_command_frame_();
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

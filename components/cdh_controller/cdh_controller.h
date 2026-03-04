#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace cdh_controller {

// Protocol constants
static const uint8_t FRAME_START = 0x76;
static const uint8_t FRAME_LENGTH = 0x16;  // 22 payload bytes
static const uint8_t FRAME_SIZE = 24;
static const uint8_t FULL_FRAME_SIZE = 48;  // TX + RX in passive mode
static const uint32_t BAUD_RATE = 25000;

// Operating modes
static const uint8_t MODE_THERMOSTAT = 0xCD;
static const uint8_t MODE_FIXED_HZ = 0xA5;

// Run state codes
static const char* run_state_str(uint8_t code);
static const char* error_code_str(uint8_t code);

class CDHController;

// ---- Switch: Heater On/Off ----
class HeaterSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(CDHController *parent) { parent_ = parent; }
  void write_state(bool state) override;
 protected:
  CDHController *parent_{nullptr};
};

// ---- Switch: ESP Control Mode ----
class ControlModeSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(CDHController *parent) { parent_ = parent; }
  void write_state(bool state) override;
 protected:
  CDHController *parent_{nullptr};
};

// ---- Number entities for control parameters ----
class CDHNumber : public number::Number, public Component {
 public:
  void set_parent(CDHController *parent) { parent_ = parent; }
  void set_param_type(uint8_t type) { param_type_ = type; }
  void control(float value) override;
 protected:
  CDHController *parent_{nullptr};
  uint8_t param_type_{0};
};

// Parameter type IDs for CDHNumber
static const uint8_t PARAM_DESIRED_TEMP = 1;
static const uint8_t PARAM_MIN_PUMP_HZ = 2;
static const uint8_t PARAM_MAX_PUMP_HZ = 3;
static const uint8_t PARAM_MIN_FAN_RPM = 4;
static const uint8_t PARAM_MAX_FAN_RPM = 5;
static const uint8_t PARAM_ALTITUDE = 6;
static const uint8_t PARAM_SUPPLY_VOLTAGE = 7;
static const uint8_t PARAM_GLOW_PLUG_POWER = 8;
static const uint8_t PARAM_MIN_TEMP = 9;
static const uint8_t PARAM_MAX_TEMP = 10;

// ---- Select: Operating Mode ----
class CDHSelect : public select::Select, public Component {
 public:
  void set_parent(CDHController *parent) { parent_ = parent; }
  void control(const std::string &value) override;
 protected:
  CDHController *parent_{nullptr};
};

// ---- Main Controller Component ----
class CDHController : public PollingComponent, public uart::UARTDevice {
 public:
  CDHController() : PollingComponent(1000) {}

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // --- Configuration setters (called by Python codegen) ---
  void set_relay_pin(GPIOPin *pin) { relay_pin_ = pin; }

  // Sensor setters
  void set_current_temperature_sensor(sensor::Sensor *s) { current_temp_sensor_ = s; }
  void set_fan_speed_sensor(sensor::Sensor *s) { fan_speed_sensor_ = s; }
  void set_supply_voltage_sensor(sensor::Sensor *s) { supply_voltage_sensor_ = s; }
  void set_heat_exchanger_temp_sensor(sensor::Sensor *s) { heat_exchanger_sensor_ = s; }
  void set_glow_plug_voltage_sensor(sensor::Sensor *s) { glow_plug_voltage_sensor_ = s; }
  void set_glow_plug_current_sensor(sensor::Sensor *s) { glow_plug_current_sensor_ = s; }
  void set_pump_frequency_sensor(sensor::Sensor *s) { pump_frequency_sensor_ = s; }
  void set_fan_voltage_sensor(sensor::Sensor *s) { fan_voltage_sensor_ = s; }
  void set_desired_temperature_sensor(sensor::Sensor *s) { desired_temp_sensor_ = s; }
  void set_run_state_sensor(text_sensor::TextSensor *s) { run_state_sensor_ = s; }
  void set_error_code_sensor(text_sensor::TextSensor *s) { error_code_sensor_ = s; }
  void set_on_off_sensor(binary_sensor::BinarySensor *s) { on_off_sensor_ = s; }
  void set_control_active_sensor(binary_sensor::BinarySensor *s) { control_active_sensor_ = s; }

  // Switch setters
  void set_heater_switch(HeaterSwitch *s) { heater_switch_ = s; }
  void set_control_mode_switch(ControlModeSwitch *s) { control_mode_switch_ = s; }

  // Number setters
  void set_desired_temp_number(CDHNumber *n) { desired_temp_number_ = n; }
  void set_min_pump_hz_number(CDHNumber *n) { min_pump_hz_number_ = n; }
  void set_max_pump_hz_number(CDHNumber *n) { max_pump_hz_number_ = n; }
  void set_min_fan_rpm_number(CDHNumber *n) { min_fan_rpm_number_ = n; }
  void set_max_fan_rpm_number(CDHNumber *n) { max_fan_rpm_number_ = n; }
  void set_altitude_number(CDHNumber *n) { altitude_number_ = n; }
  void set_supply_voltage_number(CDHNumber *n) { supply_voltage_number_ = n; }
  void set_glow_plug_power_number(CDHNumber *n) { glow_plug_power_number_ = n; }
  void set_min_temp_number(CDHNumber *n) { min_temp_number_ = n; }
  void set_max_temp_number(CDHNumber *n) { max_temp_number_ = n; }

  // Select setter
  void set_operating_mode_select(CDHSelect *s) { operating_mode_select_ = s; }

  // --- Control methods (called by switches/numbers/selects) ---
  void set_heater_on(bool on);
  void set_esp_control(bool active);
  void set_desired_temp(float value);
  void set_min_pump_hz(float value);
  void set_max_pump_hz(float value);
  void set_min_fan_rpm(float value);
  void set_max_fan_rpm(float value);
  void set_altitude(float value);
  void set_supply_voltage_setting(float value);
  void set_glow_plug_power(float value);
  void set_min_temp_limit(float value);
  void set_max_temp_limit(float value);
  void set_operating_mode(const std::string &mode);

  // --- State access ---
  bool is_esp_control_active() const { return esp_control_active_; }

 protected:
  // --- Protocol methods ---
  void build_tx_frame_();
  void send_command_frame_();
  bool read_response_frame_(uint8_t *frame, uint32_t timeout_ms);
  void parse_rx_frame_(const uint8_t *frame);
  void parse_tx_frame_(const uint8_t *frame);
  void parse_passive_frames_();
  uint16_t calc_crc16_(const uint8_t *data, uint8_t len);
  bool validate_frame_(const uint8_t *frame);
  int find_frame_start_(const uint8_t *buf, int len);
  bool is_tx_frame_(const uint8_t *frame);

  // --- Hardware ---
  GPIOPin *relay_pin_{nullptr};

  // --- Control state (what we command) ---
  bool esp_control_active_{false};
  bool heater_on_cmd_{false};
  uint8_t desired_temp_{22};
  uint8_t min_pump_freq_raw_{16};    // 1.6 Hz * 10
  uint8_t max_pump_freq_raw_{55};    // 5.5 Hz * 10
  uint16_t min_fan_rpm_{1680};
  uint16_t max_fan_rpm_{4500};
  uint8_t supply_voltage_raw_{120};  // 12.0V * 10
  uint8_t fan_sensor_pulses_{1};
  uint8_t operating_mode_{MODE_THERMOSTAT};
  uint8_t min_temp_limit_{8};
  uint8_t max_temp_limit_{35};
  uint8_t glow_plug_power_{5};
  uint8_t pump_mode_{0};            // 0 = auto
  uint16_t altitude_{0};
  uint8_t ambient_temp_{20};        // Updated from last RX frame

  // --- TX/RX buffers ---
  uint8_t tx_frame_[FRAME_SIZE];
  uint8_t rx_buf_[256];

  // --- Timing ---
  uint32_t last_tx_time_{0};
  uint32_t last_valid_rx_time_{0};
  static const uint32_t TX_INTERVAL_MS = 1000;
  static const uint32_t RX_TIMEOUT_MS = 250;
  static const uint32_t COMMS_TIMEOUT_MS = 5000;

  // --- Sensor pointers ---
  sensor::Sensor *current_temp_sensor_{nullptr};
  sensor::Sensor *fan_speed_sensor_{nullptr};
  sensor::Sensor *supply_voltage_sensor_{nullptr};
  sensor::Sensor *heat_exchanger_sensor_{nullptr};
  sensor::Sensor *glow_plug_voltage_sensor_{nullptr};
  sensor::Sensor *glow_plug_current_sensor_{nullptr};
  sensor::Sensor *pump_frequency_sensor_{nullptr};
  sensor::Sensor *fan_voltage_sensor_{nullptr};
  sensor::Sensor *desired_temp_sensor_{nullptr};
  text_sensor::TextSensor *run_state_sensor_{nullptr};
  text_sensor::TextSensor *error_code_sensor_{nullptr};
  binary_sensor::BinarySensor *on_off_sensor_{nullptr};
  binary_sensor::BinarySensor *control_active_sensor_{nullptr};

  // --- Switch pointers ---
  HeaterSwitch *heater_switch_{nullptr};
  ControlModeSwitch *control_mode_switch_{nullptr};

  // --- Number pointers ---
  CDHNumber *desired_temp_number_{nullptr};
  CDHNumber *min_pump_hz_number_{nullptr};
  CDHNumber *max_pump_hz_number_{nullptr};
  CDHNumber *min_fan_rpm_number_{nullptr};
  CDHNumber *max_fan_rpm_number_{nullptr};
  CDHNumber *altitude_number_{nullptr};
  CDHNumber *supply_voltage_number_{nullptr};
  CDHNumber *glow_plug_power_number_{nullptr};
  CDHNumber *min_temp_number_{nullptr};
  CDHNumber *max_temp_number_{nullptr};

  // --- Select pointers ---
  CDHSelect *operating_mode_select_{nullptr};
};

}  // namespace cdh_controller
}  // namespace esphome

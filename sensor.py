import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_SPEED,
    DEVICE_CLASS_FREQUENCY,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_REVOLUTIONS_PER_MINUTE,
)
from . import CDHController, CONF_CDH_CONTROLLER_ID

CONF_CURRENT_TEMPERATURE = "current_temperature"
CONF_FAN_SPEED = "fan_speed"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_HEAT_EXCHANGER_TEMP = "heat_exchanger_temp"
CONF_GLOW_PLUG_VOLTAGE = "glow_plug_voltage"
CONF_GLOW_PLUG_CURRENT = "glow_plug_current"
CONF_PUMP_FREQUENCY = "pump_frequency"
CONF_FAN_VOLTAGE = "fan_voltage"
CONF_DESIRED_TEMPERATURE = "desired_temperature"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CDH_CONTROLLER_ID): cv.use_id(CDHController),
        cv.Optional(CONF_CURRENT_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FAN_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_SPEED,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:fan",
        ),
        cv.Optional(CONF_SUPPLY_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HEAT_EXCHANGER_TEMP): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:thermometer-high",
        ),
        cv.Optional(CONF_GLOW_PLUG_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:flash",
        ),
        cv.Optional(CONF_GLOW_PLUG_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:current-dc",
        ),
        cv.Optional(CONF_PUMP_FREQUENCY): sensor.sensor_schema(
            unit_of_measurement=UNIT_HERTZ,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_FREQUENCY,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:pump",
        ),
        cv.Optional(CONF_FAN_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:fan",
        ),
        cv.Optional(CONF_DESIRED_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:thermometer",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CDH_CONTROLLER_ID])

    for key, setter in [
        (CONF_CURRENT_TEMPERATURE, "set_current_temperature_sensor"),
        (CONF_FAN_SPEED, "set_fan_speed_sensor"),
        (CONF_SUPPLY_VOLTAGE, "set_supply_voltage_sensor"),
        (CONF_HEAT_EXCHANGER_TEMP, "set_heat_exchanger_temp_sensor"),
        (CONF_GLOW_PLUG_VOLTAGE, "set_glow_plug_voltage_sensor"),
        (CONF_GLOW_PLUG_CURRENT, "set_glow_plug_current_sensor"),
        (CONF_PUMP_FREQUENCY, "set_pump_frequency_sensor"),
        (CONF_FAN_VOLTAGE, "set_fan_voltage_sensor"),
        (CONF_DESIRED_TEMPERATURE, "set_desired_temperature_sensor"),
    ]:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(parent, setter)(sens))

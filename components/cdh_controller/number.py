import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_VOLT,
)
from . import CDHController, CDHNumber, CONF_CDH_CONTROLLER_ID, cdh_controller_ns

# Parameter type constants (must match C++ header)
PARAM_DESIRED_TEMP = 1
PARAM_MIN_PUMP_HZ = 2
PARAM_MAX_PUMP_HZ = 3
PARAM_MIN_FAN_RPM = 4
PARAM_MAX_FAN_RPM = 5
PARAM_ALTITUDE = 6
PARAM_SUPPLY_VOLTAGE = 7
PARAM_GLOW_PLUG_POWER = 8
PARAM_MIN_TEMP = 9
PARAM_MAX_TEMP = 10
PARAM_SET_PUMP_HZ = 11

CONF_DESIRED_TEMP = "desired_temperature"
CONF_MIN_PUMP_HZ = "min_pump_frequency"
CONF_MAX_PUMP_HZ = "max_pump_frequency"
CONF_MIN_FAN_RPM = "min_fan_rpm"
CONF_MAX_FAN_RPM = "max_fan_rpm"
CONF_ALTITUDE = "altitude"
CONF_SUPPLY_VOLTAGE_SET = "supply_voltage_setting"
CONF_GLOW_PLUG_POWER = "glow_plug_power"
CONF_MIN_TEMP_LIMIT = "min_temp_limit"
CONF_MAX_TEMP_LIMIT = "max_temp_limit"
CONF_SET_PUMP_HZ = "set_pump_frequency"

def cdh_number_schema(param_type, **kwargs):
    """Create a number schema for a CDH parameter."""
    return number.number_schema(CDHNumber, **kwargs)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CDH_CONTROLLER_ID): cv.use_id(CDHController),
        cv.Optional(CONF_DESIRED_TEMP): cdh_number_schema(
            PARAM_DESIRED_TEMP,
            icon="mdi:thermometer",
            unit_of_measurement=UNIT_CELSIUS,
        ),
        cv.Optional(CONF_MIN_PUMP_HZ): cdh_number_schema(
            PARAM_MIN_PUMP_HZ,
            icon="mdi:pump",
            unit_of_measurement=UNIT_HERTZ,
        ),
        cv.Optional(CONF_MAX_PUMP_HZ): cdh_number_schema(
            PARAM_MAX_PUMP_HZ,
            icon="mdi:pump",
            unit_of_measurement=UNIT_HERTZ,
        ),
        cv.Optional(CONF_MIN_FAN_RPM): cdh_number_schema(
            PARAM_MIN_FAN_RPM,
            icon="mdi:fan",
            unit_of_measurement="RPM",
        ),
        cv.Optional(CONF_MAX_FAN_RPM): cdh_number_schema(
            PARAM_MAX_FAN_RPM,
            icon="mdi:fan",
            unit_of_measurement="RPM",
        ),
        cv.Optional(CONF_ALTITUDE): cdh_number_schema(
            PARAM_ALTITUDE,
            icon="mdi:elevation-rise",
            unit_of_measurement="m",
        ),
        cv.Optional(CONF_SUPPLY_VOLTAGE_SET): cdh_number_schema(
            PARAM_SUPPLY_VOLTAGE,
            icon="mdi:car-battery",
            unit_of_measurement=UNIT_VOLT,
        ),
        cv.Optional(CONF_GLOW_PLUG_POWER): cdh_number_schema(
            PARAM_GLOW_PLUG_POWER,
            icon="mdi:flash",
        ),
        cv.Optional(CONF_MIN_TEMP_LIMIT): cdh_number_schema(
            PARAM_MIN_TEMP,
            icon="mdi:thermometer-low",
            unit_of_measurement=UNIT_CELSIUS,
        ),
        cv.Optional(CONF_MAX_TEMP_LIMIT): cdh_number_schema(
            PARAM_MAX_TEMP,
            icon="mdi:thermometer-high",
            unit_of_measurement=UNIT_CELSIUS,
        ),
        cv.Optional(CONF_SET_PUMP_HZ): cdh_number_schema(
            PARAM_SET_PUMP_HZ,
            icon="mdi:pump",
            unit_of_measurement=UNIT_HERTZ,
        ),
    }
)


NUMBER_PARAMS = {
    CONF_DESIRED_TEMP:      ("set_desired_temp_number",      PARAM_DESIRED_TEMP),
    CONF_MIN_PUMP_HZ:       ("set_min_pump_hz_number",       PARAM_MIN_PUMP_HZ),
    CONF_MAX_PUMP_HZ:       ("set_max_pump_hz_number",       PARAM_MAX_PUMP_HZ),
    CONF_MIN_FAN_RPM:       ("set_min_fan_rpm_number",       PARAM_MIN_FAN_RPM),
    CONF_MAX_FAN_RPM:       ("set_max_fan_rpm_number",       PARAM_MAX_FAN_RPM),
    CONF_ALTITUDE:          ("set_altitude_number",          PARAM_ALTITUDE),
    CONF_SUPPLY_VOLTAGE_SET:("set_supply_voltage_number",    PARAM_SUPPLY_VOLTAGE),
    CONF_GLOW_PLUG_POWER:   ("set_glow_plug_power_number",  PARAM_GLOW_PLUG_POWER),
    CONF_MIN_TEMP_LIMIT:    ("set_min_temp_number",          PARAM_MIN_TEMP),
    CONF_MAX_TEMP_LIMIT:    ("set_max_temp_number",          PARAM_MAX_TEMP),
    CONF_SET_PUMP_HZ:       ("set_set_pump_freq_number",     PARAM_SET_PUMP_HZ),
}


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CDH_CONTROLLER_ID])

    for key, (setter, param_type) in NUMBER_PARAMS.items():
        if key in config:
            num = await number.new_number(
                config[key],
                min_value=_get_min(key),
                max_value=_get_max(key),
                step=_get_step(key),
            )
            await cg.register_component(num, config[key])
            cg.add(num.set_parent(parent))
            cg.add(num.set_param_type(param_type))
            cg.add(getattr(parent, setter)(num))


def _get_min(key):
    return {
        CONF_DESIRED_TEMP: 8,
        CONF_MIN_PUMP_HZ: 0.8,
        CONF_MAX_PUMP_HZ: 0.8,
        CONF_MIN_FAN_RPM: 1000,
        CONF_MAX_FAN_RPM: 1000,
        CONF_ALTITUDE: 0,
        CONF_SUPPLY_VOLTAGE_SET: 10.0,
        CONF_GLOW_PLUG_POWER: 1,
        CONF_MIN_TEMP_LIMIT: 1,
        CONF_MAX_TEMP_LIMIT: 10,
        CONF_SET_PUMP_HZ: 0.8,
    }.get(key, 0)

def _get_max(key):
    return {
        CONF_DESIRED_TEMP: 40,
        CONF_MIN_PUMP_HZ: 6.0,
        CONF_MAX_PUMP_HZ: 6.0,
        CONF_MIN_FAN_RPM: 5000,
        CONF_MAX_FAN_RPM: 5000,
        CONF_ALTITUDE: 3500,
        CONF_SUPPLY_VOLTAGE_SET: 30.0,
        CONF_GLOW_PLUG_POWER: 10,
        CONF_MIN_TEMP_LIMIT: 20,
        CONF_MAX_TEMP_LIMIT: 45,
        CONF_SET_PUMP_HZ: 6.0,
    }.get(key, 100)

def _get_step(key):
    return {
        CONF_DESIRED_TEMP: 1,
        CONF_MIN_PUMP_HZ: 0.1,
        CONF_MAX_PUMP_HZ: 0.1,
        CONF_MIN_FAN_RPM: 10,
        CONF_MAX_FAN_RPM: 10,
        CONF_ALTITUDE: 100,
        CONF_SUPPLY_VOLTAGE_SET: 0.1,
        CONF_GLOW_PLUG_POWER: 1,
        CONF_MIN_TEMP_LIMIT: 1,
        CONF_MAX_TEMP_LIMIT: 1,
        CONF_SET_PUMP_HZ: 0.1,
    }.get(key, 1)

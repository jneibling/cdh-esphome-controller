import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID
from . import CDHController, HeaterSwitch, ControlModeSwitch, CONF_CDH_CONTROLLER_ID

CONF_HEATER_SWITCH = "heater_switch"
CONF_CONTROL_MODE_SWITCH = "control_mode_switch"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CDH_CONTROLLER_ID): cv.use_id(CDHController),
        cv.Optional(CONF_HEATER_SWITCH): switch.switch_schema(
            HeaterSwitch,
            icon="mdi:fire",
        ),
        cv.Optional(CONF_CONTROL_MODE_SWITCH): switch.switch_schema(
            ControlModeSwitch,
            icon="mdi:remote",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CDH_CONTROLLER_ID])

    if CONF_HEATER_SWITCH in config:
        sw = await switch.new_switch(config[CONF_HEATER_SWITCH])
        await cg.register_component(sw, config[CONF_HEATER_SWITCH])
        cg.add(sw.set_parent(parent))
        cg.add(parent.set_heater_switch(sw))

    if CONF_CONTROL_MODE_SWITCH in config:
        sw = await switch.new_switch(config[CONF_CONTROL_MODE_SWITCH])
        await cg.register_component(sw, config[CONF_CONTROL_MODE_SWITCH])
        cg.add(sw.set_parent(parent))
        cg.add(parent.set_control_mode_switch(sw))

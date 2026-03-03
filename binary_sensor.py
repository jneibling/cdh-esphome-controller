import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from . import CDHController, CONF_CDH_CONTROLLER_ID

CONF_ON_OFF_STATE = "on_off_state"
CONF_CONTROL_ACTIVE = "control_active"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CDH_CONTROLLER_ID): cv.use_id(CDHController),
        cv.Optional(CONF_ON_OFF_STATE): binary_sensor.binary_sensor_schema(
            icon="mdi:fire",
        ),
        cv.Optional(CONF_CONTROL_ACTIVE): binary_sensor.binary_sensor_schema(
            icon="mdi:remote",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CDH_CONTROLLER_ID])

    if CONF_ON_OFF_STATE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_ON_OFF_STATE])
        cg.add(parent.set_on_off_sensor(sens))

    if CONF_CONTROL_ACTIVE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONTROL_ACTIVE])
        cg.add(parent.set_control_active_sensor(sens))

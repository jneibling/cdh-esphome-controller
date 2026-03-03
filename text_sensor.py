import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import CDHController, CONF_CDH_CONTROLLER_ID

CONF_RUN_STATE = "run_state"
CONF_ERROR_CODE = "error_code"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CDH_CONTROLLER_ID): cv.use_id(CDHController),
        cv.Optional(CONF_RUN_STATE): text_sensor.text_sensor_schema(
            icon="mdi:state-machine",
        ),
        cv.Optional(CONF_ERROR_CODE): text_sensor.text_sensor_schema(
            icon="mdi:alert-circle",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CDH_CONTROLLER_ID])

    if CONF_RUN_STATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_RUN_STATE])
        cg.add(parent.set_run_state_sensor(sens))

    if CONF_ERROR_CODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_ERROR_CODE])
        cg.add(parent.set_error_code_sensor(sens))

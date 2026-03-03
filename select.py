import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID
from . import CDHController, CDHSelect, CONF_CDH_CONTROLLER_ID

CONF_OPERATING_MODE = "operating_mode"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CDH_CONTROLLER_ID): cv.use_id(CDHController),
        cv.Optional(CONF_OPERATING_MODE): select.select_schema(
            CDHSelect,
            icon="mdi:thermostat",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CDH_CONTROLLER_ID])

    if CONF_OPERATING_MODE in config:
        sel = await select.new_select(
            config[CONF_OPERATING_MODE],
            options=["Thermostat", "Fixed Hz"],
        )
        await cg.register_component(sel, config[CONF_OPERATING_MODE])
        cg.add(sel.set_parent(parent))
        cg.add(parent.set_operating_mode_select(sel))

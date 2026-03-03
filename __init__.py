import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@jeremiah"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor", "switch", "number", "select"]
MULTI_CONF = False

CONF_CDH_CONTROLLER_ID = "cdh_controller_id"
CONF_RELAY_PIN = "relay_pin"

cdh_controller_ns = cg.esphome_ns.namespace("cdh_controller")
CDHController = cdh_controller_ns.class_(
    "CDHController", cg.PollingComponent, uart.UARTDevice
)

# Export classes for other platform files
HeaterSwitch = cdh_controller_ns.class_("HeaterSwitch", cg.Component)
ControlModeSwitch = cdh_controller_ns.class_("ControlModeSwitch", cg.Component)
CDHNumber = cdh_controller_ns.class_("CDHNumber", cg.Component)
CDHSelect = cdh_controller_ns.class_("CDHSelect", cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CDHController),
            cv.Optional(CONF_RELAY_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RELAY_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RELAY_PIN])
        cg.add(var.set_relay_pin(pin))

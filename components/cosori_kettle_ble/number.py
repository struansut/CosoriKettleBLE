"""Number platform for Cosori Kettle BLE."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID
from . import COSORI_KETTLE_BLE_COMPONENT_SCHEMA, CONF_COSORI_KETTLE_BLE_ID, cosori_kettle_ble_ns

CONF_TARGET_SETPOINT = "target_setpoint"

CosoriKettleNumber = cosori_kettle_ble_ns.class_("CosoriKettleNumber", number.Number, cg.Component)

CONFIG_SCHEMA = COSORI_KETTLE_BLE_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_TARGET_SETPOINT): number.number_schema(
            CosoriKettleNumber,
            unit_of_measurement="°F",
            min_value=104,
            max_value=212,
            step=1,
        ),
    }
)


async def to_code(config):
    """Code generation for number platform."""
    parent = await cg.get_variable(config[CONF_COSORI_KETTLE_BLE_ID])

    if CONF_TARGET_SETPOINT in config:
        conf = config[CONF_TARGET_SETPOINT]
        num = await number.new_number(conf, min_value=104, max_value=212, step=1)
        await cg.register_parented(num, config[CONF_COSORI_KETTLE_BLE_ID])
        cg.add(parent.set_target_setpoint_number(num))

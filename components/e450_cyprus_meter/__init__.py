import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_VOLT,
    UNIT_WATT,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_FRAME_TIMEOUT = "frame_timeout"
CONF_VALIDATE_CRC = "validate_crc"

UNIT_VAR_HOURS = "varh"
UNIT_WATT_HOURS = "Wh"

NUMERIC_SENSORS = {
    "active_power_import_l1": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "active_power_import_l2": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "active_power_import_l3": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "active_power_export_l1": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "active_power_export_l2": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "active_power_export_l3": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "active_power_import_total": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "active_power_export_total": (UNIT_WATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, 0),
    "reactive_power_import": (
        "var",
        None,
        STATE_CLASS_MEASUREMENT,
        0,
    ),
    "reactive_power_export": (
        "var",
        None,
        STATE_CLASS_MEASUREMENT,
        0,
    ),
    "power_factor": ("", DEVICE_CLASS_POWER_FACTOR, STATE_CLASS_MEASUREMENT, 3),
    "current_l1": (UNIT_AMPERE, DEVICE_CLASS_CURRENT, STATE_CLASS_MEASUREMENT, 2),
    "current_l2": (UNIT_AMPERE, DEVICE_CLASS_CURRENT, STATE_CLASS_MEASUREMENT, 2),
    "current_l3": (UNIT_AMPERE, DEVICE_CLASS_CURRENT, STATE_CLASS_MEASUREMENT, 2),
    "voltage_l1": (UNIT_VOLT, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT, 0),
    "voltage_l2": (UNIT_VOLT, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT, 0),
    "voltage_l3": (UNIT_VOLT, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT, 0),
    "frequency": (UNIT_HERTZ, DEVICE_CLASS_FREQUENCY, STATE_CLASS_MEASUREMENT, 0),
    "disconnect_control_state": ("", None, STATE_CLASS_MEASUREMENT, 0),
    "active_energy_import_t1": (
        UNIT_WATT_HOURS,
        DEVICE_CLASS_ENERGY,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "active_energy_import_t2": (
        UNIT_WATT_HOURS,
        DEVICE_CLASS_ENERGY,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "active_energy_export_t1": (
        UNIT_WATT_HOURS,
        DEVICE_CLASS_ENERGY,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "active_energy_export_t2": (
        UNIT_WATT_HOURS,
        DEVICE_CLASS_ENERGY,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "active_energy_import_total": (
        UNIT_WATT_HOURS,
        DEVICE_CLASS_ENERGY,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "active_energy_export_total": (
        UNIT_WATT_HOURS,
        DEVICE_CLASS_ENERGY,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "reactive_energy_import": (
        UNIT_VAR_HOURS,
        None,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "reactive_energy_export": (
        UNIT_VAR_HOURS,
        None,
        STATE_CLASS_TOTAL_INCREASING,
        0,
    ),
    "disconnect_limiter_state": ("", None, STATE_CLASS_MEASUREMENT, 0),
}

TEXT_SENSORS = (
    "push_timestamp",
    "meter_clock",
    "energy_push_timestamp",
    "meter_logical_device_name",
    "meter_serial_number",
    "active_tariff",
)

e450_cyprus_meter_ns = cg.esphome_ns.namespace("e450_cyprus_meter")
E450CyprusMeterComponent = e450_cyprus_meter_ns.class_(
    "E450CyprusMeterComponent", cg.Component, uart.UARTDevice
)


def _sensor_schema(unit, device_class, state_class, accuracy_decimals):
    kwargs = {
        "unit_of_measurement": unit,
        "state_class": state_class,
        "accuracy_decimals": accuracy_decimals,
    }
    if device_class is not None:
        kwargs["device_class"] = device_class
    return sensor.sensor_schema(**kwargs)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(E450CyprusMeterComponent),
            cv.Optional(CONF_FRAME_TIMEOUT, default="1500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_VALIDATE_CRC, default=True): cv.boolean,
            **{
                cv.Optional(key): _sensor_schema(*value)
                for key, value in NUMERIC_SENSORS.items()
            },
            **{
                cv.Optional(key): text_sensor.text_sensor_schema()
                for key in TEXT_SENSORS
            },
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_frame_timeout(config[CONF_FRAME_TIMEOUT]))
    cg.add(var.set_validate_crc(config[CONF_VALIDATE_CRC]))

    for key in NUMERIC_SENSORS:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))

    for key in TEXT_SENSORS:
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_text_sensor")(sens))

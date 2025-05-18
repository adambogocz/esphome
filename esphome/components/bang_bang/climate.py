from esphome import automation
import esphome.codegen as cg
from esphome.components import climate, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_AWAY_CONFIG,
    CONF_COOL_ACTION,
    CONF_DEFAULT_TARGET_TEMPERATURE_HIGH,
    CONF_DEFAULT_TARGET_TEMPERATURE_LOW,
    CONF_HEAT_ACTION,
    CONF_HUMIDITY_SENSOR,
    CONF_ID,
    CONF_IDLE_ACTION,
    CONF_NAME,
    CONF_PRESET,
    CONF_SENSOR,
)

CONF_PRESET_CHANGE = "preset_change"
CONF_DEFAULT_PRESET = "default_preset"
CONF_ON_BOOT_RESTORE_FROM = "on_boot_restore_from"

bang_bang_ns = cg.esphome_ns.namespace("bang_bang")
BangBangClimate = bang_bang_ns.class_("BangBangClimate", climate.Climate, cg.Component)
BangBangClimateTargetTempConfig = bang_bang_ns.struct("BangBangClimateTargetTempConfig")

PRESET_CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BangBangClimateTargetTempConfig),
        cv.Required(CONF_NAME): cv.string_strict,
        cv.Optional(CONF_DEFAULT_TARGET_TEMPERATURE_HIGH): cv.temperature,
        cv.Optional(CONF_DEFAULT_TARGET_TEMPERATURE_LOW): cv.temperature,
    }
)

CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(BangBangClimate),
            cv.Required(CONF_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
            cv.Required(CONF_DEFAULT_TARGET_TEMPERATURE_LOW): cv.temperature,
            cv.Required(CONF_DEFAULT_TARGET_TEMPERATURE_HIGH): cv.temperature,
            cv.Required(CONF_IDLE_ACTION): automation.validate_automation(single=True),
            cv.Optional(CONF_COOL_ACTION): automation.validate_automation(single=True),
            cv.Optional(CONF_HEAT_ACTION): automation.validate_automation(single=True),
            cv.Optional(CONF_AWAY_CONFIG): cv.Schema(
                {
                    cv.Required(CONF_DEFAULT_TARGET_TEMPERATURE_LOW): cv.temperature,
                    cv.Required(CONF_DEFAULT_TARGET_TEMPERATURE_HIGH): cv.temperature,
                }
            ),
            cv.Optional(CONF_PRESET): cv.ensure_list(PRESET_CONFIG_SCHEMA),
            cv.Optional(CONF_PRESET_CHANGE): automation.validate_automation(
                single=True
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_COOL_ACTION, CONF_HEAT_ACTION),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    sens = await cg.get_variable(config[CONF_SENSOR])
    cg.add(var.set_sensor(sens))

    if CONF_HUMIDITY_SENSOR in config:
        sens = await cg.get_variable(config[CONF_HUMIDITY_SENSOR])
        cg.add(var.set_humidity_sensor(sens))

    normal_config = BangBangClimateTargetTempConfig(
        config[CONF_DEFAULT_TARGET_TEMPERATURE_LOW],
        config[CONF_DEFAULT_TARGET_TEMPERATURE_HIGH],
    )
    cg.add(var.set_normal_config(normal_config))

    await automation.build_automation(
        var.get_idle_trigger(), [], config[CONF_IDLE_ACTION]
    )

    if cool_action_config := config.get(CONF_COOL_ACTION):
        await automation.build_automation(
            var.get_cool_trigger(), [], cool_action_config
        )
        cg.add(var.set_supports_cool(True))
    if heat_action_config := config.get(CONF_HEAT_ACTION):
        await automation.build_automation(
            var.get_heat_trigger(), [], heat_action_config
        )
        cg.add(var.set_supports_heat(True))

    if away := config.get(CONF_AWAY_CONFIG):
        away_config = BangBangClimateTargetTempConfig(
            away[CONF_DEFAULT_TARGET_TEMPERATURE_LOW],
            away[CONF_DEFAULT_TARGET_TEMPERATURE_HIGH],
        )
        cg.add(var.set_away_config(away_config))

    if CONF_PRESET in config:
        for preset_config in config[CONF_PRESET]:
            name = preset_config[CONF_NAME]
            standard_preset = None
            if name.upper() in climate.CLIMATE_PRESETS:
                standard_preset = climate.CLIMATE_PRESETS[name.upper()]

            preset_target_config = BangBangClimateTargetTempConfig(
                preset_config[CONF_DEFAULT_TARGET_TEMPERATURE_LOW],
                preset_config[CONF_DEFAULT_TARGET_TEMPERATURE_HIGH],
            )

            preset_target_variable = cg.new_variable(
                preset_config[CONF_ID], preset_target_config
            )

            if standard_preset is not None:
                cg.add(var.set_preset_config(standard_preset, preset_target_variable))
            else:
                cg.add(var.set_custom_preset_config(name, preset_target_variable))

    if CONF_DEFAULT_PRESET in config:
        default_preset_name = config[CONF_DEFAULT_PRESET]

        # if the name is a built in preset use the appropriate naming format
        if default_preset_name.upper() in climate.CLIMATE_PRESETS:
            climate_preset = climate.CLIMATE_PRESETS[default_preset_name.upper()]
            cg.add(var.set_default_preset(climate_preset))
        else:
            cg.add(var.set_default_preset(default_preset_name))

    if CONF_ON_BOOT_RESTORE_FROM in config:
        cg.add(var.set_on_boot_restore_from(config[CONF_ON_BOOT_RESTORE_FROM]))

    if CONF_PRESET_CHANGE in config:
        await automation.build_automation(
            var.get_preset_change_trigger(), [], config[CONF_PRESET_CHANGE]
        )

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_UART_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = []

tdma485_ns = cg.esphome_ns.namespace("tdma485")

TDMA485 = tdma485_ns.class_("TDMA485", cg.Component, uart.UARTDevice)

CONF_ROLE = "role"
CONF_NODE_ID = "node_id"
CONF_NUM_NODES = "num_nodes"
CONF_GUARD_CHARS = "guard_chars"
CONF_BAUD = "baud"
CONF_DIRECTION_PIN = "direction_pin"
CONF_SYNC_BYTE_A = "sync_byte_a"
CONF_SYNC_BYTE_B = "sync_byte_b"
CONF_GRANT_TIMEOUT_MS = "grant_timeout_ms"
CONF_ACK_TIMEOUT_MS = "ack_timeout_ms"
CONF_ON_EVENT = "on_event"

ROLES = ["monitor", "node"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TDMA485),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_ROLE): cv.one_of(*ROLES, lower=True),
        cv.Optional(CONF_NODE_ID): cv.int_range(min=0, max=31),
        cv.Optional(CONF_NUM_NODES, default=10): cv.int_range(min=1, max=32),
        cv.Optional(CONF_BAUD, default=9600): cv.int_range(min=1200, max=1000000),
        cv.Optional(CONF_GUARD_CHARS, default=1.0): cv.float_range(min=0.5, max=10.0),
        cv.Optional(CONF_DIRECTION_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_SYNC_BYTE_A, default=0xF0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_SYNC_BYTE_B, default=0x0F): cv.int_range(min=0, max=255),
        cv.Optional(CONF_GRANT_TIMEOUT_MS, default=100): cv.int_range(min=5, max=1000),
        cv.Optional(CONF_ACK_TIMEOUT_MS, default=50): cv.int_range(min=5, max=1000),
        cv.Optional(CONF_ON_EVENT): automation.validate_automation(),
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Request UART to wake the main loop when data arrives for low-latency processing
    uart.request_wake_loop_on_rx()

    role_str = config[CONF_ROLE]
    is_monitor = role_str == "monitor"
    cg.add(var.set_is_monitor(is_monitor))

    if CONF_NODE_ID in config:
        cg.add(var.set_node_id(config[CONF_NODE_ID]))
    cg.add(var.set_num_nodes(config[CONF_NUM_NODES]))
    cg.add(var.set_baud(config[CONF_BAUD]))
    cg.add(var.set_guard_chars(config[CONF_GUARD_CHARS]))
    cg.add(var.set_sync_bytes(config[CONF_SYNC_BYTE_A], config[CONF_SYNC_BYTE_B]))
    cg.add(
        var.set_timeouts_ms(config[CONF_GRANT_TIMEOUT_MS], config[CONF_ACK_TIMEOUT_MS])
    )

    if CONF_DIRECTION_PIN in config:
        pin = await cg.gpio_output_pin_expression(config[CONF_DIRECTION_PIN])
        cg.add(var.set_direction_pin(pin))

    # Build on_event automations (monitor role)
    for conf in config.get(CONF_ON_EVENT, []):
        # args: node_id(uint8), event_type(uint8), payload(std::vector<uint8_t>)
        args = [
            (cg.uint8, "node_id"),
            (cg.uint8, "event_type"),
            (cg.std_vector(cg.uint8), "payload"),
        ]
        await automation.build_automation(var.get_on_event_trigger(), args, conf)

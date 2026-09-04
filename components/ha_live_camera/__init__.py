"""ha_live_camera -- live MJPEG viewer for ESP32-P4, decoded in hardware.

Streams from Frigate's built-in MJPEG endpoint, which serves frames straight
off the already-decoded detect stream -- no transcoding anywhere. The user
still configures plain Home Assistant `entity_id`s: the component reads each
entity's `camera_name` attribute over the API connection ESPHome already
holds and builds the Frigate URL from it. Decoded frames are presented as an
`image::Image`, so an ordinary LVGL `image:` widget can use it as `src:`.
"""

import esphome.codegen as cg
from esphome.components import image
from esphome.components.image import CONF_OPAQUE, add_metadata
import esphome.config_validation as cv
from esphome import automation
from esphome.const import (
    CONF_ENTITY_ID,
    CONF_ID,
    CONF_NAME,
    CONF_TRIGGER_ID,
)

CODEOWNERS = ["@derek"]
DEPENDENCIES = ["api", "esp32"]
AUTO_LOAD = ["image"]

CONF_FRIGATE_URL = "frigate_url"
CONF_FPS = "fps"
CONF_STREAM_HEIGHT = "stream_height"
CONF_CAMERAS = "cameras"
CONF_MAX_FRAME_BYTES = "max_frame_bytes"
CONF_MAX_HEIGHT = "max_height"
CONF_MAX_WIDTH = "max_width"
CONF_ON_FRAME = "on_frame"
CONF_ON_STATUS = "on_status"
CONF_TASK_PRIORITY = "task_priority"
CONF_RGB_ORDER = "rgb_order"
CONF_USERNAME = "username"
CONF_PASSWORD = "password"
CONF_VERIFY_SSL = "verify_ssl"

ha_live_camera_ns = cg.esphome_ns.namespace("ha_live_camera")
HaLiveCamera = ha_live_camera_ns.class_("HaLiveCamera", cg.Component, image.Image_)

ShowCameraAction = ha_live_camera_ns.class_("ShowCameraAction", automation.Action)
StopCameraAction = ha_live_camera_ns.class_("StopCameraAction", automation.Action)


def _validate_p4(config):
    from esphome.components.esp32 import get_esp32_variant
    from esphome.components.esp32.const import VARIANT_ESP32P4

    if get_esp32_variant() != VARIANT_ESP32P4:
        raise cv.Invalid(
            "ha_live_camera needs the ESP32-P4's hardware JPEG decoder; "
            "it cannot run on other ESP32 variants."
        )
    return config


CAMERA_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ENTITY_ID): cv.entity_id,
        cv.Optional(CONF_NAME): cv.string,
    }
)


def _multiple_of_16(value):
    """The JPEG engine pads decoded output to 16-pixel MCU boundaries.

    Padding on the height is harmless -- the extra rows simply are not shown --
    but padding on the width changes the row stride, and LVGL would shear the
    image. So refuse a width that is not already a multiple of 16.
    """
    value = cv.int_range(min=16, max=1920)(value)
    if value % 16 != 0:
        raise cv.Invalid(
            f"{CONF_MAX_WIDTH} must be a multiple of 16 (the JPEG decoder's MCU "
            f"size); {value} is not. Try {((value + 15) // 16) * 16}."
        )
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HaLiveCamera),
            cv.Required(CONF_FRIGATE_URL): cv.url,
            cv.Optional(CONF_FPS, default=15): cv.int_range(min=1, max=30),
            cv.Optional(CONF_STREAM_HEIGHT, default=270): cv.int_range(min=64, max=1080),
            cv.Required(CONF_CAMERAS): cv.All(
                cv.ensure_list(CAMERA_SCHEMA), cv.Length(min=1)
            ),
            cv.Optional(CONF_MAX_WIDTH, default=480): _multiple_of_16,
            cv.Optional(CONF_MAX_HEIGHT, default=272): cv.int_range(min=16, max=1080),
            cv.Optional(CONF_MAX_FRAME_BYTES, default="128kB"): cv.validate_bytes,
            cv.Optional(CONF_TASK_PRIORITY, default=5): cv.int_range(min=1, max=20),
            cv.Optional(CONF_RGB_ORDER, default="bgr"): cv.one_of("rgb", "bgr", lower=True),
            # Frigate's authenticated port (8971). Omit both to talk to the
            # unauthenticated port 5000, which hands admin-equivalent access to
            # anything that can reach it.
            cv.Optional(CONF_USERNAME): cv.string_strict,
            cv.Optional(CONF_PASSWORD): cv.string_strict,
            # Only consulted for https:// URLs. Frigate's own certificate is
            # self-signed and cannot be verified by anything, so the default is
            # False -- encrypted, but not authenticated. Set True only behind a
            # reverse proxy with a real certificate.
            cv.Optional(CONF_VERIFY_SSL, default=False): cv.boolean,
            cv.Optional(CONF_ON_FRAME): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        automation.Trigger.template()
                    )
                }
            ),
            cv.Optional(CONF_ON_STATUS): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        automation.Trigger.template(cg.std_string)
                    )
                }
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_p4,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_frigate_url(config[CONF_FRIGATE_URL]))
    cg.add(var.set_stream_params(config[CONF_FPS], config[CONF_STREAM_HEIGHT]))
    cg.add(var.set_max_size(config[CONF_MAX_WIDTH], config[CONF_MAX_HEIGHT]))
    cg.add(var.set_max_frame_bytes(config[CONF_MAX_FRAME_BYTES]))
    cg.add(var.set_task_priority(config[CONF_TASK_PRIORITY]))
    cg.add(var.set_rgb_order_bgr(config[CONF_RGB_ORDER] == "bgr"))
    cg.add(var.set_verify_ssl(config[CONF_VERIFY_SSL]))
    if CONF_USERNAME in config:
        cg.add(var.set_credentials(config[CONF_USERNAME], config.get(CONF_PASSWORD, "")))

    # LVGL inspects image metadata at codegen time to decide which colour
    # formats to compile in. Register ours as an opaque RGB565 image of the
    # maximum configured size; the runtime dimensions follow the stream.
    add_metadata(
        config[CONF_ID],
        config[CONF_MAX_WIDTH],
        config[CONF_MAX_HEIGHT],
        "RGB565",
        CONF_OPAQUE,
    )

    for cam in config[CONF_CAMERAS]:
        name = cam.get(CONF_NAME, cam[CONF_ENTITY_ID])
        cg.add(var.add_camera(cam[CONF_ENTITY_ID], name))

    # Needed for subscribe_home_assistant_state() to be compiled into the API.
    cg.add_define("USE_API_HOMEASSISTANT_STATES")

    # ESPHome excludes most ESP-IDF components from the build by default to
    # keep firmware small; esp_http_client is one of them. Opt back in, or the
    # streaming client will not link. esp_driver_jpeg is built by default on
    # P4 targets, but ask for it explicitly so this cannot regress.
    from esphome.components.esp32 import include_builtin_idf_component

    include_builtin_idf_component("esp_http_client")
    include_builtin_idf_component("esp_driver_jpeg")

    for conf in config.get(CONF_ON_FRAME, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.add_frame_trigger(trigger))
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_STATUS, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.add_status_trigger(trigger))
        await automation.build_automation(trigger, [(cg.std_string, "x")], conf)


SHOW_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(HaLiveCamera),
        cv.Required("index"): cv.templatable(cv.int_range(min=0, max=63)),
    }
)

STOP_SCHEMA = automation.maybe_simple_id(
    {cv.GenerateID(): cv.use_id(HaLiveCamera)}
)


@automation.register_action(
    "ha_live_camera.show", ShowCameraAction, SHOW_SCHEMA, synchronous=True
)
async def show_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config["index"], args, cg.int_)
    cg.add(var.set_index(template_))
    return var


@automation.register_action(
    "ha_live_camera.stop", StopCameraAction, STOP_SCHEMA, synchronous=True
)
async def stop_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
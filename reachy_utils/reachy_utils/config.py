import os
from functools import partial

import yaml

config_file = os.path.expanduser("~/.reachy.yaml")


FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI = (
    "full_kit",
    "starter_kit_right",
    "starter_kit_left",
    "headless",
    "mini",
)
STARTER_KIT_RIGHT_NO_HEAD = "starter_kit_right_no_head"

REACHY_CONFIG_MODEL = "model"
REACHY_CONFIG_NECK = "neck_config"
REACHY_CONFIG_RIGHT_SHOULDER = "right_shoulder_config"
REACHY_CONFIG_RIGHT_ELBOW = "right_elbow_config"
REACHY_CONFIG_RIGHT_WRIST = "right_wrist_config"

REACHY_CONFIG_LEFT_SHOULDER = "left_shoulder_config"
REACHY_CONFIG_LEFT_ELBOW = "left_elbow_config"
REACHY_CONFIG_LEFT_WRIST = "left_wrist_config"

REACHY_CONFIG_MOBILE_BASE = "mobile_base"


class ReachyConfig:
    def __init__(self, config_file_path="~/.reachy_config/.reachy.yaml"):
        self.config_file = os.path.expanduser(config_file_path)

        with open(self.config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

            # Robot model (Only full kit for now, TODO)
            if config[REACHY_CONFIG_MODEL] in [
                FULL_KIT
            ]:  # , STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI, STARTER_KIT_RIGHT_NO_HEAD]:
                self.model = config[REACHY_CONFIG_MODEL]
            else:
                raise ValueError(
                    'Bad robot model "{}". Expected values are {}'.format(
                        config[REACHY_CONFIG_MODEL],
                        [
                            FULL_KIT,
                            STARTER_KIT_RIGHT,
                            STARTER_KIT_LEFT,
                            HEADLESS,
                            MINI,
                            STARTER_KIT_RIGHT_NO_HEAD,
                        ],
                    )
                )

            # TODO Multiple config

            # Mobile base
            # orbita zero
            try:
                self.mobile_base_config = config[REACHY_CONFIG_MOBILE_BASE]

            except KeyError as e:
                raise KeyError("mobile_base config key not found :: {}".format(e))

            # orbita zero
            try:
                self.neck_config = config[REACHY_CONFIG_NECK]

            except KeyError as e:
                raise KeyError("orbita3d neck config key not found :: {}".format(e))

            # right shoulder
            try:
                self.right_shoulder_config = config[REACHY_CONFIG_RIGHT_SHOULDER]

            except KeyError as e:
                raise KeyError("orbita2d right shoulder key not found :: {}".format(e))

            # right elbow
            try:
                self.right_elbow_config = config[REACHY_CONFIG_RIGHT_ELBOW]
            except KeyError as e:
                raise KeyError("orbita2d right elbow key not found :: {}".format(e))

            # right wrist
            try:
                self.right_wrist_config = config[REACHY_CONFIG_RIGHT_WRIST]
            except KeyError as e:
                raise KeyError("orbita3d right wrist key not found :: {}".format(e))

            # left shoulder
            try:
                self.left_shoulder_config = config[REACHY_CONFIG_LEFT_SHOULDER]

            except KeyError as e:
                raise KeyError("orbita2d left shoulder key not found :: {}".format(e))

            # left elbow
            try:
                self.left_elbow_config = config[REACHY_CONFIG_LEFT_ELBOW]
            except KeyError as e:
                raise KeyError("orbita2d left elbow key not found :: {}".format(e))

            # left wrist
            try:
                self.left_wrist_config = config[REACHY_CONFIG_LEFT_WRIST]
            except KeyError as e:
                raise KeyError("orbita3d left wrist key not found :: {}".format(e))

    def __str__(self):
        return (
            "robot_model".ljust(25, " ")
            + "{}\n".format(self.model)
            + "neck_config".ljust(25, " ")
            + "{}\n".format(self.neck_config)
            + "right_shoulder_config".ljust(25, " ")
            + "{}\n".format(self.right_shoulder_config)
            + "right_elbow_config".ljust(25, " ")
            + "{}\n".format(self.right_elbow_config)
            + "right_wrist_config".ljust(25, " ")
            + "{}\n".format(self.right_wrist_config)
            + "left_shoulder_config".ljust(25, " ")
            + "{}\n".format(self.left_shoulder_config)
            + "left_elbow_config".ljust(25, " ")
            + "{}\n".format(self.left_elbow_config)
            + "left_wrist_config".ljust(25, " ")
            + "{}\n".format(self.left_wrist_config)
        )


def get_reachy_config():
    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        return config


def _get_config_parameter(parameter: str, part=None):
    config = get_reachy_config()
    try:
        return config[parameter]
    except KeyError:
        print(f"{parameter} not found in {config_file}")
        return -1


get_reachy_generation = partial(_get_config_parameter, "generation")
get_reachy_model = partial(_get_config_parameter, "model")
get_reachy_serial_number = partial(_get_config_parameter, "serial_number")
get_zuuu_version = partial(_get_config_parameter, "zuuu_version")
get_neck_orbita_zero = partial(_get_config_parameter, "neck_orbita_zero")
get_camera_parameters = partial(_get_config_parameter, "camera_parameters")
get_fan_trigger_temperature = partial(_get_config_parameter, "fan_trigger_temperature")

import os
from functools import partial

import yaml

# config_file = os.path.expanduser("~/.reachy.yaml")


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

REACHY_CONFIG_GRIPPERS = "grippers_config"
REACHY_CONFIG_ANTENNA = "antenna_config"


# REACHY_CONFIG_LEFT_ARM = "left_arm_config"
# REACHY_CONFIG_RIGHT_ARM = "right_arm_config"

REACHY_CONFIG_SERIAL_NUMBER = "serial_number"
REACHY_CONFIG_MOBILE_BASE = "mobile_base"
ETHERCAT = "ethercat"
BETA = "beta"
DVT = "dvt"
PVT = "pvt"


class ReachyConfig:
    def __init__(self, config_file_path="~/.reachy_config/reachy.yaml"):
        self.config_file = os.path.expanduser(config_file_path)

        with open(self.config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

            if ETHERCAT in config:
                if config[ETHERCAT] in [True, False]:
                    self.ethercat = bool(config[ETHERCAT])
                else:
                    raise ValueError('Bad ethercat value "{}". Expected values are {}'.format(config[ETHERCAT], [True, False]))
            else:
                self.ethercat = False

            if REACHY_CONFIG_SERIAL_NUMBER in config:
                if DVT in config[REACHY_CONFIG_SERIAL_NUMBER]:
                    self.dvt = True
                    self.beta = False
                    self.pvt = False
                elif BETA in config[REACHY_CONFIG_SERIAL_NUMBER]:
                    self.beta = True
                    self.dvt = False
                    self.pvt = False
                elif PVT in config[REACHY_CONFIG_SERIAL_NUMBER]:
                    self.beta = False
                    self.dvt = False
                    self.pvt = True
                else:
                    raise ValueError(
                        'Bad serial number "{}". Expected values are {}'.format(
                            config[REACHY_CONFIG_SERIAL_NUMBER], [DVT, BETA, PVT]
                        )
                    )

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

            # # Right arm
            # try:
            #     self.right_arm_config = config[REACHY_CONFIG_RIGHT_ARM]
            # except KeyError as e:
            #     raise KeyError("right_arm config key not found :: {}".format(e))

            # # Left arm
            # try:
            #     self.left_arm_config = config[REACHY_CONFIG_LEFT_ARM]
            # except KeyError as e:
            #     raise KeyError("left_arm config key not found :: {}".format(e))

            try:
                self.right_shoulder_config = config[REACHY_CONFIG_RIGHT_SHOULDER]
            except KeyError as e:
                raise KeyError("right_shoulder_config key not found :: {}".format(e))
            try:
                self.right_elbow_config = config[REACHY_CONFIG_RIGHT_ELBOW]
            except KeyError as e:
                raise KeyError("right_elbow_config key not found :: {}".format(e))
            try:
                self.right_wrist_config = config[REACHY_CONFIG_RIGHT_WRIST]
            except KeyError as e:
                raise KeyError("right_wrist_config key not found :: {}".format(e))
            try:
                self.left_shoulder_config = config[REACHY_CONFIG_LEFT_SHOULDER]
            except KeyError as e:
                raise KeyError("left_shoulder_config key not found :: {}".format(e))
            try:
                self.left_elbow_config = config[REACHY_CONFIG_LEFT_ELBOW]
            except KeyError as e:
                raise KeyError("left_elbow_config key not found :: {}".format(e))
            try:
                self.left_wrist_config = config[REACHY_CONFIG_LEFT_WRIST]
            except KeyError as e:
                raise KeyError("left_wrist_config key not found :: {}".format(e))
            try:
                self.grippers_config = config[REACHY_CONFIG_GRIPPERS]
            except KeyError as e:
                raise KeyError("grippers_config key not found :: {}".format(e))
            try:
                self.antenna_config = config[REACHY_CONFIG_ANTENNA]
            except KeyError as e:
                raise KeyError("antenna_config key not found :: {}".format(e))

    def __str__(self):
        return (
            "robot_model".ljust(25, " ")
            + "{}\n".format(self.model)
            + "neck_config".ljust(25, " ")
            + "{}\n".format(self.neck_config)
            # + "right_arm_config".ljust(25, " ")
            # + "{}\n".format(self.right_arm_config)
            # + "left_arm_config".ljust(25, " ")
            # + "{}\n".format(self.left_arm_config)
            + "ethercat".ljust(25, " ")
            + "{}\n".format(self.ethercat)
        )


def get_reachy_config():
    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        return config


def log_config(xacro_config):
    log = ""
    for line in xacro_config:
        if line != " ":
            key, value = line.split(":=")
            key = key.strip()
            log += f"{key.ljust(25, ' ')}{value}\n"

    return log


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

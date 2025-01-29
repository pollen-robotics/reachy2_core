import logging
import os
from functools import partial
from pathlib import Path

from reachy_config.parsing import dump_yaml, load_yaml
from reachy_config.utils import get_logger, merge_config
from reachy_config.validate import validate

## help me do a better, commented, professional version of this file
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

REACHY_CONFIG_PATH = os.path.expanduser("~/.reachy_config")


class ReachyConfig:
    def __init__(self, custom_config_file_path="~/.reachy_config_override", no_print=False):
        self.custom_config_dir = os.path.expanduser(custom_config_file_path)
        # in package/default
        self.default_config_dir = os.path.dirname(os.path.realpath(__file__)) + "/../config/default"
        self.schema_config_dir = os.path.dirname(os.path.realpath(__file__)) + "/../config/schema"
        self.config = {}
        # create a logger object
        self.logger = get_logger()
        # print("##########################\n#  Reachy2 Configuration #\n##########################")

        # change logger format, currently header looks like this "DEBUG:reachy_config.reachy_config:"" how do i edit header

        # ls both dir and print output
        # print("Custom config dir: ", self.custom_config_dir)
        # print("Custom config dir list: ", os.listdir(self.custom_config_dir))
        # print("\n\n")
        # print("Default config dir: ", self.default_config_dir)
        # print("Default config dir list: ", os.listdir(self.default_config_dir))
        # print("\n\n\n")

        # TODO also list config dir in other packages and add a "package" field for everything config entry

        # create .reachy_config if it does not exists
        if not os.path.exists(REACHY_CONFIG_PATH):
            os.makedirs(REACHY_CONFIG_PATH)

        # Load default configuration schema
        for default_config_file in os.listdir(self.default_config_dir):
            default_config_file_name = os.path.splitext(default_config_file)[0]
            if not default_config_file_name in self.config:
                self.config[default_config_file_name] = {}
            self.config[default_config_file_name]["mode"] = "default"
            self.config[default_config_file_name]["path"] = str(
                Path(os.path.join(self.default_config_dir, default_config_file)).resolve()
            )
            # package_path
            self.config[default_config_file_name]["package_path"] = str(
                Path(os.path.join(self.default_config_dir, "..")).resolve()
            )

            self.config[default_config_file_name]["config"] = load_yaml(self.config[default_config_file_name]["path"])

            # Add schema validation if it exists
            schema_file_path = os.path.join(self.schema_config_dir, default_config_file)
            if os.path.exists(schema_file_path):
                self.config[default_config_file_name]["schema"] = schema_file_path

        # Process config customisation
        for custom_config_file in os.listdir(self.custom_config_dir):
            if "~" in custom_config_file or not ".yaml" in custom_config_file:
                continue
            # print("parsing custom config: ", custom_config_file)
            custom_config_file_path = os.path.join(self.custom_config_dir, custom_config_file)

            # self.config[os.path.splitext(custom_config_file)[0]] = load_yaml(custom_config_file_path)
            # print file name
            # print("Custom config file: ", custom_config_file)
            # dump_yaml("test.yaml", self.config[os.path.splitext(custom_config_file)[0]])

            # print("Loading default file: ", custom_config_file)
            # default_config_file_path = os.path.join(self.default_config_dir, custom_config_file)
            # if default file does not exists, we should create it
            custom_config_file_name = os.path.splitext(custom_config_file)[0]
            custom_config = load_yaml(custom_config_file_path)

            # TODO in case custom override is an add, we should load it and link it without creating file
            if not custom_config_file_name in self.config:
                if not no_print:
                    print("\033[93m" + f"[{custom_config_file}] custom configuration added" + "\033[0m")
                self.config[custom_config_file_name] = {}
                self.config[custom_config_file_name]["mode"] = "override"
                self.config[custom_config_file_name]["config"] = custom_config
                self.config[custom_config_file_name]["path"] = custom_config_file_path
                continue

            # load both files and compare dict
            # default_config = load_yaml(default_config_file_path)
            # dump_yaml("test.yaml", custom_config)
            # dump_yaml("test.yaml", default_config)
            # print("Are files the same :", custom_config == default_config)

            # print("\n\n")
            # default_config = self.config[custom_config_file_name]["config"]

            # compare custom content with default one stored at same file adress -> self.config[custom_config_file_name]["config"]
            if custom_config != self.config[custom_config_file_name]["config"]:  # some overriding will be done
                if not no_print:
                    print("\033[93m" + f"[{custom_config_file}] configuration override" + "\033[0m")

                # override default_config in place
                # merged_config_content = default_config  # for clarity as merge is done in place
                # self.config[custom_config_file_name]["config"] same as default content, will now merge in place
                merge_config(self.config[custom_config_file_name]["config"], custom_config)
                override_path = REACHY_CONFIG_PATH + f"/{custom_config_file}"

                # Update config object
                self.config[custom_config_file_name]["mode"] = "override"
                # self.config[custom_config_file_name]["config"] = merged_config_content
                self.config[custom_config_file_name]["path"] = override_path

                # create the merged conf file
                dump_yaml(override_path, self.config[custom_config_file_name]["config"])

            # print("\n\n")
            # print("Merged config: ")
            # dump_yaml("test.yaml", default_config)

        # print("Final config: ", self.config)
        # can instead do a pretty print, indented and all
        # dump_yaml(None, self.config)

        # exit(1)

        # print("Custom config: ", self.config)

        # print("now let's try to dump the custom config")

        # dump_yaml("test.yaml", self.config["right_gripper"])

        # exit(1)

        # Validating configuration files
        validate(self.config, self.logger)
        # exit(1)

        # THE DEBUG PRINT
        # dump_yaml(None, self.config)

        # if ETHERCAT in reachy_config:
        #     if reachy_config[ETHERCAT] in [True, False]:
        #         self.ethercat = bool(reachy_config[ETHERCAT])
        #     else:
        #         raise ValueError(
        #             'Bad ethercat value "{}". Expected values are {}'.format(reachy_config[ETHERCAT], [True, False])
        #         )
        # else:
        #     self.ethercat = False

        # Reachy generation
        reachy_config = self.config["reachy"]["config"]
        if REACHY_CONFIG_SERIAL_NUMBER in reachy_config:
            if DVT in reachy_config[REACHY_CONFIG_SERIAL_NUMBER]:
                self.dvt = True
                self.beta = False
                self.pvt = False
            elif BETA in reachy_config[REACHY_CONFIG_SERIAL_NUMBER]:
                self.beta = True
                self.dvt = False
                self.pvt = False
            elif PVT in reachy_config[REACHY_CONFIG_SERIAL_NUMBER]:
                self.beta = False
                self.dvt = False
                self.pvt = True
            else:
                raise ValueError(
                    'Bad serial number "{}". Expected values are {}'.format(
                        reachy_config[REACHY_CONFIG_SERIAL_NUMBER], [DVT, BETA, PVT]
                    )
                )

        # Robot model (Only full kit for now, TODO)
        self.model = reachy_config[REACHY_CONFIG_MODEL]
        # print(reachy_config[REACHY_CONFIG_MODEL])
        # if reachy_config[REACHY_CONFIG_MODEL] in [
        #     FULL_KIT
        # ]:  # , STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI, STARTER_KIT_RIGHT_NO_HEAD]:
        #     self.model = reachy_config[REACHY_CONFIG_MODEL]
        # else:
        #     raise ValueError(
        #         'Bad robot model "{}". Expected values are {}'.format(
        #             reachy_config[REACHY_CONFIG_MODEL],
        #             [
        #                 FULL_KIT,
        #                 STARTER_KIT_RIGHT,
        #                 STARTER_KIT_LEFT,
        #                 HEADLESS,
        #                 MINI,
        #                 STARTER_KIT_RIGHT_NO_HEAD,
        #             ],
        #         )
        #     )

        # TODO Multiple config
        if not no_print:
            print("Reachy config loaded successfully")
        # exit(1)

    def __str__(self):
        return (
            "robot_model".ljust(25, " ")
            + "{}\n".format(self.model)
            # + "neck_config".ljust(25, " ")
            # + "{}\n".format(self.neck_config)
            # + "right_arm_config".ljust(25, " ")
            # + "{}\n".format(self.right_arm_config)
            # + "left_arm_config".ljust(25, " ")
            # + "{}\n".format(self.left_arm_config)
            # + "ethercat".ljust(25, " ")
            # + "{}\n".format(self.ethercat)
        )

    def part_conf(self, part, fake=False):
        def build_part_conf_path(part, mode):
            # return f'{REACHY_CONFIG_PATH}/{mode}/{self.config["reachy"]["config"]["reachy2_configuration"][part][mode]}'
            part_config_key = self.config["reachy"]["config"]["reachy2_configuration"][part]["default"].replace(".yaml", "")
            # print("part_config_key: ", part_config_key)
            # print("part: ", self.config[f"{part_config_key}"]["path"])
            if mode != "fake":
                return self.config[part_config_key]["path"]
            else:
                return f'{self.config[part_config_key]["package_path"]}/fake/{self.config["reachy"]["config"]["reachy2_configuration"][part]["fake"]}'
            # return f'{REACHY_CONFIG_PATH}/{mode}/{self.config["reachy"]["config"]["reachy2_configuration"][part][mode]}'

        # force fake mode
        if fake:
            return build_part_conf_path(part, "fake")
        else:  # can be fake, override or default,
            return build_part_conf_path(part, self.config["reachy"]["config"]["reachy2_configuration"][part]["mode"])


# def get_reachy_config():
#     with open(config_file) as f:
#         config = yaml.load(f, Loader=yaml.FullLoader)
#         return config


def log_config(xacro_config):
    log = ""
    for line in xacro_config:
        if line != " ":
            key, value = line.split(":=")
            key = key.strip()
            log += f"{key.ljust(25, ' ')}{value}\n"

    return log


# def _get_config_parameter(parameter: str, part=None):
#     config = get_reachy_config()
#     try:
#         return config[parameter]
#     except KeyError:
#         print(f"{parameter} not found in {config_file}")
#         return -1


# get_reachy_generation = partial(_get_config_parameter, "generation")
# get_reachy_model = partial(_get_config_parameter, "model")
# get_reachy_serial_number = partial(_get_config_parameter, "serial_number")
# get_zuuu_version = partial(_get_config_parameter, "zuuu_version")
# get_neck_orbita_zero = partial(_get_config_parameter, "neck_orbita_zero")
# get_camera_parameters = partial(_get_config_parameter, "camera_parameters")
# get_fan_trigger_temperature = partial(_get_config_parameter, "fan_trigger_temperature")


def config_check():
    # set log level to debug
    logging.basicConfig(level=logging.DEBUG)

    rc = ReachyConfig()
    print(rc)
    # print(rc.config["reachy"])
    dump_yaml(None, rc.config["reachy"]["config"]["reachy2_configuration"])
    # print(rc.config["reachy"]["config"]["mobile_base"]["enable"])
    # print(get_reachy_generation())
    # print(get_reachy_model())
    # print(get_reachy_serial_number())
    # print(get_zuuu_version())
    # print(get_neck_orbita_zero())
    # print(get_camera_parameters())
    # print(get_fan_trigger_temperature())
    # print(get_reachy_config())
    # print(log_config(get_reachy_config()))

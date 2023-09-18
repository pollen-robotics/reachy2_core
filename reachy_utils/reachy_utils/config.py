from functools import partial
import yaml
import os


config_file = os.path.expanduser("~/.reachy-v2.yaml")


def get_config_file_content():
    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        return config


def _get_config_parameter(parameter: str, part=None):
    config = get_config_file_content()
    try:
        return config[parameter]
    except KeyError:
        print(f"{parameter} not found in {config_file}")
        return -1


get_reachy_config = partial(_get_config_parameter, "config")
get_reachy_serial_number = partial(_get_config_parameter, "serial_number")
get_zuuu_version = partial(_get_config_parameter, "zuuu_version")
get_parts_availability = partial(_get_config_parameter, "parts")
get_software_info = partial(_get_config_parameter, "software")
get_camera_parameters = partial(_get_config_parameter, "camera_parameters")

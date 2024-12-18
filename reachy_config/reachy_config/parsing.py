import yaml


# DynamixelSerialIO
class DynamixelSerialIO:
    def __init__(self, serial_port_name, id):
        self.serial_port_name = serial_port_name
        self.id = id

    def __repr__(self):
        return f"DynamixelSerialIO(serial_port_name={self.serial_port_name}, id={self.id})"

    @staticmethod
    def constructor(loader, node):
        value = loader.construct_mapping(node, deep=True)
        return DynamixelSerialIO(serial_port_name=value["serial_port_name"], id=value["id"])

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_mapping("!DynamixelSerialIO", {"serial_port_name": data.serial_port_name, "id": data.id})


yaml.SafeLoader.add_constructor("!DynamixelSerialIO", DynamixelSerialIO.constructor)
yaml.SafeDumper.add_representer(DynamixelSerialIO, DynamixelSerialIO.representer)


# AngleLimits:
class AngleLimits:
    def __init__(self, min, max):
        self.min = min
        self.max = max

    def __repr__(self):
        return f"AngleLimits(min={self.min}, max={self.max})"

    @staticmethod
    def constructor(loader, node):
        value = loader.construct_mapping(node)
        return AngleLimits(min=value["min"], max=value["max"])

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_mapping("!AngleLimits:", {"min": data.min, "max": data.max})


yaml.SafeLoader.add_constructor("!AngleLimits:", AngleLimits.constructor)
yaml.SafeDumper.add_representer(AngleLimits, AngleLimits.representer)


# PouleEthercat
class PoulpeEthercat:
    def __init__(self, port_name, id):
        self.port_name = port_name
        self.id = id

    def __repr__(self):
        return f"PoulpeEthercat(port_name={self.port_name}, id={self.id})"

    @staticmethod
    def constructor(loader, node):
        return loader.construct_mapping(node)

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_mapping("!PoulpeEthercat", {"port_name": data.port_name, "id": data.id})


yaml.SafeLoader.add_constructor("!PoulpeEthercat", PoulpeEthercat.constructor)
yaml.SafeDumper.add_representer(PoulpeEthercat, PoulpeEthercat.representer)

# def poulpe_ethercat_constructor(loader, node):
#     return loader.construct_mapping(node)


# def angle_limits_constructor(loader, node):
#     value = loader.construct_mapping(node)
#     return f"AngleLimits(min={value['min']}, max={value['max']})"


def firmware_zero_constructor(loader, node):
    return "FirmwareZero()"


def xl330_constructor(loader, node):
    return "XL330()"


def xm_constructor(loader, node):
    return "XM()"


# Define representers for custom tags (for dumping YAML)


# def dynamixel_serial_io_representer(dumper, data):
#     return dumper.represent_mapping("!DynamixelSerialIO", data.value)


# def poulpe_ethercat_representer(dumper, data):
#     return dumper.represent_mapping("!PoulpeEthercat", data)


# def angle_limits_representer(dumper, data):
#     return dumper.represent_mapping("!AngleLimits:", data)


def firmware_zero_representer(dumper, data):
    return dumper.represent_scalar("!FirmwareZero", "")


def xl330_representer(dumper, data):
    return dumper.represent_scalar("!XL330", "")


# also add !XM


def xm_representer(dumper, data):
    return dumper.represent_scalar("!XM", "")


# Register all constructors with SafeLoader
# yaml.SafeLoader.add_constructor("!PoulpeEthercat", poulpe_ethercat_constructor)
# yaml.SafeLoader.add_constructor("!AngleLimits:", angle_limits_constructor)
yaml.SafeLoader.add_constructor("!FirmwareZero", firmware_zero_constructor)
yaml.SafeLoader.add_constructor("!XL330", xl330_constructor)
yaml.SafeLoader.add_constructor("!XM", xm_constructor)


class FirmwareZero:
    pass


class XL330:
    pass


class XM:
    pass


# Register representers
# yaml.SafeDumper.add_representer(dict, poulpe_ethercat_representer)
# yaml.SafeDumper.add_representer(dict, angle_limits_representer)
yaml.SafeDumper.add_representer(FirmwareZero, firmware_zero_representer)
yaml.SafeDumper.add_representer(XL330, xl330_representer)
yaml.SafeDumper.add_representer(XM, xm_representer)


# Example classes for custom tags


def load_yaml(file_path):
    with open(file_path) as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def dump_yaml(file_path, data):
    print("\nData:")
    print(data)
    output_yaml = yaml.dump(data, Dumper=yaml.SafeDumper, default_flow_style=False)
    print("\nDumped YAML:")
    print(output_yaml)
    # exit(1)
    # with open(file_path, "w") as f:
    #     yaml.dump(data, f)

import yaml


def perror(msg):
    print("\033[91m" + msg + "\033[0m")
    exit(1)


# DynamixelSerialIO
class DynamixelSerialIO:
    def __init__(self, serial_port_name, id, mode=None, current_limit=None, model=None):
        self.serial_port_name = serial_port_name
        self.id = id
        self.mode = mode
        self.current_limit = current_limit
        self.model = model

        self.validate()

    def __repr__(self):
        return f"DynamixelSerialIO(serial_port_name={self.serial_port_name}, id={self.id})"

    def __eq__(self, other):
        return self.serial_port_name == other.serial_port_name and self.id == other.id

    @staticmethod
    def constructor(loader, node):
        value = loader.construct_mapping(node, deep=True)
        return DynamixelSerialIO(serial_port_name=value["serial_port_name"], id=value["id"])

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_mapping("!DynamixelSerialIO", {"serial_port_name": data.serial_port_name, "id": data.id})

    def validate(self):
        """
        Perform basic validation of the object's attributes.

        Raises:
            ValueError: If any of the attributes do not meet the expected criteria.
        """
        if not isinstance(self.serial_port_name, str) or not self.serial_port_name.startswith("/dev/"):
            perror(f"DynamixelSerialIO : Invalid serial_port_name: {self.serial_port_name}")
        if not isinstance(self.id, int) or not (1 <= self.id <= 253):
            perror(f"DynamixelSerialIO : Invalid id: {self.id}")
        if self.mode is not None and (not isinstance(self.mode, int) or not (1 <= self.mode <= 5)):
            perror(f"DynamixelSerialIO : Invalid mode: {self.mode}")
        if self.current_limit is not None and (perror(self.current_limit, float) or not (0.0 <= self.current_limit <= 1.0)):
            perror(f"DynamixelSerialIO : Invalid current_limit: {self.current_limit}")
        if self.model is not None and not isinstance(self.model, str):
            perror(f"DynamixelSerialIO : Invalid model: {self.model}")


yaml.SafeLoader.add_constructor("!DynamixelSerialIO", DynamixelSerialIO.constructor)
yaml.SafeDumper.add_representer(DynamixelSerialIO, DynamixelSerialIO.representer)


# AngleLimits:
class AngleLimits:
    def __init__(self, min, max):
        self.min = min
        self.max = max

        self.validate()

    def __repr__(self):
        return f"AngleLimits(min={self.min}, max={self.max})"

    def __eq__(self, other):
        return self.min == other.min and self.max == other.max

    @staticmethod
    def constructor(loader, node):
        value = loader.construct_mapping(node)  # This expects a dictionary
        # Ensure min and max exist and are floats
        min_val = value.get("min")
        max_val = value.get("max")

        if isinstance(min_val, float) and isinstance(max_val, float):
            return AngleLimits(min_val, max_val)
        raise yaml.YAMLError("Invalid AngleLimits data: 'min' and 'max' must be floats.")

    @staticmethod
    def representer(dumper, data):
        # When dumping, we represent it as a custom tag !AngleLimits
        return dumper.represent_mapping("!AngleLimits:", {"min": data.min, "max": data.max})

    def validate(self):
        if not isinstance(self.min, float) or not (-18000.0 <= self.min <= 18000.0):
            perror(f"AngleLimits : Invalid min: {self.min}")
        if not isinstance(self.max, float) or not (-18000.0 <= self.max <= 18000.0):
            perror(f"AngleLimits : Invalid max: {self.max}")


yaml.SafeLoader.add_constructor("!AngleLimits:", AngleLimits.constructor)
yaml.SafeDumper.add_representer(AngleLimits, AngleLimits.representer)


# PoulpeEthercat
class PoulpeEthercat(dict):
    """Dict subclass so downstream code accesses fields normally while preserving the YAML tag on dump."""

    def __repr__(self):
        return f"PoulpeEthercat({dict.__repr__(self)})"

    @staticmethod
    def constructor(loader, node):
        value = loader.construct_mapping(node)
        return PoulpeEthercat(value)

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_mapping("!PoulpeEthercat", dict(data))


yaml.SafeLoader.add_constructor("!PoulpeEthercat", PoulpeEthercat.constructor)
yaml.SafeDumper.add_representer(PoulpeEthercat, PoulpeEthercat.representer)

# def poulpe_ethercat_constructor(loader, node):
#     return loader.construct_mapping(node)


# def angle_limits_constructor(loader, node):
#     value = loader.construct_mapping(node)
#     return f"AngleLimits(min={value['min']}, max={value['max']})"


def firmware_zero_constructor(loader, node):
    return FirmwareZero()


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
    def __init__(self):
        self.label = "FirmwareZero"

    def __repr__(self):
        return "FirmwareZero()"

    def __eq__(self, other):
        return self.label == other.label


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


# FakeMotors
class FakeMotors(dict):
    """Dict subclass preserving the !FakeMotors tag on dump."""

    @staticmethod
    def constructor(loader, node):
        value = loader.construct_mapping(node, deep=True)
        return FakeMotors(value)

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_mapping("!FakeMotors", dict(data))


yaml.SafeLoader.add_constructor("!FakeMotors", FakeMotors.constructor)
yaml.SafeDumper.add_representer(FakeMotors, FakeMotors.representer)


class ZeroStartup:
    def __repr__(self):
        return "ZeroStartup()"

    def __eq__(self, other):
        return isinstance(other, ZeroStartup)


def zero_startup_constructor(loader, node):
    return ZeroStartup()


def zero_startup_representer(dumper, data):
    return dumper.represent_scalar("!ZeroStartup", "")


yaml.SafeLoader.add_constructor("!ZeroStartup", zero_startup_constructor)
yaml.SafeDumper.add_representer(ZeroStartup, zero_startup_representer)


# Poulpe
class Poulpe:
    def __init__(self, id, orbita_type, name):
        self.id = id
        self.orbita_type = orbita_type
        self.name = name

        self.validate()

    def __repr__(self):
        return f"Poulpe(id={self.id}, orbita_type={self.orbita_type}, name={self.name})"

    def __eq__(self, other):
        return self.id == other.id and self.orbita_type == other.orbita_type and self.name == other.name

    @staticmethod
    def constructor(loader, node):
        value = loader.construct_mapping(node, deep=True)
        return Poulpe(
            id=value["id"],
            orbita_type=value["orbita_type"],
            name=value["name"],
        )

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_mapping(
            "!Poulpe",
            {
                "id": data.id,
                "orbita_type": data.orbita_type,
                "name": data.name,
            },
        )

    def validate(self):
        if not isinstance(self.id, int) or self.id < 0:
            perror(f"Poulpe : Invalid id: {self.id}")
        if not isinstance(self.orbita_type, int) or self.orbita_type < 0:
            perror(f"Poulpe : Invalid orbita_type: {self.orbita_type}")
        if not isinstance(self.name, str) or not self.name.strip():
            perror(f"Poulpe : Invalid name: {self.name}")


# Register Poulpe with YAML
yaml.SafeLoader.add_constructor("!Poulpe", Poulpe.constructor)
yaml.SafeDumper.add_representer(Poulpe, Poulpe.representer)


def load_yaml(file_path):
    with open(file_path) as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def dump_yaml(file_path, data):
    output = yaml.dump(data, Dumper=yaml.SafeDumper, default_flow_style=False, sort_keys=False)
    # Remove trailing empty-string quotes added by PyYAML for scalar custom tags
    output = output.replace("!FirmwareZero ''", "!FirmwareZero")
    output = output.replace("!XL330 ''", "!XL330")
    output = output.replace("!XM ''", "!XM")

    if file_path is None:
        print(output)
    else:
        with open(file_path, "w") as f:
            f.write(output)

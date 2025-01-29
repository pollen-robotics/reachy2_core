from cerberus import TypeDefinition, Validator

from reachy_config.parsing import AngleLimits, DynamixelSerialIO, FirmwareZero, dump_yaml, load_yaml


def validate_schema(schema_path, config_dict, logger):
    schema = load_yaml(schema_path)

    # Extend the Cerberus validator with your custom type
    class CustomValidator(Validator):
        types_mapping = Validator.types_mapping.copy()
        types_mapping["dynamixelserialio"] = TypeDefinition("dynamixelserialio", (DynamixelSerialIO,), ())
        # also handle anglelimits
        types_mapping["anglelimits"] = TypeDefinition("anglelimits", (AngleLimits,), ())
        # firmwarezero
        types_mapping["firmwarezero"] = TypeDefinition("firmwarezero", (FirmwareZero,), ())

    # dump_yaml(None, config_dict)

    # Check if the config is valid
    validator = CustomValidator(schema)
    # print("Config dict: ", config_dict)
    if validator.validate(config_dict):
        logger.debug("Configuration is valid.")
    else:
        # print("Validation errors:")
        # logger.error("Validation errors:")
        # pritn in RED
        print("\033[91mValidation errors:\033[0m")

        for field, errors in validator.errors.items():
            # logger.error(f"  - {field}: {errors}")
            print(f"\033[91m  - {field}: {errors}\033[0m")
        return True
    return False


def validate(config, logger):
    print("\nValidating configuration files...")

    got_errors = False

    for config_file in config:
        if config_file != "reachy":
            continue
        logger.debug(f"Validating {config_file}...")
        # print path
        # logger.debug(f"Path: {config[config_file]['path']}")
        # if has schema, print it
        if "schema" in config[config_file]:
            logger.debug(f"Schema found, validating...")
            got_errors |= validate_schema(config[config_file]["schema"], config[config_file]["config"], logger)
            # logger.debug(f"Schema: {config[config_file]['schema']}")
        else:
            logger.debug("No schema found.")
        logger.debug("\n")

    if got_errors:
        print("\033[91mConfiguration files are invalid.\033[0m")
        exit(1)

    # exit(1)
    # print("\n\nValidating config...")
    # print("Config: ", config)
    # Load the schema

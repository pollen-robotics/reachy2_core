import logging


# Merge config files
def merge_config(base, override, key_path=""):
    for key in override:
        if key in base:
            if isinstance(base[key], dict):
                merge_config(base[key], override[key], key_path + key + ".")
            else:  # override is a replace
                if base[key] != override[key]:
                    print(f"  Replaced {key_path + key}\t{base[key]} => {override[key]}")
                    base[key] = override[key]

        else:  # override is an add
            base[key] = override[key]
            print(f"  Added {key_path + key}: {override[key]}")


def get_logger():
    logger = logging.getLogger(__name__)
    handler = logging.StreamHandler()

    formatter = logging.Formatter("%(message)s")

    handler.setFormatter(formatter)

    logger.propagate = False
    logger.addHandler(handler)
    return logger

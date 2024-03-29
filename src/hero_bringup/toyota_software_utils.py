from typing import Dict, List

import logging
import os
import rospkg.common
import rospkg.rospack
from xml.etree.ElementTree import ElementTree
import yaml

RP = None


def read_dict_yaml(file_path: str) -> dict:
    """
    Read a yaml file, which should be a dict at toplevel

    :param file_path: yaml file to read
    :return: yaml object of dictionaries and lists
    :raises yaml.parser.ParserError, yaml.scanner.ScannerError: In case of invalid yaml syntax
    """

    with open(file_path) as f:
        try:
            yaml_obj = yaml.load(f, yaml.CSafeLoader)
        except AttributeError:
            yaml_obj = yaml.load(f, yaml.SafeLoader)
        except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
            logging.getLogger(__name__).error(f"Invalid yaml syntax: {e}")
            raise

    return yaml_obj


def get_rospack() -> rospkg.rospack.RosPack:
    """
    Get a reference to the global RosPack instance. It is initialized in case it is not yet done.
    The construction takes a long time, therefore it is preferred to only have one instance.

    :return: RosPack instance
    """
    global RP
    if RP is None:
        RP = rospkg.rospack.RosPack()
    return RP


def get_compatible_toyota_software_version_file() -> str:
    """
    Provides the path at which the version of the compatible Toyota software should be stored. This is different for
    dev pc's and the robot.

    :return: path to yaml file
    """
    hero_bringup_path = get_rospack().get_path("hero_bringup")
    compatible_pkgs_file = os.path.join(
        hero_bringup_path,
        "toyota_robot_versions.yaml" if os.environ.get("ROBOT_REAL", False) else "toyota_dev_versions.yaml",
    )
    return compatible_pkgs_file


def get_installed_toyota_pkgs() -> List[str]:
    """
    Provides a list of currently installed packages of Toyota. It checks for 'hsr' or 'tmc' in the name.

    :return: list of names of all installed toyota packages
    """
    rp = get_rospack()
    opt_ros_path = os.path.join(os.sep, "opt", "ros")
    toyota_pkgs = [
        pkg
        for pkg in rp.list()
        if ("hsr" in pkg or "tmc" in pkg) and rp.get_path(pkg).startswith(opt_ros_path)
    ]
    return toyota_pkgs


def pkgs_to_version_dict(pkg_list: List[str]) -> Dict[str, str]:
    """
    Get the version of all packages in a list. The results are stored in a dict. The key is the package name and the
    version is stored as the value.

    :param pkg_list: List of package names
    :return: Version dictionary
    :raises Exception: In case a package file is missing or can't be parsed
    """
    installed_pkgs = {}
    mm = rospkg.rospack.ManifestManager(rospkg.common.PACKAGE_FILE)
    for pkg in pkg_list:
        path = mm.get_path(pkg)
        package_manifest = os.path.join(path, rospkg.common.PACKAGE_FILE)
        try:
            root = ElementTree(None, package_manifest)
            version = root.findtext("version")
        except Exception as e:
            logging.getLogger(__name__).error(
                f"Error during parsing of '{rospkg.common.PACKAGE_FILE}' of package '{pkg}': {e}"
            )
            raise
        installed_pkgs.update({pkg: version})

    return installed_pkgs

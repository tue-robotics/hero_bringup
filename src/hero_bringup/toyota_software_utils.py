import logging
import os
import rospkg.common
import rospkg.rospack
from xml.etree.ElementTree import ElementTree
import yaml

RP = None


def read_dict_yaml(file_path):
    # type: (str) -> dict
    """
    Read a yaml file, which should be a dict at toplevel

    :param file_path: yaml file to read
    :type file_path: str
    :return: yaml object of dictionaries and lists
    :rtype: dict
    :raises yaml.parser.ParserError, yaml.scanner.ScannerError: In case of invalid yaml syntax
    """

    with open(file_path) as f:
        try:
            yaml_obj = yaml.load(f, yaml.CSafeLoader)
        except AttributeError:
            yaml_obj = yaml.load(f, yaml.SafeLoader)
        except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
            logging.getLogger(__name__).error("Invalid yaml syntax: {0}".format(e))
            raise

    return yaml_obj


def get_rospack():
    # type: () -> rospkg.rospack.RosPack
    """
    Get a reference to the global RosPack instance. It is initialized in case it is not yet done.
    The construction takes a long time, therefone it is preffered to only have one instance.

    :return: RosPack instance
    :rtype: rospkg.rospack.RosPack
    """
    global RP
    if RP is None:
        RP = rospkg.rospack.RosPack()
    return RP


def get_compatible_toyota_software_version_file():
    # type: () -> str
    """
    Provides the path at which the version of the compatible Toyota software should be stored. This is different for
    dev pc's and the robot.

    :return: path to yaml file
    :rtype: str
    """
    hero_bringup_path = get_rospack().get_path("hero_bringup")
    compatible_pkgs_file = os.path.join(
        hero_bringup_path,
        "toyota_robot_versions.yaml" if os.environ.get("ROBOT_REAL", False) else "toyota_dev_versions.yaml",
    )
    return compatible_pkgs_file


def get_installed_toyota_pkgs():
    # type: () -> list
    """
    Provides a list of currently installed packages of Toyota. It checks for 'hsr' or 'tmc' in the name.

    :return: list of names of all installed toyota packages
    :rtype: list
    """
    rp = get_rospack()
    toyota_pkgs = [pkg for pkg in rp.list() if "hsr" in pkg or "tmc" in pkg]
    return toyota_pkgs


def pkgs_to_version_dict(pkg_list):
    # type: (list) -> dict
    """
    Get the version of all packages in a list. The results are stored in a dict. The key is the package name and the
    version is stored as the value.

    :param pkg_list: List of package names
    :type pkg_list: list
    :return: Version dictionary
    :rtype: dict
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
            logging.getLogger(__name__).error("Error during parsing of '{}' of package '{}': {}".format(rospkg.common.PACKAGE_FILE, pkg, e))
            raise
        installed_pkgs.update({pkg: version})

    return installed_pkgs


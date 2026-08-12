from glob import glob
from setuptools import find_packages, setup
import sys

package_name = "franka_lock_unlock"


if len(sys.argv) >= 2 and sys.argv[1] != "clean":
    from generate_parameter_library_py.setup_helper import generate_parameter_module

    # set module_name and yaml file
    module_name = "franka_lock_unlock_params"
    yaml_file = "franka_lock_unlock/params.yaml"
    generate_parameter_module(module_name, yaml_file)

setup(
    name=package_name,
    version="4.2.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*launch.[pxy][yma]*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jk-ethz",
    maintainer_email="ethz@juliankeller.net",
    description="Lock or unlock the Franka Emika Panda joint brakes programmatically.",
    license="AGPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "franka_lock_unlock = franka_lock_unlock.franka_lock_unlock:main",
            "franka_shutdown = franka_lock_unlock.franka_shutdown:main",
            "franka_lock_unlock_node = franka_lock_unlock.nodes.franka_lock_unlock_node:main",
        ],
    },
)

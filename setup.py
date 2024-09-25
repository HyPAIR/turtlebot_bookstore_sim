from setuptools import find_packages, setup
from glob import glob
import os

package_name = "turtlebot_bookstore_sim"


def package_files(directory_list):
    """Glob wasn't working how I intended, so I took this nice function from:
    https://answers.ros.org/question/397319/how-to-copy-folders-with-subfolders-to-package-installation-path/
    """
    paths_dict = {}
    data_files = []
    for directory in directory_list:
        for path, _, filenames in os.walk(directory):

            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=package_files(["models/", "maps/", "params/"])
    + [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.world*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Charlie Street",
    maintainer_email="me@charliestreet.net",
    description="A simulation of a Turtlebot in a bookstore. Used to evaluate refine-plan for CONVINCE.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)

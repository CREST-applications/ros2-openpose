from setuptools import find_packages, setup
import os
from glob import glob

package_name = "pose"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.py")),
    ],
    install_requires=[
        "setuptools",
        "opencv-python",
        "numpy",
        "pydantic",
        "cv_bridge",
    ],
    zip_safe=True,
    maintainer="root",
    maintainer_email="jme.co.jp@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera = camera.main:main",
            "display = display.main:main",
            "requester = requester.test:main",
        ],
    },
)

from setuptools import find_packages, setup

package_name = "renderer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "main = renderer.main:main",
        ],
    },
)

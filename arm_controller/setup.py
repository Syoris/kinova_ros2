from setuptools import find_packages, setup

package_name = "arm_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="charles",
    maintainer_email="charles.sirois@mail.mcgill.ca",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_controller = arm_controller.arm_controller:main",
            "demo = arm_controller.demo:main",
        ],
    },
)

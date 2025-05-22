from setuptools import find_packages, setup

package_name = "rokey"

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
    maintainer="juwan",
    maintainer_email="dlacksdn352@gmail.com",
    description="ROKEY BOOT CAMP Package",
    license="Apache 2.0 License",
    entry_points={
        "console_scripts": [
            "simple_amove=rokey.basic.amove_test:main",
            "force_control = rokey.basic.force_control:main",
            "get_current_pos=rokey.basic.get_current_pos:main",
            "getting_position = rokey.basic.getting_position:main",
            "grip=rokey.basic.grip:main",
            "jog = rokey.basic.jog_complete:main",
            "move_periodic = rokey.basic.move_periodic:main",
            "simple_move=rokey.basic.move:main",
            "simple_movesx=rokey.basic.movesx_test:main",
            "data_recording=rokey.basic.data_recording:main"
        ],
    },
)

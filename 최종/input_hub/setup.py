from setuptools import find_packages, setup

package_name = 'input_hub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='josh',
    maintainer_email='josh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "input_hub_node=input_hub.con_to_hub:main",
             'gripper_state_sub = input_hub.gripper_state_sub:main',
        ],
    }
)
from setuptools import setup
import os
from glob import glob

package_name = 'manage_hub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        f'{package_name}.nodes',
        f'{package_name}.core',
        f'{package_name}.utils',
        f'{package_name}.config'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyeongyeon',
    maintainer_email='hyeongyeon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hub_node = manage_hub.nodes.input_hub_node:main',
            'unload_node = manage_hub.nodes.unload_node:main',
        ],
    },
) 
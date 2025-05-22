from setuptools import setup, find_packages

package_name = 'input_hub'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyeongyeon',
    maintainer_email='your.email@example.com',
    description='Input Hub Robot Control Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_hub_node = input_hub.nodes.input_hub_node:main',
        ],
    },
) 
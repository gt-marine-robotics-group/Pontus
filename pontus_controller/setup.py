import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pontus_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*[pxy][yma]*'))),

        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),

        (os.path.join('share', package_name, 'test'),
            glob(os.path.join('test', '.flake8'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mroglan',
    maintainer_email='manueljoseph113@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller = pontus_controller.velocity_controller:main',
            'hybrid_controller = pontus_controller.hybrid_controller:main',
            'thruster_controller = pontus_controller.thruster_controller:main',
            'direct_thruster_controller = pontus_controller.direct_thruster_controller:main',
            'firmware_cmds = pontus_controller.firmware_cmds:main',
            'stop = pontus_controller.stop:main',
            'position_controller = pontus_controller.position_controller:main',
            'command_mode_mux = pontus_controller.command_mode_mux:main',
            'joy_listener = pontus_controller.joy_listener:main'
        ],
    },
)

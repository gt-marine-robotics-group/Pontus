from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pontus_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

        (os.path.join('share', package_name, 'config/'),
            glob(os.path.join('config',  '*.yaml'))),

        (os.path.join('share', package_name, 'yolo/auv'),
            glob(os.path.join('yolo', 'auv', '*.pt'))),

        (os.path.join('share', package_name, 'yolo/sim'),
            glob(os.path.join('yolo', 'sim', '*.pt'))),
        (os.path.join('share', package_name, 'test'),
            glob(os.path.join('test', '.flake8'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbturton',
    maintainer_email='mbturton33@gmail.com',
    description='ROS Package to handle the perception onboard our RoboSub vehicle. This includes processing of sensor data through computer vision methodds such as YOLO and tools for working with our sonar data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo = pontus_perception.yolo_node:main',
            'color_thresolding = pontus_perception.color_thresholding:main'
        ],
    },
)

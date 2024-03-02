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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        
        (os.path.join('share', package_name, 'config/auv'), 
            glob(os.path.join('config', 'auv', '*.yaml'))),
        
        (os.path.join('share', package_name, 'config/sim'), 
            glob(os.path.join('config', 'sim', '*.yaml'))),

        (os.path.join('share', package_name, 'yolo/auv'),
            glob(os.path.join('yolo', 'auv', '*.pt'))),

        (os.path.join('share', package_name, 'yolo/sim'),
            glob(os.path.join('yolo', 'sim', '*.pt')))
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
            'yolo = pontus_perception.yolo.yolo_node:main'
        ],
    },
)

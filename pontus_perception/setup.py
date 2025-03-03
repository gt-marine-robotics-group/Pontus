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
            'yolo = pontus_perception.yolo_node:main',
            'image_preprocessing = pontus_perception.image_preprocessing.preprocessing_node:main',
            'sonoptix_cloud_transform = pontus_perception.sonoptix_pc_transform:main',
            'sonoptix = pontus_perception.sonoptix:main',
            'point_cloud_camera = pontus_perception.point_cloud_camera:main',
            'point_cloud_downsampling = pontus_perception.point_cloud_downsampling:main',
            'gate_detection = pontus_perception.gate_detection.gate_detection:main',
            'vertical_marker_detection = pontus_perception.vertical_marker_detection.vertical_marker_detection:main',
            'bag2mp4 = pontus_perception.bag2mp4:main',
            'yolo_pose_detection = pontus_perception.yolo_pose_detection:main',
        ],
    },
)

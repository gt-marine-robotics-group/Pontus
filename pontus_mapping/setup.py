from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pontus_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'visual_meshes'), 
            glob(os.path.join('visual_meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_map_manager = pontus_mapping.exploration_map_manager:main',
            'semantic_map_manager = pontus_mapping.semantic_map_manager:main',
        ],
    },
)

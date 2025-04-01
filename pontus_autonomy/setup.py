from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pontus_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch',
                                                                          '*.launch.py'))),
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
            'prequal_run = pontus_autonomy.prequalification_run_localization:main',
            'semi_run = pontus_autonomy.semi_run_localization:main',
            'search_region_test = pontus_autonomy.helpers.SearchRegionClient:main'
        ],
    },
)

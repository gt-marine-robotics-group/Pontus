import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pontus_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*[pxy][yma]*'))),
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
        ],
    },
    scripts = [
        'scripts/record_all_topics.sh',
    ],
)

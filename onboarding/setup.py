from setuptools import find_packages, setup

package_name = 'onboarding'

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
    maintainer='root',
    maintainer_email='jeff300fang@gmail.com',
    description='Package purely used for onboarding. Should have no actual effect on the sub',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onboarding_node_topic1 = onboarding.questions.topic1:main',
        ],
    },
)

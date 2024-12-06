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
            'test_topic_1_1 = onboarding.autograder.topic_1.question1_1_grader:main',
            'test_topic_1_2 = onboarding.autograder.topic_1.question1_2_grader:main',
            'test_topic_1_3 = onboarding.autograder.topic_1.question1_3_grader:main',
            'test_topic_2_1 = onboarding.autograder.topic_2.question2_1_grader:main',
            'test_topic_2_2 = onboarding.autograder.topic_2.question2_2_grader:main',
            'node_q_1_1 = onboarding.autograder.topic_1.mock_node:main',
        ],
    },
)

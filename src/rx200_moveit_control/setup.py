from setuptools import setup

package_name = 'rx200_moveit_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='MoveIt control nodes for RX200 robot arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rx200_moveit_control = rx200_moveit_control.rx200_moveit_action_client:main',
            'rx200_gripper_control = rx200_moveit_control.rx200_gripper_control:main',
        ],
    },
)

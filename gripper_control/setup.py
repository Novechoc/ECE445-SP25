from setuptools import setup

package_name = 'gripper_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/gripper_launch.py', 'launch/combined_launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'zdt_stepper', 'pyserial'],
    zip_safe=True,
    maintainer='Yuxi Chen',
    maintainer_email='yuxi5@illinois.edu',
    description='Gripper control using stepper motor and FSR sensor in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = gripper_control.gripper_node:main',
        ],
    },
)
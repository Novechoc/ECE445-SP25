from setuptools import setup

package_name = 'fsr_sensor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gpiozero'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='FSR sensor node publishing pressure state',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsr_node = fsr_sensor.fsr_node:main',
        ],
    },
)
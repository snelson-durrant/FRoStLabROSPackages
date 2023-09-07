import os
from glob import glob
from setuptools import setup

package_name = 'frost_uuv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='High-level control ROS nodes for use with the FRoSt Lab UUV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_instructions_pub = frost_uuv.nav_instructions_publisher:main',
            'echo_data_pub = frost_uuv.echo_data_publisher:main',
            'leak_detected_sub = frost_uuv.leak_detected_subscriber:main',
            'voltage_sub = frost_uuv.voltage_subscriber:main',
            'humidity_sub = frost_uuv.humidity_subscriber:main',
            'gps_data_pub = frost_uuv.gps_data_publisher:main',
        ],
    },
)

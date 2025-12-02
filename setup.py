import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ugv_peripherals'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ruthvick Bandaru, Stanley Cokro, Pedram Kayedpour, Eric Yang',
    maintainer_email='ruthvick.bandaru@stonybrook.edu, scokro7141@gmail.com, pedram.kayedpour@stonybrook.edu, eric.yang2@.stonybrook.edu',
    description='This package contains software for testing and controlling all of the rover peripherals.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
       'console_scripts': [
            # AUDIO NODES
            'periodic_growl = ugv_peripherals.audio.periodic_growl_node:main',
            'custom_play_audio = ugv_peripherals.audio.custom_play_audio:main',
            'speaker = ugv_peripherals.audio.speaker_node:main',

            # SERIAL NODES
            'serial_controller = ugv_peripherals.serial.serial_controller_node:main',
            'serial_echo = ugv_peripherals.serial.serial_echo_node:main',
            'get_tty_acm = ugv_peripherals.serial.get_tty_acm:main',

            # LIGHTS NODES
            'flashing = ugv_peripherals.lights.flashing_node:main',
            'battery = ugv_peripherals.lights.battery_node:main',
        ],
    },
)

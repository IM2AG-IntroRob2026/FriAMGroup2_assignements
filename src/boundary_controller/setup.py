import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'boundary_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alioune-diene',
    maintainer_email='alioune@example.com',
    description='Turtle boundary drawing with interruptible manual teleop',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'boundary_fsm      = boundary_controller.boundary_fsm:main',
            'keyboard_teleop   = boundary_controller.keyboard_teleop:main',
        ],
    },
)

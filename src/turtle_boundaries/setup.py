import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'turtle_boundaries'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # THIS LINE IS THE ONE THAT INSTALLS THE LAUNCH FILE
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hebasleiman',
    maintainer_email='Heba.SLeiman@univ-grenoble-alpes.fr',
    description='Turtle boundary drawing',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'boundary_drawer = turtle_boundaries.boundary_drawer:main',
            'teleop_handler  = turtle_boundaries.teleop_handler:main',
        ],
    },
)

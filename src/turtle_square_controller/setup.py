from setuptools import find_packages, setup

package_name = 'turtle_square_controller'

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
    maintainer='hebasleiman',
    maintainer_email='Heba.SLeiman@univ-grenoble-alpes.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
    'console_scripts': [
        'draw_square_server = turtle_square_controller.draw_square_server:main',
    ],
},
)

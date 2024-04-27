from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'robox'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfw',
    maintainer_email='shahfaisalwani14@gmail.com',
    description='Robox package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drive_node = robox.drive_node:main",
            "key_input_node = robox.key_input_node:main",
            "lidar_node = robox.lidar_node:main",
            "encoder_node = robox.encoder_node:main",
            "dynamic_transform = robox.dynamic_transform:main"
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'robox'

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
    maintainer='sfw',
    maintainer_email='sfw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drive_node = robox.drive_node:main",
            "key_input_node = robox.key_input_node:main",
            "lidar_node = robox.lidar_node:main"
        ],
    },
)

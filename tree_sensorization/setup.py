from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tree_sensorization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/tree_sensorization_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imml',
    maintainer_email='frostmar@oregonstate.edu',
    description='TODO: Package description',
    license='BSD3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_sensor_subscriber = tree_sensorization.multi_sensor_subscriber:main',
            'tree_sensorization_launch = tree_sensorization.launch.tree_sensorization_launch:main',
        ],
    },
)

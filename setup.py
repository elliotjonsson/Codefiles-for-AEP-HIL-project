import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aep_package_vr'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='testuser',
    maintainer_email='user@example.com',
    description='AEP VR Robot Control Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_listener = aep_package_vr.VR_listener:main',
            'lidar_distance = aep_package_vr.lidar_distance_brake:main',
            'gen_steering = aep_package_vr.gen_steering:main',
            'message_node = aep_package_vr.message_node:main',
            'stream = aep_package_vr.stream:main',
            'aep_launch = aep_package_vr.aep_launcher:main',
        ],
    },
)

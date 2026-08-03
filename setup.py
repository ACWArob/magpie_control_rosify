from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'magpie_control'
package_version = '2024.9'

setup(
    name=package_name,
    version=package_version,
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stefan Caldararu',
    maintainer_email='stefan.caldararu@colorado.edu',
    description='Gripper, F/T sensor, and tactile sensor control for MAGPIE',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = magpie_control.gripper_node:main',
            'ft_sensor_node = magpie_control.ft_sensor_node:main',
            'tactile_sensor_node = magpie_control.tactile_sensor_node:main',
            'ur5_node = magpie_control.ur5_node:main',
            'deligrasp_node = magpie_control.deligrasp_node:main',
        ],
    },
)

import os
from glob import glob

from setuptools import setup

package_name = 'racer_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdw',
    maintainer_email='alexshin3@kookmin.ac.kr',
    description='D-Racer 실차 액추에이터 브링업/캘리브레이션 (Stage 6-a)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = racer_bringup.calibration_node:main',
        ],
    },
)

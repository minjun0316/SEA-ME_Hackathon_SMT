import os
from glob import glob

from setuptools import setup

package_name = 'd_racer_perception'


def files_only(pattern):
    """data_files는 디렉토리를 못 담는다 → glob에서 정규파일만 남긴다.

    (models/ 에 ncnn 디렉토리가 있으면 setuptools copy가 실패하므로.)
    """
    return [p for p in glob(pattern) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         files_only('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         files_only('config/*.yaml')),
        (os.path.join('share', package_name, 'models'),
         files_only('models/*')),
        (os.path.join('share', package_name, 'test_data'),
         files_only('test_data/*')),
        (os.path.join('share', package_name, 'scripts'),
         files_only('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SMT',
    maintainer_email='alexshin3@kookmin.ac.kr',
    description='D-Racer perception experiment nodes for YOLO detection tests.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detect_test_node = d_racer_perception.yolo_detect_test_node:main',
            'lane_detect_node = d_racer_perception.lane_detect_node:main',
        ],
    },
)

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'perception_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SMT',
    maintainer_email='alexshin3@kookmin.ac.kr',
    description='YOLO26n-seg 세그멘테이션 추론 노드 (camera 이미지 구독 → 추론 → 판단쪽 발행).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_seg_node = perception_yolo.yolo_seg_node:main',
        ],
    },
)

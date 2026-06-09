import os
from glob import glob

from setuptools import setup

package_name = 'openarmx_ptp_pick'

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
    # 런타임 튜닝 데이터(pick_seq_config_{side}.yaml, {side}_grasp_reference_model.yaml)는
    # core 가 dirname(__file__) 로 읽는다 → 모듈 디렉토리에 동봉(--symlink-install 시 src 에서 직접 읽힘).
    package_data={package_name: ['*.yaml']},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='openarmx',
    maintainer_email='kukwonko@gmail.com',
    description='ptp pick&place 정본: 상주 픽 서버(좌/우) + 검출 루프 + 컨테이너 게이트',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptp_pick_resident = openarmx_ptp_pick.ptp_pick_resident:main',
            'ptp_pick_seq = openarmx_ptp_pick.ptp_pick_seq_v2_left:main',
            'box_detect_loop = openarmx_ptp_pick.box_detect_loop:main',
            'container_pick_gate = openarmx_ptp_pick.container_pick_gate:main',
        ],
    },
)

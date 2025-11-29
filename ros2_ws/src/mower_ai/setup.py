# setup.py 

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mower_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 'launch' 폴더에 있는 모든 .py 파일을 찾아서,
        # 설치 시 'share/mower_ai/launch' 폴더에 복사해달라는 규칙입니다.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models'), glob('mower_ai/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sy',
    maintainer_email='sy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_controller = mower_ai.ai_controller_node:main',
        ],
    },
)
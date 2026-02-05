from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kinematics_so100'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/display.launch.py', 'launch/rsp.launch.py']),
        (f'share/{package_name}/so100_description', glob('so100_description/*.urdf')),
        (f'share/{package_name}/so100_description/assets', glob('so100_description/assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koki',
    maintainer_email='koki.muramoto77@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_publish_example = kinematics_so100.joint_state_publish_example:main',
            'yolob8_realsense_example = kinematics_so100.yolob8_realsense_example:main',
            'translate_joint_name = kinematics_so100.translate_joint_name:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'x1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share','x1_description','rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='controlslab',
    maintainer_email='mnguyen6@unb.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Mcnamu_driver = x1_bringup.Mcnamu_driver:main',
            'image_republisher = x1_bringup.image_republisher:main'
        ],
    },
)

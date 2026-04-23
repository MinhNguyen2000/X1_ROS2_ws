from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'x1_drl_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include the model for DRL inference
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
            for f in glob('policies/**/*', recursive=True) if os.path.isfile(f)
        ]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='mnguyen6@unb.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'policy_node = x1_drl_policy.policy_node:main',
        ],
    },
)

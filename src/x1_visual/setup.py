from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'x1_visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        # config files
        # model weights
        (os.path.join('share', package_name, 'models', 'face_detection'), glob('models/face_detection/*.onnx')),
        (os.path.join('share', package_name, 'models', 'emotion_recognition'), glob('models/emotion_recognition/*.onnx'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'face_detection_node = x1_visual.face_detection_node:main',
            'emotion_recognition_node = x1_visual.emotion_recognition_node:main'
        ],
    },
)

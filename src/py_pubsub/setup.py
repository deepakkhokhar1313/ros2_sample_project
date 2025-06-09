import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.talker:main',
            'listener = py_pubsub.listener:main',
            'param_talker = py_pubsub.param_talker:main',
            'service_server = py_pubsub.service_server:main',
            'service_client = py_pubsub.service_client:main',
            'action_server = py_pubsub.action_server:main',
            'action_client = py_pubsub.action_client:main',
            'robot_controller = py_pubsub.robot_controller:main',
            'robot_commander = py_pubsub.robot_commander:main',
        ],
    },
)

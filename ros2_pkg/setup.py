from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qos_demo_sub_node = ros2_pkg.qos_demo_sub_node:main',
            'qos_demo_pub_node = ros2_pkg.qos_demo_pub_node:main',
            'img_publisher_node = ros2_pkg.img_publisher_node:main',
            'img_subscriber_node = ros2_pkg.img_subscriber_node:main',
            'PurePursuit = ros2_pkg.pure_pursuit:main',
            'pure_pursuit_param = ros2_pkg.pure_pursuit_param:main',
            'pure = ros2_pkg.pure:main'
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'back_to_the_feature'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igor',
    maintainer_email='igor@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'static_turtle_tf2_broadcaster = back_to_the_feature.static_turtle_tf2_broadcaster:main',
        'turtle_tf2_broadcaster = back_to_the_feature.turtle_tf2_broadcaster:main',
        'turtle_tf2_listener = back_to_the_feature.turtle_tf2_listener:main',
        'fixed_frame_tf2_broadcaster = back_to_the_feature.fixed_frame_tf2_broadcaster:main',
        'dynamic_frame_tf2_broadcaster = back_to_the_feature.dynamic_frame_tf2_broadcaster:main',
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_rpi_bot'

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
    maintainer='aryan',
    maintainer_email='aryankumarnadipally@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = vision_rpi_bot.publisher:main',
            'subscriber_node = vision_rpi_bot.subscriber:main',
            'line_follower_node = vision_rpi_bot.line_following_real:main',
            'lane_follower_node = vision_rpi_bot.lane_following:main',

                           ],
    },
)

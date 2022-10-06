from setuptools import setup
import os
from glob import glob

package_name = 'crazyflie_examples'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    package_data={'package_name': ['data/*.csv']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('config/*')),
        (os.path.join('share', package_name, 'data'), glob('data/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfgang Hönig',
    maintainer_email='hoenig@tu-berlin.de',
    description='Examples for Crayzswarm2 ROS stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = crazyflie_examples.hello_world:main',
            'nice_hover = crazyflie_examples.nice_hover:main',
            'figure8 = crazyflie_examples.figure8:main',
            'cmd_full_state = crazyflie_examples.cmd_full_state:main',
        ],
    },
)

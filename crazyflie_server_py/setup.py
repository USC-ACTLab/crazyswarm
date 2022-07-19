from setuptools import setup

package_name = 'crazyflie_server_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kimberly McGuire',
    maintainer_email='kimberly@bitcraze.io',
    description='Crazyflie ROS2 communication server based on the crazyflie python library (CFlib)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'crazyflie_server_py = crazyflie_server_py:main',
        ],
    },
)

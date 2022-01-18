from setuptools import setup

package_name = 'crazyswarm2_examples'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    package_data={'package_name': ['data/*.csv']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'hello_world = crazyswarm2_examples.hello_world:main',
            'nice_hover = crazyswarm2_examples.nice_hover:main',
            'figure8 = crazyswarm2_examples.figure8:main',
            'cmd_full_state = crazyswarm2_examples.cmd_full_state:main',
        ],
    },
)

from setuptools import setup

package_name = 'prolb_haring'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start.launch.py']),
        ('share/' + package_name + '/map', ['map/map.yaml', 'map/map.pgm'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your-name',
    maintainer_email='re23m009@technikum-wien.at',
    description='Full bringup for TurtleBot3 with Gazebo, Nav2 and RViz2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
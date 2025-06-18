from glob import glob
from setuptools import find_packages, setup

package_name = 'patrol_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayeon',
    maintainer_email='hayeon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'waypoint_follower = patrol_system.waypoint_follower:main',
        'analyzer_node = patrol_system.analyzer_node:main',
        ],
    },
)

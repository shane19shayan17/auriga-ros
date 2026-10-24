from glob import glob
from setuptools import setup

package_name = 'grid_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Text grid map + Nav2 planner_server learning project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_publisher = grid_nav.map_publisher:main',
            'runner = grid_nav.runner:main',
        ],
    },
)

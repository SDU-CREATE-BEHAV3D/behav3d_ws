from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'world_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab',
    maintainer_email='lab@sdu.dk',
    description='Mesh visualization node for reconstructed scans',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mesh_visualizer = world_visualizer.mesh_visualizer:main',
        ],
    },
)

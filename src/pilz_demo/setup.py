from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pilz_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),        # Marker
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas José Helle',
    maintainer_email='luh@iti.sdu.dk',
    description='Interactive demo that converts text commands into Pilz-planned motions and visualises them.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_demo = pilz_demo.demo:main',
        ],
    },
)

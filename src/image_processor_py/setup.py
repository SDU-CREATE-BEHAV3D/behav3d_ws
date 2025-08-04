from setuptools import setup, find_packages
from glob import glob

package_name = 'image_processor_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'image_processor_node = image_processor_py.image_processor_node:main',
            'image_processor_capture = image_processor_py.image_processor_capture:main',
        ],
    },
)

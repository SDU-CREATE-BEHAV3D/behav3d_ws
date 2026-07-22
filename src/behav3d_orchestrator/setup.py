from glob import glob

from setuptools import setup, find_packages

package_name = 'behav3d_orchestrator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Özgüç Bertuğ Çapunaman',
    maintainer_email='ozca@iti.sdu.dk',
    description='Core Python library for BEHAV3D @ SDU CREATE',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'print_yaml_and_scan_sequence = behav3d_orchestrator.print_yaml_and_scan_sequence:main',
            'scan_sequence = behav3d_orchestrator.scan_sequence:main',
            'print_field_oriented_sequence_v2 = behav3d_orchestrator.print_field_oriented_sequence_v2:main',
            'polyline_motion_sequence = behav3d_orchestrator.polyline_motion_sequence:main',
            'scan_yaml_targets_sequence = behav3d_orchestrator.scan_yaml_targets_sequence:main',
        ],
    },
)

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
            'orchestrator_node = behav3d_orchestrator.orchestrator_node:main',
            'yaml_target_sequence = behav3d_orchestrator.yaml_target_sequence:main',
            'print_path_sequence = behav3d_orchestrator.print_path_sequence:main',
            'print_dots_sequence = behav3d_orchestrator.print_dots_sequence:main',
            'print_scan_dots_sequence = behav3d_orchestrator.print_scan_dots_sequencenc:main',
            'print_scan_dots_sequencenc = behav3d_orchestrator.print_scan_dots_sequencenc:main',
            'depth_bias_capture_sequence = behav3d_orchestrator.depth_bias_capture_sequence:main',
            'print_field_sequence = behav3d_orchestrator.print_field_sequence:main',
            'print_field_centered_sequence = behav3d_orchestrator.print_field_centered_sequence:main',
            'print_field_oriented_sequence = behav3d_orchestrator.print_field_oriented_sequence:main',
            'interactive_scan_motion_sequence = behav3d_orchestrator.interactive_scan_motion_sequence:main',
        ],
    },
)

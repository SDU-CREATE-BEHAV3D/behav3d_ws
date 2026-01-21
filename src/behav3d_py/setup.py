from setuptools import setup, find_packages

package_name = 'behav3d_py'

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
    ],
    entry_points={
        'console_scripts': [
            'modbus_test = behav3d_py.modbus_test:main',
            'print_test = behav3d_py.print_test:main',
            'move_and_print_test = behav3d_py.move_and_print_test:main',
            'move_and_print_test_async = behav3d_py.move_and_print_test_async:main',
            'run_yaml_test = behav3d_py.run_yaml_test:main',
            'handeye_capture_sequence = behav3d_py.handeye_capture_sequence:main',
            'orchestrator_test = behav3d_py.orchestrator_test:main',
            'session_sync_demo = behav3d_py.session_sync_demo:main',

        ],
    },
)

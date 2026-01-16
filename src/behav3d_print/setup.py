from setuptools import setup, find_packages

package_name = 'behav3d_print'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools','pymodbus'],
    zip_safe=True,
    maintainer='Lucas Jose Helle',
    maintainer_email='luh@iti.sdu.dk',
    description='Printing control node for BEHAV3D.',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'print_node = behav3d_print.print_node:main',
        ],
    },
)

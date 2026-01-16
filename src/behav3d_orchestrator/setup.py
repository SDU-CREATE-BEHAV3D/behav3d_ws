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
    ],
    entry_points={
        'console_scripts': [
            'orchestrator_node = behav3d_orchestrator.orchestrator_node:main',
        ],
    },
)

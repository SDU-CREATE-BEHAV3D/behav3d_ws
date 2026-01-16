from setuptools import setup, find_packages

package_name = 'behav3d_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='behav3d',
    maintainer_email='you@example.com',
    description='RGB-D capture/processing (MVP)',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'image_manager = behav3d_vision.image_manager:main',
            'capture_sync = behav3d_vision.capture_sync:main',
        ],
    },
)

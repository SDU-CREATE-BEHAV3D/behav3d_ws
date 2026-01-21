from setuptools import find_packages, setup

package_name = 'behav3d_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='behav3d lab',
    maintainer_email='lab@iti.sdu.dk',
    description='Example sessions built on behav3d_commands.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_and_print_test = behav3d_examples.move_and_print_test:main',
            'handeye_capture_sequence = behav3d_examples.handeye_capture_sequence:main',
            'custom_sequence = behav3d_examples.custom_sequence:main',
        ],
    },
)

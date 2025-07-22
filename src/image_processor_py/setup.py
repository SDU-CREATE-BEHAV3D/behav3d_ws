from setuptools import find_packages, setup

package_name = 'image_processor_py'

setup(
    name='image_processor_py',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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


from setuptools import setup

package_name = 'motion_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luc',
    maintainer_email='tu@correo.com',
    description='Dummy setup.py for building',
    license='MIT',
    tests_require=['pytest'],
)

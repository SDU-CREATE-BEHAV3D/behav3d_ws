from setuptools import setup

package_name = 'pilz_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@mail.com',
    description='Dummy setup.py for building',
    license='MIT',
    tests_require=['pytest'],
)

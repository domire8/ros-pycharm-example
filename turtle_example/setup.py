import os
from glob import glob

from setuptools import setup

package_name = 'turtle_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Reber',
    maintainer_email='dominic.reber@epfl.ch',
    description='The turtle_example package',
    license='TODO',
    package_dir={'': 'src'},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['circler = turtle_example.circler:main']
    }
)

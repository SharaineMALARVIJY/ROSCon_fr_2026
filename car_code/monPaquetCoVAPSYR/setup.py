import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'monPaquetCoVAPSYR'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sorbonne',
    maintainer_email='sorbonne@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'CoVAPSy_cmdR = monPaquetCoVAPSYR.CoVAPsy_cmdR:main',
            'CoVAPSy_conduiteR = monPaquetCoVAPSYR.CoVAPSy_conduiteR:main'
        ],
    },
)

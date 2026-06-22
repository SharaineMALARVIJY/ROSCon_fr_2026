from glob import glob
from setuptools import find_packages, setup

package_name = 'webots_rosconfr'
data_files = [
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*')),
    ('share/' + package_name + '/worlds', glob('worlds/*')),
    ('share/' + package_name + '/resource', glob('resource/*')),
    ('share/' + package_name + '/protos', glob('protos/*')),
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name + '/controllers/controller_violet',
        ['controllers/controller_violet/controller_violet.py'])
]

# colcon make a warning if append is not used
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sylvain ARGENTIERI',
    maintainer_email='sylvain.argentieri@sorbonne-universite.fr',
    description='Webots package for the ROSCon FR 2026 Hackathon',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)

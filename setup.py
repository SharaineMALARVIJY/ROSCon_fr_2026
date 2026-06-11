from setuptools import find_packages, setup

package_name = 'rosconfr'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch',
                  ['launch/webots_rosconfr.launch.py']))
data_files.append(('share/' + package_name + '/launch',
                  ['launch/webots_rosconfr_cam.launch.py']))
data_files.append(('share/' + package_name + '/worlds',
                  ['worlds/Piste_CoVAPSy_2025a.wbt']))
data_files.append(('share/' + package_name + '/worlds',
                  ['worlds/Piste_CoVAPSy_2025a_camera.wbt']))
data_files.append(('share/' + package_name + '/worlds',
                  ['worlds/ImageToStl_virage.obj']))
data_files.append(('share/' + package_name + '/resource',
                  ['resource/TT02_jaune.urdf']))
data_files.append(('share/' + package_name + '/resource',
                  ['resource/TT02_jaune_cam.urdf']))
data_files.append(('share/' + package_name + '/protos',
                  ['protos/TT02_2025a.proto']))
data_files.append(('share/' + package_name + '/protos',
                  ['protos/TT02_2025a_camera.proto']))
data_files.append(('share/' + package_name + '/protos',
                  ['protos/TT02Wheel.proto']))
data_files.append(('share/' + package_name + '/protos',
                  ['protos/ChevroletCamaroLight.stl']))
data_files.append(('share/' + package_name + '/controllers/controller_violet',
                   ['controllers/controller_violet/controller_violet.py']))
data_files.append(('share/' + package_name, ['package.xml']))

# voiture bleu
data_files.append(('share/' + package_name + '/resource',
                  ['resource/TT02_bleu.urdf']))
data_files.append(('share/' + package_name + '/controllers/controller_blue',
                   ['controllers/controller_blue/controller_blue.py']))
data_files.append(('share/' + package_name + '/launch',
                  ['launch/blue_odom.launch.py']))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosconfr',
    maintainer_email='rosconfr@todo.todo',
    description='Simulaton webots pour le Hackathon ROSConfr 2026',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_ackermann = rosconfr.teleop_ackermann:main',
            'ps4_teleop_ackermann = rosconfr.ps4_teleop_ackermann:main',
            'follow_gap_blue = rosconfr.follow_gap_blue:main',
            'dial_nav_blue = rosconfr.dial_nav_blue:main',
            'front_scan_filter = rosconfr.front_scan_filter:main',
        ],
    },
)

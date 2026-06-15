from setuptools import find_packages, setup

package_name = 'webots_rosconfr'
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
data_files.append(('share/' + package_name + '/resource',
                  ['resource/TT02_bleu.urdf']))
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

data_files.append(('share/' + package_name + '/config',
                  ['config/yellow_driver_params.yaml']))
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

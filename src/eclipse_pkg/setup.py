import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'eclipse_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.geojson')
         + glob('config/*.json')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=False,
    maintainer='cargo220',
    maintainer_email='ley184526@icloud.com',
    description='TARS mudflat rescue robot ROS 2 nodes and launch',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'eclipse_test_controller = eclipse_pkg.eclipse_test_controller:main',
            'iahrs = eclipse_pkg.iahrs:main',
            'gps_node = eclipse_pkg.GPS_node:main',
            'gamepad_drive = eclipse_pkg.gamepad_drive:main',
            'yolo_detect_node = eclipse_pkg.yolo_detect_node:main',
            'yolo_3d_node = eclipse_pkg.yolo_3d_node:main',
            'gps_waypoint_commander = eclipse_pkg.gps_waypoint_commander:main',
            'gps_health_supervisor = eclipse_pkg.gps_health_supervisor:main',
            'tide_watch_node = eclipse_pkg.tide_watch_node:main',
            'tide_patrol_node = eclipse_pkg.tide_patrol_node:main',
            'probe_sensor = eclipse_pkg.probe_sensor:main',
        ],
    },
)

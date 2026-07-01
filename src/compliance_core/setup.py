import os
from glob import glob

from setuptools import setup

package_name = 'compliance_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'audio'),
            glob('audio/*.wav') + glob('audio/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deshwin Dharile',
    maintainer_email='deshwind02@gmail.com',
    description='CCTV-triggered compliance robot core nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = compliance_core.yolo_detector_node:main',
            'scenario_publisher_node = compliance_core.scenario_publisher_node:main',
            'tracking_node = compliance_core.tracking_node:main',
            'event_confirmation_node = compliance_core.event_confirmation_node:main',
            'coordinate_transform_node = compliance_core.coordinate_transform_node:main',
            'nav2_goal_sender_node = compliance_core.nav2_goal_sender_node:main',
            'compliance_fsm_node = compliance_core.compliance_fsm_node:main',
            'audio_warning_node = compliance_core.audio_warning_node:main',
            'incident_logger_node = compliance_core.incident_logger_node:main',
            'incident_emailer_node = compliance_core.incident_emailer_node:main',
            'patrol_node = compliance_core.patrol_node:main',
            'return_to_base_node = compliance_core.return_to_base_node:main',
            'l298n_driver_node = compliance_core.l298n_driver_node:main',
            'admin_joy_node = compliance_core.admin_joy_node:main',
            'dashboard_node = compliance_core.dashboard_node:main',
            'camera_publisher_node = compliance_core.camera_publisher_node:main',
            'location_manager_node = compliance_core.location_manager_node:main',
            'safety_monitor_node = compliance_core.safety_monitor_node:main',
            'localization_monitor_node = compliance_core.localization_monitor_node:main',
        ],
    },
)

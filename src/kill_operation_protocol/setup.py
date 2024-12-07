from setuptools import find_packages, setup

package_name = 'kill_operation_protocol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoonkangrok',
    maintainer_email='ykr0919@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_box = kill_operation_protocol.world_view.draw_box:main',
            'pub_cam = kill_operation_protocol.world_view.pub_cam:main',
            'test_sub = kill_operation_protocol.world_view.test_sub:main',

            'web = kill_operation_protocol.flask.web:main',
            'log_publisher = kill_operation_protocol.flask.log_publisher:main',
            'dispatch_receiver = kill_operation_protocol.flask.dispatch_receiver:main',

            'yolo_tracking = kill_operation_protocol.tracking.yolo_tracking:main',
            'tracking_pub_cam = kill_operation_protocol.tracking.tracking_pub_cam:main',

            'start_swat = kill_operation_protocol.swat_nav2.amr_ctrl:main',
        ],
    },
)

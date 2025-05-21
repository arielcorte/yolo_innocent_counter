from setuptools import setup

package_name = 'yolo_innocent_counter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'ultralytics'
        # ultralytics & opencv-python will be installed via pip separately
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 node that counts innocents (teddy bears) via YOLOv8',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_innocent_counter = '
            'yolo_innocent_counter.yolo_innocent_counter:main',
            'camera_streamer = '
+           'yolo_innocent_counter.camera_streamer:main',
            'camera_streamer_perf = '
+           'yolo_innocent_counter.camera_streamer_perf:main',
            'camera_streamer_rpi = '
+           'yolo_innocent_counter.camera_streamer_rpi:main',
        ],
    },
)

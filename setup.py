from setuptools import setup

package_name = 'yolo_person_counter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        # ultralytics & opencv-python will be installed via pip separately
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 node that counts persons via YOLOv8',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_person_counter = '
            'yolo_person_counter.yolo_person_counter:main',
        ],
    },
)

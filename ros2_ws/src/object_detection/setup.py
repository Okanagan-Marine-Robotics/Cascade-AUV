from setuptools import find_packages, setup

package_name = 'object_detection'

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
    maintainer='eryk',
    maintainer_email='erykhalicki0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_detector = object_detection.example_detector:main',
            'yolo_coco_detector = object_detection.yolo_coco_detector:main',
            'dummy_detector = object_detection.example_detector:main',
            'image_data_merger = object_detection.image_data_merger:main',
            'data_collector = object_detection.data_collector:main',
        ],
    },
)

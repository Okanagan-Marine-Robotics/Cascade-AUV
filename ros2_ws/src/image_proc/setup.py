from setuptools import find_packages, setup

package_name = 'image_proc'

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
            'depth_map = image_proc.depth_pub:main',
            'background_remover = image_proc.background_remover:main',
            'depth_labeler = image_proc.depth_labeler:main',
            'data_collector = image_proc.data_collector:main',
        ],
    },
)

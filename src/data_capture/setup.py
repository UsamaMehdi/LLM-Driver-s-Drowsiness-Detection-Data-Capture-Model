from setuptools import find_packages, setup

package_name = 'data_capture'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mos1585',
    maintainer_email='mos1585@thi.de',
    description='A ROS2 package for capturing data from carla simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_capture_node = data_capture.data_capture_node:main'
        ],
    },
)

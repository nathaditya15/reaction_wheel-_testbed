from setuptools import find_packages, setup

package_name = 'attitude_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','adafruit-bno055'],
    zip_safe=True,
    maintainer='Aditya',
    maintainer_email='anroy23@iitk.ac.in',
    description='A ROS2 publisher node for BNO055 sensor data.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_publisher = attitude_node.bno055_publisher:main',
            'attitude_control = attitude_node.attitude_control:main',
            'aruco_tracker = attitude_node.aruco_tracker:main',
        ],
    },
)

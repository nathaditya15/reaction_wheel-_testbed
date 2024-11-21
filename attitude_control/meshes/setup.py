from setuptools import setup

package_name = 'attitude_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', ['meshes/satellite.STL']),  # Include the STL file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdfcl',
    maintainer_email='sdfcl@todo.todo',
    description='Visualization package for displaying satellite STL model in RViz2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_stl = attitude_control.display_stl:main',
        ],
    },
)

import os
from setuptools import setup
from glob import glob

package_name = 'eye_on_you'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Include the mesh files in the share directory
        (os.path.join('share', package_name, 'meshes'), ['meshes/arm.stl', 'meshes/base.stl', 'meshes/home.stl']),
        # Include other necessary files (e.g., URDF)
        (os.path.join('share', package_name), ['urdf/robot.urdf']),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        # Install Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Include RViz configuration file
        (os.path.join('share', package_name, 'rviz'), ['rviz/robot_view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kadir Yavuz Kurt',
    maintainer_email='k.yavuzkurt1@gmail.com',
    description='Simulated Robot That Tracks Faces',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = eye_on_you.camera_node:main',
            'face_detection_node = eye_on_you.face_detection:main',
            'face_detection_simulation = eye_on_you.face_detection_simulation:main',
            'mearm_controller = eye_on_you.mearm_controller:main',
            'servo_controller = eye_on_you.servo_controller:main',
            'simulation_controller = eye_on_you.simulation_controller:main'
        ],
    },
)

from setuptools import setup

package_name = 'omnirevolve_ros2_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='IvanNekrasov',
    author_email='userofbgtu@gmail.com',
    maintainer='IvanNekrasov',
    maintainer_email='userofbgtu@gmail.com',
    description='Minimal GUI + ROS2 node to control the OmniRevolve Plotter and publish byte stream.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omnirevolve_ros2_ui = omnirevolve_ros2_ui.omnirevolve_ros2_ui_node:main',
        ],
    },
)


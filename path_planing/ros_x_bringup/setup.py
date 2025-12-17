from setuptools import find_packages, setup

package_name = 'ros_x_bringup'

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
    maintainer='vuminh',
    maintainer_email='vuminh2012004@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_sender = ros_x_bringup.path_sender_node:main',
            'debug_plotter = ros_x_bringup.debug_plotter:main',
        ],
    },
)

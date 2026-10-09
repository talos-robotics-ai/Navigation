import glob

from setuptools import find_packages, setup

package_name = 'x2_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*.urdf')),
        ('share/' + package_name + '/tools', glob.glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ggal',
    maintainer_email='gabriel.voss01@gmail.com',
    description='X2 laptop-side bring-up',
    license='MIT',
    entry_points={'console_scripts': [
        'robojudo_link_node = x2_bringup.robojudo_link_node:main',
        'dummy_obstacles_node = x2_bringup.dummy_obstacles_node:main',
    ]},
)

import glob

from setuptools import find_packages, setup

package_name = 'x2_box_pnp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ggal',
    maintainer_email='gabriel.voss01@gmail.com',
    description='X2 box pick-and-place FSM',
    license='MIT',
    entry_points={'console_scripts': ['pnp_fsm_node = x2_box_pnp.pnp_fsm_node:main']},
)

from setuptools import find_packages, setup

package_name = 'agentic_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TalOS Robotics',
    maintainer_email='dev@talos-robotics.ai',
    description='Agentic navigation layer (skills + interface) over the geometric A*+MPC stack.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Phase-1 low-level nav test harness (no VLM/agent): drive the existing
            # planner through the NavigationInterface and report goal outcomes.
            'low_level_nav_test = agentic_nav.nodes.low_level_nav_test_node:main',
        ],
    },
)

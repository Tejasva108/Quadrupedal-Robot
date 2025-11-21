from setuptools import find_packages, setup

package_name = 'robotdog_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=['IK', 'cmd_manager', 'body_motion_planner', 'gait_generator'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tejasva',
    maintainer_email='tejasva108@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'IK_node = IK.inverse_kinematics_node:main',
            'cmd_manager_node = cmd_manager.main:main',
        ],
    },
)

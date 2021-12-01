from setuptools import setup

package_name = 'dm_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel@todo.todo',
    description='Hardware control package for diymore 6dof arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_movement_server = dm_arm_control.arm_movement_server:main',
            'kinematics_service = dm_arm_control.kinematics_service:main',
            'joint_driver = dm_arm_control.joint_driver_server:main'
        ],
    },
)

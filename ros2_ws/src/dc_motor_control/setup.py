from setuptools import find_packages, setup

package_name = 'dc_motor_control'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_node_py = dc_motor_control.motor_node:main',
            'serial_motor_node_py = dc_motor_control.serial_motor_node:main',
            'motor_ctl_py = dc_motor_control.motor_ctl:main',
        ],
    },
)

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
    maintainer='osugi_w',
    maintainer_email='osugiartow@gmail.com',
    description='GUI control for Final project ELEC537',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gui_control_py = dc_motor_control.gui_control:main'
        ],
    },
)

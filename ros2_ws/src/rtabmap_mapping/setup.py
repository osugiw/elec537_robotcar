from setuptools import find_packages, setup

package_name = 'rtabmap_mapping'

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
            'realsense_d435i_color_py = rtabmap_mapping.realsense_d435i_color:main',
            'realsense_d435i_infra_py = rtabmap_mapping.realsense_d435i_infra:main',
            'realsense_d435i_stereo_py = rtabmap_mapping.realsense_d435i_stereo:main',
        ],
    },
)

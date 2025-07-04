from setuptools import find_packages, setup

package_name = 'vrremote'

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
    maintainer='jyw',
    maintainer_email='jiyingwei@openloong.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'mocap_ros2_node = src.mocap_ros2_node:main',
             'webrtc_ros2_node = src.webrtc_ros2_node:main'
        ],
    },
)

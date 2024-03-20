from setuptools import setup

package_name = 'lidar'

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
    maintainer='pimpmobile',
    maintainer_email='pimpmobile@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'lidar_talker = lidar.publisher_member_function:main',
        	'lidar_listener = lidar.lidarsubscriber:main',
        ],
    },
)

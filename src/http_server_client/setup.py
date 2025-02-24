from setuptools import find_packages, setup

package_name = 'http_server_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','flask'],
    zip_safe=True,
    maintainer='Sai Ganesh Raju',
    maintainer_email='rsaiganesh74@gmail.com',
    description='TODO: This package hosts the API endpoints for the WMS and the robot cell',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_server = http_server_client.robot_http_server:main',
        ],
    },
)

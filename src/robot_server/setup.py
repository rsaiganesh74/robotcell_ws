from setuptools import find_packages, setup

package_name = 'robot_server'

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
    maintainer='Sai Ganesh Raju',
    maintainer_email='rsaiganesh74@gmail.com',
    description='TODO: This package mimics the actual robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_server = robot_server.robot_server:main',
        ],
    },
)

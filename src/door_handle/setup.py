from setuptools import find_packages, setup

package_name = 'door_handle'

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
    description='TODO: This package mimics a door handle of a robot cell',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_handle = door_handle.door_handle:main',
        ],
    },
)

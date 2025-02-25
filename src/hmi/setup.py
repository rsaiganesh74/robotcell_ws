from setuptools import find_packages, setup
from glob import glob

package_name = 'hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('hmi/config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sai Ganesh Raju',
    maintainer_email='rsaiganesh74@gmail.com',
    description='TODO: This package contains the HMI for the entire system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmi_node = hmi.hmi:main',
        ],
    },
)

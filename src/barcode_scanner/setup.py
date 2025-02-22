from setuptools import find_packages, setup

package_name = 'barcode_scanner'

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
    description='TODO: This package will mimic a barcode scanner',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanner = barcode_scanner.scanner:main',
        ],
    },
)

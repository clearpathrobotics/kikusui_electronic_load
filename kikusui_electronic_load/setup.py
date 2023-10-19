from setuptools import find_packages, setup

package_name = 'kikusui_electronic_load'

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
    maintainer='administrator',
    maintainer_email='rfaultless@clearpathrobotics.com',
    description='Driver for controlling Kikusui PLZ1004W electronic load',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = kikusui_electronic_load.kikusui_electronic_load:main',
        ],
    },
)
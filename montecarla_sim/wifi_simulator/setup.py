from setuptools import setup

package_name = 'wifi_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adrián',
    maintainer_email='adrmoralf@gmail.com',
    description='WiFi simulator for Montecarla experiment',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'wifi_simulator_node = wifi_simulator.node:main',
        ],
    },
)

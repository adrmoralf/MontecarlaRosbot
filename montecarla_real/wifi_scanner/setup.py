from setuptools import setup

setup(
    name='wifi_scanner',
    version='0.1.0',
    packages=['wifi_scanner'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'wifi_scanner_node = wifi_scanner.wifi_scanner_node:main',
        ],
    },
)

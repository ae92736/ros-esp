from setuptools import setup
from glob import glob
import os

package_name = 'sim_io'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aye',
    maintainer_email='ae.92736@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_input = sim_io.input:main',
            'sim_output = sim_io.output:main',
            'starter = sim_io.sim_start:main',
            "mqtt = sim_io.mqtt_out:main"
        ],
    },
)

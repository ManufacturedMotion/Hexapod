from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hexapod_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ensure launch files are installed
        (os.path.join('share', package_name), glob('launch/**')),
        ('share/hexapod_manager/resource', ['resource/battery_table.json', 'resource/manufactured_motion_arrow.png', 'resource/mokoto.regular.ttf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_controller = hexapod_manager.led_controller:main',
            'display = hexapod_manager.display:main',
            'status_reporter = hexapod_manager.status_reporter:main',
        ],
    },
)

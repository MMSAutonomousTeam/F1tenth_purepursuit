from setuptools import setup
import os
from glob import glob
from setuptools import setup
package_name = 'vehicle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

        
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

            'teleop_controller = vehicle_controller.teleop_controller:main', 
            'purepursuit_controller = vehicle_controller.purepursuit_controller:main',
            "wall_followe_controller=vehicle_controller.wall_follower_controller:main"
        ],
    },
)
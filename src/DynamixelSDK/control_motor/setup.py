from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'control_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob(os.path.join('launch','*launch.[pxy][yma]'))),
        (os.path.join('share',package_name,'config'), glob(os.path.join('config/*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anderson',
    maintainer_email='anderson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_dual_xl = control_motor.control_dual_xl:main',
            'control_dual_xm = control_motor.control_dual_xm:main',

            'cpxl = control_motor.cpxl:main',
	    'dp = control_motor.dp:main',
	    'seven = control_motor.seven:main',
            'position_control = control_motor.position_control:main',

            'Twist2Speed = control_motor.Twist2Speed:main',

        ],
    },
)

from setuptools import find_packages, setup

package_name = 'fly'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_pub.launch.py']),  
    ],
    install_requires=['setuptools', 'pytest'],  
    zip_safe=True,
    maintainer='notchad069',
    maintainer_email='imacoder1809@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cmd_node = fly.vehiclecmd:main',
            'mode_node = fly.offboard:main',
            'pos_node = fly.trajectorysp:main',
            'all_nodes = fly.all_nodes:main',
            'cam_node = fly.cam:main',
            'key_node = fly.keyboard_control:main',
            'mul_way = fly.mult_waypoints:main',
            'pid_node = fly.pid:main',
            'lqr_node = fly.lqr:main',
            'aruco_node = fly.aruco:main',
            'hover_node = fly.hover:main',
            'trajectory_node = fly.drone_aruco_trajectory:main',
        ],
    },
)


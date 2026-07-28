from setuptools import setup

package_name = 'robot_odometry'

setup(
    name=package_name,          # ✅ robot_odometry (underscore)
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Encoder odometry node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'odom_node = robot_odometry.odom_node:main',
        ],
    },
)


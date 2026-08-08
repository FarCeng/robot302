from setuptools import find_packages, setup

package_name = 'robot302_autonomy'

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
    maintainer='farceng',
    maintainer_email='farceng@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'multi_waypoint = robot302_autonomy.multi_waypoint:main',
            'patroli = robot302_autonomy.patroli:main',
            'robot_test_runner = robot302_autonomy.robot_test_runner:main',
        ],
    },
)

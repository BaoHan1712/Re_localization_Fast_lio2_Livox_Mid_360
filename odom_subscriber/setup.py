from setuptools import find_packages, setup

package_name = 'odom_subscriber'

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
    maintainer='hanbao',
    maintainer_email='hanbao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'show_odom = odom_subscriber.odom_listener:main',
        'odom_3d_map = odom_subscriber.odom_3d_map:main',
    ],
},
)

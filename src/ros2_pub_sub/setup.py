from setuptools import setup

package_name = 'ros2_pub_sub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Chuang',
    maintainer_email='ericchuang0929@gmail.com',
    description='publish and subscribe image folder and system time',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'Pub_ImageFolder = ros2_pub_sub.Pub_ImageFolder:main',
                'Sub_ImageFolder = ros2_pub_sub.Sub_ImageFolder:main',
                'sender_time = ros2_pub_sub.sender_time:main',
                'receiver_time = ros2_pub_sub.receiver_time:main',
        ],
    },
)

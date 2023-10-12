from setuptools import setup

package_name = 'my_sailboat_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='github@JohnkaGL',
    maintainer_email='john.giraldo2@udea.edu.co',
    description='This package implements sensors and controllers into ROS system in order to make an autonomus sailing boat. It is conciebed \
                to receive info trought MCUs as ESP32 and Raspberry Pi pico, or ROS enable simulation plattforms sensors as Gazebo or WeBotz \
                to create a Digital Twin capable of implementing Hardware in the Loop strategies to train controllers',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_controller_node= my_sailboat_controller.my_controller_node:main",  ### Para lanzar los scripts con el comando 'ros2 run [package] [node]'
            "decode_node= my_sailboat_controller.my_decoding_node:main",  ### To be able to launch scripts with 'ros2 run [package] [node]' command
            "gz_decode_node= my_sailboat_controller.gazebo_node:main",
        ],
    },
)

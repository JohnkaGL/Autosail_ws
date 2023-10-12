from setuptools import setup

package_name = 'GUI'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,'GUI/Simulator'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='camilo',
    maintainer_email='camilo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'GUI_node = GUI.GUI_node:main', ### Process node
            'GUI_start = GUI.GUI_init:main', ### Configurate Simulation node
        ],
    },
)

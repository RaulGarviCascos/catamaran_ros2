from setuptools import find_packages, setup

package_name = 'py_pkg'

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
    maintainer='raulgarvicascos',
    maintainer_email='raul.garvi.cascos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "my_listener = py_pkg.my_listener:main",
            "my_publisher = py_pkg.my_publisher:main",
            "control_velocity_py = py_pkg.control_velocity_py:main",
            "joystick_cmd_vel = py_pkg.joystick_pygame_cmd_vel:main",
        ],
    },
)

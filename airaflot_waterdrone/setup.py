from setuptools import find_packages, setup

package_name = 'airaflot_waterdrone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'airaflot_msgs', 'RPi.GPIO', 'rpimotorlib'],
    zip_safe=True,
    maintainer='airalab',
    maintainer_email='airaflot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'rc_controller = airaflot_waterdrone.rc_commands_controller:main',
                'water_sampler_servo = airaflot_waterdrone.peripheral_modules.water_sampler.servo:main',
                'water_sampler_motor = airaflot_waterdrone.peripheral_modules.water_sampler.motor:main',
                'water_sampler = airaflot_waterdrone.peripheral_modules.water_sampler.water_sampler:main',
                'ecostab_sensors_publisher = airaflot_waterdrone.peripheral_modules.ecostab_sensors.publisher:main'
        ],
},
)

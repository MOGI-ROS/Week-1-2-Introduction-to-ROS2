from setuptools import find_packages, setup

package_name = 'bme_ros2_tutorials_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    #packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='Python tutorials for the BME MOGI ROS2 course',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_hello_world = scripts.hello_world:main',
            'py_publisher = scripts.publisher:main',
            'py_publisher_oop = scripts.publisher_oop:main',
            'py_subscriber = scripts.subscriber:main',
            'py_subscriber_oop = scripts.subscriber_oop:main',
            'py_publisher_with_param = scripts.publisher_with_parameter:main',
            'py_service_server = scripts.service_server:main'
        ],
    },
)

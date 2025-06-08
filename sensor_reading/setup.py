from setuptools import find_packages, setup

package_name = 'sensor_reading'

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
    maintainer='nidhin',
    maintainer_email='nj82523@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = sensor_reading.sensor_publisher:main',
            'sensor_subscriber = sensor_reading.sensor_subscriber:main',
        ],
    },
)

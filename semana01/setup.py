from setuptools import find_packages, setup

package_name = 'semana01'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_publisher = semana01.temperature_publisher:main',
            'temperature_monitor = semana01.temperature_monitor:main',
            'temperature_avg_monitor = semana01.temperature_avg_monitor:main',
            'average_reset_client = semana01.average_reset_client:main'
        ],
    },
)

from setuptools import setup

package_name = 'turtles_service'

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
    maintainer='jun',
    maintainer_email='your_email@example.com',
    description='A package to call NavigateToPose action from a service',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_service = turtles_service.nav_service:main',
            'waypoint_service = turtles_service.waypoint_service:main'

        ],
    },
)

from setuptools import find_packages, setup

package_name = 'wiz_status'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnav',
    maintainer_email='arnavkatewale@gmail.com',
    description='Drone status publisher/subscriber nodes for the Wiz stack',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_publisher = wiz_status.status_publisher:main',
            'status_subscriber = wiz_status.status_subscriber:main',
        ],
    },
)

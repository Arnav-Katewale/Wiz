from setuptools import find_packages, setup

package_name = 'wiz_terrain'

setup(
    name=package_name,
    version='0.4.0',
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
    description='Single-frame camera-relative terrain analysis and traversability grid for the Wiz stack',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_node = wiz_terrain.terrain_node:main',
        ],
    },
)

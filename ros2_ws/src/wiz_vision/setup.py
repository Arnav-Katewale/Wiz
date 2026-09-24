from setuptools import find_packages, setup

package_name = 'wiz_vision'

setup(
    name=package_name,
    version='0.3.0',
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
    description='Camera capture and computer-vision processing nodes for the Wiz stack',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = wiz_vision.camera_node:main',
            'vision_node = wiz_vision.vision_node:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'camera_processor'

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
    maintainer='elena',
    maintainer_email='elena11j@protonmail.ch',
    description='Dummy test package to learn ROS and to gather images from sensor for further processing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = camera_processor.camera_processor:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'model_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='elena',
    maintainer_email='elena11j@protonmail.ch',
    description='Handle execution of computer vision model and feed into perception system.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_runner = model_runner.model_runner:main',
            'talker = model_runner.publisher_member_function:main',
            'listener = model_runner.subscriber_member_function:main',
        ],
    },
)

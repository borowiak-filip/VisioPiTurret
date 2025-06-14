from setuptools import find_packages, setup

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gpiozero'],
    zip_safe=True,
    maintainer='filip_borowiak',
    maintainer_email='filip.borowiak.info@icloud.com',
    description='Package responsible for controlling servos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller_node = servo_controller.servo_controller_node:main',
        ],
    },
)

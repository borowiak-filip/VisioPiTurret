from setuptools import find_packages, setup

package_name = 'yolo_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='filip_borowiak',
    maintainer_email='filip.borowiak.info@icloud.com',
    description='Package allowing to detect, and calculate servo adjustments for object tracking.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_tracker = yolo_tracker.yolo_tracker:main'
        ],
    },
)

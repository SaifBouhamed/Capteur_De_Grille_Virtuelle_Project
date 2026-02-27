from setuptools import find_packages, setup

package_name = 'ia_turtlebot_vision'

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
    maintainer='laurine',
    maintainer_email='laurinecollindufresne@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'person_detector_node = ia_turtlebot_vision.person_detector_node:main',
        'webcam_publisher_node = ia_turtlebot_vision.webcam_publisher_node:main',
        'gesture_detector_node = ia_turtlebot_vision.gesture_detector_node:main',
        ],
    },
)

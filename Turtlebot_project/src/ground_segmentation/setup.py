from setuptools import setup
import os
from glob import glob

package_name = 'ground_segmentation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kamel',
    maintainer_email='kamel@todo.todo',
    description='Package corrected',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_grid_node = ground_segmentation.interactive_grid_node:main',
            'grid_projection_node = ground_segmentation.grid_projection_node:main',
            'activator_node = ground_segmentation.activator:main',
            'pointcloud_translator = ground_segmentation.pointcloud_translator:main',
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'milestone2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wester',
    maintainer_email='wester@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = milestone2.safety_node:main',
            "wall_follow_node = milestone2.wall_follow_node:main",
            "gap_follow_node = milestone2.gap_follow_node:main"
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'handson_planning'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Praveen Kumar Murali',
    maintainer_email='praveen.kumarmurali@udg.edu',
    description='Phase 1 — Mapping + RRT* + DWA',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapping_node       = handson_planning.mapping_node:main',
            'global_planner     = handson_planning.global_planner:main',
            'dwa_planner        = handson_planning.dwa_planner:main',
            'frontier_explorer  = handson_planning.frontier_explorer:main',
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'velmobil_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'obstacles', 'static_obstacles'), glob('obstacles/static_obstacles/*.sdf')),
        (os.path.join('share', package_name, 'obstacles', 'dynamic_obstacles'), glob('obstacles/dynamic_obstacles/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='milosz',
    maintainer_email='milosz.piecha05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest', 'numpy'
        ],
    },
    entry_points={
        'console_scripts': [
            'agent_training_drl = velmobil_agent.agent_training:main',
            'manage_simulation = velmobil_agent.manage_simulation:main'
        ],
    },
)

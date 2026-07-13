from setuptools import find_packages, setup

package_name = 'rl_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=['Kamil Troszczyński', 'Miłosz Piecha'],
    maintainer_email=['troszczynskikamil@outlook.com', 'milosz.piecha05@gmail.com'],
    description='RL Algorhitms for usage in velmobil simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'training_node = rl_tools.training_node:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'catch2024_teamr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hirobon',
    maintainer_email='hirobon1690@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image = catch2024_teamr.image:main',
            'lowlayer = catch2024_teamr.lowlayer:main',
            'mainarm_full_manual = catch2024_teamr.mainarm.full_manual:main',
            'mainarm_viewer = catch2024_teamr.mainarm.mainarm_viewer:main',
            'mainarm_semi_auto = catch2024_teamr.mainarm.semi_auto:main',
        ],
    },
)

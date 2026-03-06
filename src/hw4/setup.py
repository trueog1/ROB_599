from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hw4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[py]*'))),
        (os.path.join('share', package_name, 'config/rviz'), glob(os.path.join('config/rviz', '*.rviz*'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.world*'))),
        (os.path.join('share', package_name, 'world/include'), glob(os.path.join('world/include', '*.inc*'))),
        (os.path.join('share', package_name, 'world/bitmaps'), glob(os.path.join('world/bitmaps', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trueog1',
    maintainer_email='gehrkeo@oregonstate.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'mcl = hw4.test:main',
        'hw4controller = hw4.hw4controller:main'
        ],
    },
)

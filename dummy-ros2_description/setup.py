from setuptools import setup
import os
from glob import glob

package_name = 'dummy-ros2_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os. path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os. path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),  # 关键：这里必须是 *.stl
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muzixiaowen',
    maintainer_email='xin.li@switchpi.com',
    description='The dummy-ros2_description package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

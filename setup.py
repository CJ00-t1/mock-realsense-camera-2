from setuptools import setup
from glob import glob

package_name = 'mock_realsense_camera_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/config', ['config/realsense_camera_info.yaml']),
        ('share/' + package_name + '/data/rgb', glob('data/rgb/*.png')),
        ('share/' + package_name + '/data/depth', glob('data/depth/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafe',
    maintainer_email='rafemurr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_realsense_publisher = mock_realsense_camera_2.mock_realsense_publisher:main',
        ],
    },
)

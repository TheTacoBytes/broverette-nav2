from setuptools import find_packages, setup
import glob

package_name = 'broverette_nav2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        # Include all .yaml files in the config/oldtest directory
        # ('share/' + package_name + '/config', glob.glob('config/oldtest/*.yaml')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        # Include specific map files
        ('share/' + package_name + '/maps', ['maps/map.yaml', 'maps/map.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beto',
    maintainer_email='thetacobytes@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

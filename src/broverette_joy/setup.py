from setuptools import find_packages, setup

package_name = 'broverette_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/controller_launch.py']),
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
        'ds4_joy = broverette_joy.ps4_controller_node:main'
        ],
    },
)

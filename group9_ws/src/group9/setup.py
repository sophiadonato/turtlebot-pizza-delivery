from setuptools import find_packages, setup

package_name = 'group9'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laptop10',
    maintainer_email='SDonato@student.uiege.be',
    description='Group 9 project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'delivery-v1 = group9.delivery_v1:main',
            'fake = group9.fake:main',
            'delivery = group9.delivery:main',
            'robot_pose = group9.robot_pose:main'
        ],
    },
)

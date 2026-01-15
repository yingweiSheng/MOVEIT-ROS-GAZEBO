from setuptools import find_packages, setup

package_name = 'receive_message_ros'

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
    maintainer='syw',
    maintainer_email='hananon125@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rec_pub_from_main = receive_message_ros.rec_pub_from_main:main',
            'rec_pub_to_main = receive_message_ros.rec_pub_to_main:main'
        ],
    },
)

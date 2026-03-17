import os

from setuptools import find_packages, setup


def package_data_files(package_name, directory):
    data_files = []
    for root, _, files in os.walk(directory):
        if not files:
            continue
        install_dir = os.path.join("share", package_name, root)
        file_paths = [os.path.join(root, file_name) for file_name in files]
        data_files.append((install_dir, file_paths))
    return data_files

package_name = 'mushr_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *package_data_files(package_name, 'launch'),
        *package_data_files(package_name, 'config'),
        *package_data_files(package_name, 'maps'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg',
    maintainer_email='sg@todo.todo',
    description='MuSHR base nodes and launch files',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_teleop = mushr_base.joy_teleop:main',
            'nav_msg_converter = mushr_base.nav_msg_converter:main',
            'racecar_state = mushr_base.racecar_state:main',
        ],
    },
)

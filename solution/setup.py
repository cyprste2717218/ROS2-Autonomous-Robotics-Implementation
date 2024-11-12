# Based on https://answers.ros.org/question/397319/how-to-copy-folders-with-subfolders-to-package-installation-path/
import os
from setuptools import find_packages, setup

package_name = 'Autonomous Robotic Object Retrieval Solution (AURO-H Module Coursework)'

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['models/', 'launch/', 'worlds/', 'rviz/', 'config/', 'params/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Y3904914',
    maintainer_email='"user@todo.todo"',
    description='Package which implements an Autonomous Robotic Solution for object retrieval and distribution within a simulated environment',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = solution.robot_controller:main',
            'data_logger = solution.data_logger:main',
        ],
    },
)


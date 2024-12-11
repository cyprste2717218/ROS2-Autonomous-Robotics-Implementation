from setuptools import find_packages, setup

package_name = 'wk1_practical'

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
    maintainer='Stephen Ingham',
    maintainer_email='stephenmichael833@gmail.com',
    description='My package for wk1 practical',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'turtle_topic_echo = wk1_practical.topic_echo:main',
        	'turtle_do_twist = wk1_practical.turtle_twist:main',
        	'turtle_do_zig_zag = wk1_practical.turtle_zig_zag:main'
        ],
    },
)

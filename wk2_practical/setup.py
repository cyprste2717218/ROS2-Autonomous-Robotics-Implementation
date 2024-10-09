from setuptools import find_packages, setup

package_name = 'wk2_practical'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/wk2_practical/launch', glob.glob(os.path.join('launch', '*_launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stephen Ingham',
    maintainer_email='stephenmichael833@gmail.com',
    description='Week2 Practical',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

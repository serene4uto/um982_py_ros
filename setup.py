from setuptools import find_packages, setup
from glob import glob

package_name = 'um982_py_ros'

setup(
    name=package_name,
    version='0.0.1',
    author='Ha Trung Nguyen',
    packages=find_packages(exclude=['test']),
    # packages=[package_name],
    # package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # add launch dir
        ('share/' + package_name + '/launch', glob('launch/*.*')),
        # add config dir
        ('share/' + package_name + '/config', glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ha Trung Nguyen',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='This is a package for UM982 GNSS receiver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "um982_node = src.um982_node:main",
        ],
    },
)

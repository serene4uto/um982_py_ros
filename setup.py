from setuptools import find_packages, setup

package_name = 'um982_py_ros'

setup(
    name=package_name,
    version='0.0.1',
    author='Ha Trung Nguyen',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add launch dir
        ('share/' + package_name + '/launch', ['launch/*']),
        # add config dir
        ('share/' + package_name + '/config', ['config/*']),
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

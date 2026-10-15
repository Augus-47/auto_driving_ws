from setuptools import find_packages, setup

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # 把根目录的 camera_pub_node.py 作为顶层模块安装，使入口点 camera_pub_node:main 可导入
    py_modules=['camera_pub_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='15218303+c305automatic-driving@user.noreply.gitee.com',
    description='Camera driver package for ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_pub_node = camera_pub_node:main',
        ],
    },
)

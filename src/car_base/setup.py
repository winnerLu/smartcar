from setuptools import find_packages, setup

package_name = 'car_base'

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
    maintainer='lsc',
    maintainer_email='lushici@ldrobot.com',
    description='STM32 底盘串口通信节点与验证工具',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_check = car_base.serial_check:main',
        ],
    },
)

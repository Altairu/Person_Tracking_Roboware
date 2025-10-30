from setuptools import find_packages, setup

package_name = 'Robowarepkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altair',
    maintainer_email='Altairu@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'Roboware_node = Robowarepkg.Roboware_node:main',
            'serial_send_node = Robowarepkg.serial_send_node:main',
            'serial_read_node = Robowarepkg.serial_read_node:main',
            'web_socket_node = Robowarepkg.web_socket_node:main',
            'RealSense_node = Robowarepkg.RealSense_node:main',
            'PID_node = Robowarepkg.PID_node:main',
            'FaceAnimation_node = Robowarepkg.FaceAnimation_node:main',
        ],
    },
)

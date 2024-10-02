from setuptools import find_packages, setup

package_name = 'Robowarepkg'

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
    maintainer='altair',
    maintainer_email='Altairu@github.comu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Robowarenode = Robowarepkg.Robowarenode:main',
        'serial_send_node = Robowarepkg.serial_send_node:main',
        'serial_read_node = Robowarepkg.serial_read_node:main',
        ],
    },
)

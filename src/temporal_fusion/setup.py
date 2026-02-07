from setuptools import find_packages, setup

package_name = 'temporal_fusion'

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
    maintainer='gurumanohar',
    maintainer_email='manoharguptabaratam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "temporal_measurement = temporal_fusion.measurement_node:main",
            "kalman_filter = temporal_fusion.kalman_tracker_node:main"
        ],
    },
)

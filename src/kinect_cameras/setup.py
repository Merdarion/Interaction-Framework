from setuptools import setup

package_name = 'kinect_cameras'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='merdarion',
    maintainer_email='Jens.Rueppel@gmx.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'kinect_publisher = kinect_cameras.kinect_publisher:main',
        ],
    },
)

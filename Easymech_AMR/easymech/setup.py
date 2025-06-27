from setuptools import find_packages, setup

package_name = 'easymech'

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
    maintainer='baveshs',
    maintainer_email='bavesh_s@ignitarium',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "driver = easymech.motor_controller:main",
            "sample_test = easymech.sample_control:main",
            "odom = easymech.odom_calc:main"
        ],
    },
)

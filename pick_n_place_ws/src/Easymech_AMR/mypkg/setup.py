from setuptools import find_packages, setup

package_name = 'mypkg'

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
            "py_node = mypkg.node1:main",
            "radio = mypkg.ex_publisher:main",
            "player = mypkg.ex_sub:main"
        ],
    },
)

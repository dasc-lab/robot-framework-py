from setuptools import setup

package_name = 'dasc_robot_py'

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
    maintainer='root',
    maintainer_email='devansh@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ex_rover_test = dasc_robot_py.ex_rover_test:main',
            'ex_drone_hover = dasc_robot_py.ex_drone_hover:main',
            'ex_rover_test_classes = dasc_robot_py.ex_rover_test_classes:main',
            'ex_drone_control_geometric = dasc_robot_py.ex_drone_control_geometric:main',
        ],
    },
)

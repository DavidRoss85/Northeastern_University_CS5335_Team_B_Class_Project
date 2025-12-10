from setuptools import find_packages, setup

package_name = 'node1pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/turtlebot_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='robo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node1=node1pkg.node1:main",
            # Action-based control nodes
            'executive_node=node1pkg.executive_node:main',
            'driving_node=node1pkg.driving_node:main',
            'odom_node=node1pkg.driving_node_odom:main',
            'driving_node_unbiased=node1pkg.driving_node_unbiased:main',
            'driving_node_biased=node1pkg.driving_node_withbiased:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'julia_turtlebot_circle_test'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "launch_julia_turtlebot = julia_turtlebot_circle_test.launch_julia_turtlebot:main",
            "launch_julia_turtlebot_v2 = julia_turtlebot_circle_test.launch_julia_turtlebot_v2:main",
        ],
    },
)

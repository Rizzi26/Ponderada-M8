from setuptools import find_packages, setup

package_name = 'pond_nicola'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'typer',
        'rclpy',
        ],
    zip_safe=True,
    maintainer='rizzi',
    maintainer_email='rizzi@todo.todo',
    description='Package for ROS navigation challenge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pond_nicola_cli = pond_nicola.main:app",
        ],
    },
)

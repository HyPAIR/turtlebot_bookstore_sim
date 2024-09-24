from setuptools import find_packages, setup

package_name = 'turtlebot_bookstore_sim'

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
    maintainer='Charlie Street',
    maintainer_email='me@charliestreet.net',
    description='A simulation of a Turtlebot in a bookstore. Used to evaluate refine-plan for CONVINCE.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

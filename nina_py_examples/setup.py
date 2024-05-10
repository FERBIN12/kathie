from setuptools import find_packages, setup

package_name = 'nina_py_examples'

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
    maintainer='ferbin',
    maintainer_email='fjferbin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_publisher = nina_py_examples.simple_publisher:main",
            "simple_subscriber = nina_py_examples.simple_subscriber:main",
            "simple_parameter = nina_py_examples.simple_parameter:main",
        ],
    },
)

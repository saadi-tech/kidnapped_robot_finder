from setuptools import find_packages, setup

package_name = 'global_localizer'
submodules = "mypackage/submodules"

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
    maintainer='saad',
    maintainer_email='name.surname@beko.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_localizer = global_localizer.find_robot:main'
        ],
    },
)

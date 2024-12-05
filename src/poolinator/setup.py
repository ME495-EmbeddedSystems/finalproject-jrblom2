from setuptools import find_packages, setup

package_name = 'poolinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            ['launch/image.launch.xml', 'launch/rs_launch.py'],
        ),
        (
            'share/' + package_name + '/config',
            ['config/default.rviz', 'config/tags.yaml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='annguyen',
    maintainer_email='annguyen2025@u.northwestern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridger = poolinator.bridger:main',
            'control = poolinator.control:main',
        ],
    },
)

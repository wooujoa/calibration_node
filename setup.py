from setuptools import find_packages, setup

package_name = 'calibration_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwg',
    maintainer_email='wjddnrud4487@naver.com',
    description='Transforms crop detection positions using a hand-eye matrix.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'crop_transformer_node = calibration_node.calibration:main',
        ],
    },
)

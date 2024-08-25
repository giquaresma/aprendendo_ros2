from setuptools import find_packages, setup

package_name = 'pacotedagi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launchdagi.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giovanna Quaresma',
    maintainer_email='giovanna.quaresma@hotmail.com',
    description='Tentando aprender a criar um lauch com dois nós',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodagi =  pacotedagi.nodagi:main',
            'nodagi2 =  pacotedagi.nodagi2:main',
        ],
    },
)

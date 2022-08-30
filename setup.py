from setuptools import setup

package_name = 'ezrassor_autonomous_control'

launch_files = [
    'launch/test_controls_launch.py'
]

data_files = []
data_files.append(('share/ament_index/resource_index/packages',['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', launch_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robert',
    maintainer_email='robert.forristall@knights.ucf.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_control = ezrassor_autonomous_control.test_control:main'
        ],
    },
)

from setuptools import setup
package_name = 'plevelai_control'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/plevelai_mock_pipeline.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='PlevelAI control pre-hw',
    entry_points={
        'console_scripts': [
            'targeting_node = plevelai_control.targeting_node:main',
            'firing_node = plevelai_control.firing_node:main',
            'mock_laser = plevelai_control.mock_laser:main',
            'serial_sim = plevelai_control.serial_sim:main',
        ],
    },
)

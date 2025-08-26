from setuptools import setup
package_name = 'plevelai_vision'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/vision_mock.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='PlevelAI vision pre-hw',
    entry_points={
        'console_scripts': [
            'detector_node = plevelai_vision.detector_node:main',
            'mock_camera = plevelai_vision.mock_camera:main',
        ],
    },
)

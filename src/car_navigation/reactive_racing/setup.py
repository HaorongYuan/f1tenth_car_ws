from setuptools import setup, find_packages

package_name = 'reactive_racing'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/reactive_racing.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Reactive racing heuristic navigation node for F1TENTH.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'reactive_racing_node = reactive_racing.reactive_racing_node:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'ros2_snake'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/assets',
        ['ros2_snake/assets/background.jpg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='anton@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'snake_game = ros2_snake.snake_game_node:main',
            'renderer = ros2_snake.renderer_node:main',
            'controller = ros2_snake.snake_controller_node:main'
        ],
    },
)

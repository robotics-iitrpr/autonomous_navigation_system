from setuptools import find_packages, setup

package_name = 'encoder_publish'

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
    maintainer='ans',
    maintainer_email='ans@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_publish = encoder_publish.publish:main',
            'calc_rpm = encoder_publish.calc_rpm:main',
            'rotate = encoder_publish.rotate:main',
            'forward = encoder_publish.forward:main',
            'backward = encoder_publish.backward:main',
            'omni_teleop = encoder_publish.omni_teleop:main',
            'omni_nav2_bridge = encoder_publish.omni_nav2_bridge:main',
            'mini_nav = encoder_publish.mini_nav:main'
        ],
    },
)

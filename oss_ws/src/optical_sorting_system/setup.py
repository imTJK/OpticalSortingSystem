from setuptools import setup

package_name = 'optical_sorting_system'
raspi = 'optical_sorting_system/raspi'
pc = 'optical_sorting_system/pc'
utilities = "optical_sorting_system/raspi/utilities"


setup(
    name=package_name,
    version='0.9.5',
    packages=[package_name, pc, raspi, utilities],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'classifier = optical_sorting_system.pc.classifier_node:main',
            'image_handler = optical_sorting_system.raspi.image_handler_node:main'
        ],
    },
)

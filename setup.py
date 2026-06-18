from setuptools import setup

package_name = 'eco_cleanbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claire',
    maintainer_email='claire@todo.todo',
    description='Eco CleanBot trash detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trash_detector = eco_cleanbot.scripts.trash_detector:main',
        ],
    },
)

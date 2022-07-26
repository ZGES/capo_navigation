from setuptools import setup

package_name = 'metrics_collector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZGES',
    maintainer_email='piotrpassternak@gmail.com',
    description='Module for collecting metrics from nav2 stack',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metric = metrics_collector.metric_collectors_node:main',
        ],
    },
)

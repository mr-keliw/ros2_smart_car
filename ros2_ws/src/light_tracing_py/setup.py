from setuptools import setup

package_name = 'light_tracing_py'

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
    maintainer='rosadmin',
    maintainer_email='rosadmin@todo.todo',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = light_tracing_py.light_tracing_server:main',
            'client = light_tracing_py.light_tracing_client:main',
        ],
    },
)

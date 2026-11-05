from glob import glob

from setuptools import setup

package_name = 'parc_referee'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PARC Organisers',
    maintainer_email='info@parcrobotics.org',
    description='Referee, timer and dashboard for the PARC 2026 Real World Phase.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'referee = parc_referee.referee_node:main',
            'example_team = parc_referee.example_team:main',
        ],
    },
)

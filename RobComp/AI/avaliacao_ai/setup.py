from setuptools import find_packages, setup

package_name = 'avaliacao_ai'

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
    maintainer='guedes',
    maintainer_email='guilhermegg5@al.insper.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'q1 = avaliacao_ai.q1:main',
            'laser = avaliacao_ai.laser:main',
            'odom = avaliacao_ai.odom:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'entregavel_7'

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
            'segue_linha_meu = entregavel_7.segue_linha_meu:main',
            'pega = entregavel_7.pega:main',
            'odom = entregavel_7.odom:main',
            'garra = entregavel_7.garra:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'entregavel_6'

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
            'mobilenet_detector = entregavel_6.mobilenet_detector:main',
            'module_aruco = entregavel_6.module_aruco:main',
            'module_net = entregavel_6.module_net:main',
            'aruco_detector = entregavel_6.aruco_detector:main',
            'segue_linha = entregavel_6.segue_linha:main',
            'aproxima = entregavel_6.aproxima:main',
            'filtro_cor = entregavel_6.filtro_cor:main'
        ],
    },
)

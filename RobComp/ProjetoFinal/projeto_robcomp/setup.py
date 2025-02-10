from setuptools import find_packages, setup

package_name = 'projeto_robcomp'

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
    maintainer='borg',
    maintainer_email='antoniocgsf@al.insper.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobilenet_detector = projeto_robcomp.mobilenet_detector:main',
            'goto = projeto_robcomp.goto:main',
            'missaoD = projeto_robcomp.missaoD:main',
        ],
    },
)

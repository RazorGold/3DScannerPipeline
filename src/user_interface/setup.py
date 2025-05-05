from setuptools import find_packages, setup

package_name = 'user_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'flask',
                      'flask-cors'],
    zip_safe=True,
    maintainer='dduboi',
    maintainer_email='dkdubois@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_node = user_interface.ui_node:main'
        ],
    },
)

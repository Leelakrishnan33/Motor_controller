from setuptools import setup

package_name = 'motor_controller'

setup(
    name='motor_controller',
    version='0.0.1',
    packages=['motor_controller'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Motor controller for an omnidirectional robot',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = motor_controller.motor_controller:main',
        ],
    },
)

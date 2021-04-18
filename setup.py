import os
from glob import glob
from setuptools import setup

package_name = 'slider_publisher'

setup(
    name=package_name,
    version='1.0.2',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        #(os.path.join('share', package_name, 'launch'), glob('*.launch')),
        # Include examples.
        (os.path.join('share', package_name, 'examples'), glob('examples/*'))
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Olivier Kermorgant',
    author_email='olivier.kermorgant@ec-nantes.fr',
    maintainer='Olivier Kermorgant',
    maintainer_email='olivier.kermorgant@ec-nantes.fr',
    keywords=['rqt'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: MIT',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='This packages proposes a slider-based publisher node similar to the joint_state_publisher, but that can publish any type of message.',
    license='MIT',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'slider_publisher = slider_publisher.slider_publisher:main'
        ],
    },
)

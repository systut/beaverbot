from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=['tsnd5_driver'],
    package_dir={'': ''},
)

setup(**setup_args)

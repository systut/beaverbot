from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=['beavorbot_localization'],
    package_dir={'': ''},
)

setup(**setup_args)

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['se3_controller', 'python_utils'],
    package_dir={'': 'python'},
)

setup(**d)

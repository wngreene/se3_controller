from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['se3_controller'],
    package_dir={'': 'python/se3_controller'},
)

setup(**d)

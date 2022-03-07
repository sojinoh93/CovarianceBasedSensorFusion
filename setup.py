from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['silbot3_localization'],
    package_dir={'': 'nodes'}
)
setup(**d)
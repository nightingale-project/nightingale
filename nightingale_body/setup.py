from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["nightingale_body"],
    package_dir={"": "src"},
)

setup(**setup_args)

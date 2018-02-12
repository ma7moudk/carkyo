from distutils.core import setup
from catkin_pkg.python_setup impory=t generate_distutils_setup

d= generate_distutils_setup(
    packages=['python'],
    package_dir={'':'src'}
   )
   
setup(**d)

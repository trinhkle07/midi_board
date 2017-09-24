from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['midi_board',
              'midi_board.adapters',
              'midi_board.controls'],
    package_dir={'': 'src'}
)

setup(**d)

from distutils.core import setup
from Cython.Build import cythonize

files = "constraint.pyx", "point.pyx", "cloth.pyx", "shapecloth.pyx", "circlecloth.pyx", "mouse.pyx", "tensioner.pyx"

for file in files:
	setup(
	    ext_modules = cythonize(file)
	)

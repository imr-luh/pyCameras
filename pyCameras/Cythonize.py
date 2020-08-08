from setuptools import setup, Extension
from Cython.Build import cythonize

# setup(ext_modules=cythonize("cython_hello_world.pyx", annotate=True))


ext_modules = cythonize([Extension("cython_ov2740", ["OV2740_cython.py"])], annotate=True)

setup(ext_modules=ext_modules)
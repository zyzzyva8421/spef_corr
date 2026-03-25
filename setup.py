from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os
import pybind11

# Get include directory
include_dir = pybind11.get_include()

# Required for Python 3.12+
if sys.version_info >= (3, 12):
    os.environ["CXX"] = "g++"

# Build extension
spef_core = Extension(
    'spef_core',
    sources=['src/wrapper.cpp', 'src/spef_core.cpp'],
    include_dirs=[include_dir],
    language='c++',
    extra_compile_args=['-O3', '-march=native', '-std=c++17'],
    extra_link_args=[],
)

setup(
    name='spef_core',
    version='1.0.0',
    description='C++ optimized SPEF parser and shortest path',
    ext_modules=[spef_core],
    cmdclass={'build_ext': build_ext},
    zip_safe=False,
)

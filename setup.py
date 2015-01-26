#!/usr/bin/env python

"""
setup.py file for SWIG smbus
"""

from distutils.core import setup, Extension


smbus_module = Extension('_smbus',
                           sources=['smbus_wrap.c', 'smbus.c'],
                           )

setup (name = 'smbus',
       version = '0.1',
       author      = "SWIG Docs",
       description = """Simple swig smbus from docs""",
       ext_modules = [smbus_module],
       py_modules = ["smbus"],
       )

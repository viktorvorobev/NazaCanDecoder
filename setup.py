from distutils.core import setup, Extension

module = Extension('NazaCanDecoder',
                   sources=['NazaCanDecoder.cpp'],
                   extra_compile_args=['-std=c++11'])

setup (name='NazaCanDecoder',
       version='1.0',
       description='Decoder for CAN messages from DJI Naza M v2',
       ext_modules=[module])
#!/usr/bin/env python
NAME = 'thorlabs_apt'
AUTHOR = 'Tobias Gehring'
AUTHOR_EMAIL = 'tobias.gehring@fysik.dtu.dk'
LICENSE = 'GPLv2'
URL = 'https://github.com/qpit/thorlabs_apt'
DOWNLOAD_URL = ''
DESCRIPTION = 'python wrapper for Thorlabs\' APT library'
LONG_DESCRIPTION = '''\
thorlabs_apt provides a wrapper for Thorlabs\' APT, a library to control different Thorlabs motors.
'''
CLASSIFIERS = """\
Development Status :: 4 - Beta
Intended Audience :: Science/Research
License :: OSI Approved :: GNU General Public License v2 (GPLv2)
Programming Language :: Python
Topic :: Scientific/Engineering
Topic :: Software Development :: Libraries
Topic :: System :: Hardware
Operating System :: Microsoft :: Windows
"""
PLATFORMS = ['Windows']
MAJOR               = 0
MINOR               = 1
ISRELEASED          = False
VERSION             = '%d.%d' % (MAJOR, MINOR)

if __name__=='__main__':

    from setuptools import setup

    setup(
        name = NAME,
        author = AUTHOR,
        author_email = AUTHOR_EMAIL,
        license = LICENSE,
        url = URL,
        download_url = DOWNLOAD_URL,
        version = VERSION,
        description = DESCRIPTION,
        long_description = LONG_DESCRIPTION,
        classifiers = filter(None, CLASSIFIERS.split('\n')),
        platforms = PLATFORMS,
        packages = ['thorlabs_apt'])

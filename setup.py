import re
from distutils.core import setup

from setuptools import find_packages


def get_property(prop, project):
    with open(project + '/__init__.py', 'r') as f:
        result = re.search(r'{}\s*=\s*[\'"]([^\'"]*)[\'"]'.format(prop), f.read())
    return result.group(1)

project_name = 'pyCameras'

setup(
    name=project_name,
    version=get_property('__version__', project_name),
    packages=find_packages(),
    url='https://gitlab.imr.uni-hannover.de/imr/pyCameras',
    license='',
    author='Tim Betker, Niklas Kröger',
    author_email='tim.betker@imr.uni-hannover.de',
    description='Collection of Python camera interfaces for use in the IMR'
)

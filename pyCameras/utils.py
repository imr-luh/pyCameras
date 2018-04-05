#!/usr/bin/env python3
__author__ = 'Niklas Kroeger'
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import logging
import pkgutil

import pyCameras


def dynamic_import_class(name):
    """
    Dynamically import a class defined by a string

    Parameters
    ----------
    name : str
        String describing the class that should be imported like a normal
        python import.

    Returns
    -------
    class : Object
        Class that is imported by the given 'name'

    Notes
    -----
    Taken from
    https://stackoverflow.com/questions/547829/how-to-dynamically-load-a-python-class

    Examples
    --------
    dynamic_import_class(pyCameras.cameraUSB.CameraUSB) will return CameraUSB
    """
    # split import path into package and desired class, import package, and
    # return class from package
    mod, cls = name.rsplit('.', 1)
    mod = __import__(mod, fromlist=[cls])
    return getattr(mod, cls)


def listCameraImplementations():
    package = pyCameras
    classes = []
    for importer, modname, ispkg in pkgutil.walk_packages(
            path=package.__path__,
            prefix=package.__name__ + '.',
            onerror=lambda x: None):
        try:
            classes.append(dynamic_import_class(modname + '.Camera'))
        except ImportError as e:
            logging.warning('Failed to import {modname}: {e}'
                            ''.format(modname=modname,
                                      e=e))
        except AttributeError:
            # there is no class named Camera in that module
            pass
    return classes


def getAllAvailableDevices():
    return {implementation.__module__: implementation.listDevices() for
            implementation in listCameraImplementations()}

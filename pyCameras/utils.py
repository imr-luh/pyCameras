#!/usr/bin/env python3
__author__ = 'Niklas Kroeger'
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import logging
import pkgutil
try:
    from itertools import zip_longest
except ImportError:
    # python2 uses a different name
    from itertools import izip_longest as zip_longest

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
    """
    Find all classes with the name `Camera` in the pyCameras project

    Returns
    -------
    classes : list
        List of all class implementations with the name `Camera`
    """
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


def getAllDevices():
    """
    Find all available camera devices that are reachable right now

    Returns
    -------
    devices : dict
        Dict of all available camera devices that are reachable at the moment.
        The devices are separated by their respective class implementation.
        The implementation.__module__ string is the dict key under which a list
        of the respective devices is returned.
    """
    return {implementation.__module__: implementation.listDevices() for
            implementation in listCameraImplementations()}


def grouper(iterable, n, fillvalue=None):
    """
    Collect data into fixed-length chunks or blocks

    example: grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx"

    :param iterable: some iterable from which the groups should be returned
    :param n: number of elements in the groups
    :param fillvalue: object that should be inserted if the iterable can't be
        split up into a full number of groups
    :return: iterable of iterable groups with length n
    """
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)


class SettingsHandler(object):
    """
    Helper class to generate a simple settings interface for classes.

    This class implements two functions that can be used to register and call
    functions that change some settings values. To register a new setting the
    registerFeature function should be used. To change the value of a setting
    the setFeature function should be called. For details on the use of these
    functions see their corresponding docstring.
    """
    def __init__(self):
        super(SettingsHandler, self).__init__()

        # only add the logger if no other parent class took care of it yet
        if not hasattr(self, 'logger'):
            self.logger = logging.getLogger(__name__)

        self.features = {}

    def listFeatures(self):
        """
        Helper function to return the settings dict
        """
        return list(self.features.keys())

    def registerFeature(self, key, callback):
        """
        Register a setFeature function by defining the corresponding key and
        callback function

        Parameters
        ----------
        key : str
            Key describing the feature that should be registered

        callback : function
            Function that should be called to set the corresponding feature

        Notes
        -----
        To prevent typos in capitalization of keys all feature registrations
        are done with key.lower(). This is already incorporated in the
        settingsHandler.setFeature() by searching the self.features dict for
        key.lower().
        """
        self.features[key.lower()] = callback

    def setFeature(self, *args, **kwargs):
        """
        Update a setting (described by 'key') to a new value

        This function expects features in the form of 'key' - 'value'. The key
        describes what feature should be changed and the value parameter is
        passed to the corresponding function implementation. The key and its
        corresponding function have to be registered in the self.features
        dictionary. To do this use self.registerFeature.

        Several different ways of passing 'key' - 'value' pairs are allowed.

        For the simplest usecase of updating one setting simply pass 'key' and
        'value' in the correct order or as keyword arguments.
        If mutliple settings should be updated with a single call a number of
        'key' - 'value' pairs can be passed as list or tuple in the order
        [key1, value1, key2, value2, ... , keyN, valueN].
        Alternatively a dict can be passed where the keys of the dict math the
        feature keys and the associated value corresponds to the desired value
        e.g.
        {'resolutionX': 640, 'resolutionY': 480}

        If only a single string is passed this function assumes this is the
        path to a configuration file that should be parsed and loaded. NOTE:
        NOT YET IMPLEMENTED!

        Parameters
        ----------
        key : str
            key describing the function as registered via self.registerFeature

        value : object
            Parameters shat should be passed on to the corresponding function
            implementation

        Notes
        -----
        To prevent capitalization typos all feature registrations are done with
        key.lower() (see settingsHandler.registerFeature()). This means that
        feature lookups are also done with key.lower(). This has to be
        considered if this function is overloaded.
        """
        if len(args) == 1:
            if isinstance(args[0], dict):
                settings = args[0]
                for key in settings.keys():
                    self.setFeature(key=key, value=settings[key])
            elif isinstance(args[0], (list, tuple)):
                # assume the settings are ordered as ['key', value]
                for (key, value) in grouper(args[0], 2):
                    self.setFeature(key=key, value=value)
            elif isinstance(args[0], str):
                # This might still be a single key with the value given as a
                # kwarg. So check if a single kwarg with key 'value' exists
                if len(kwargs) >= 1 and 'value' in kwargs.keys():
                    self.setFeature(key=args[0], value=kwargs['value'])
                else:
                    # assume this is the path to a settings file we should
                    # parse
                    # TODO: implement file parsing
                    pass
        elif len(args) >= 2:
            # assume the arguments were passed in order ['key', 'value']
            # there may be multiple key value pairs so try to parse all of them
            for (key, value) in grouper(args, 2):
                self.setFeature(key=key, value=value)

        if all(k in kwargs.keys() for k in ('key', 'value')):
            try:
                self.logger.debug("Setting key: {key} with value: {value}"
                                  "".format(key=kwargs['key'],
                                            value=kwargs['value']))
                self.features[kwargs['key'].lower()](kwargs['value'])
            except KeyError:
                raise NotImplementedError('The desired key \'{key}\' has no '
                                          'registered implementation. Desired '
                                          'value: \'{value}\''
                                          ''.format(key=kwargs['key'],
                                                    value=kwargs['value']))
            except Exception as e:
                self.logger.exception('Failed to set \'{key}\' to '
                                      '\'{value}\', {e}'
                                      ''.format(key=kwargs['key'],
                                                value=kwargs['value'],
                                                e=e))

    def getFeatures(self):
        """
        Returns the dictionary of registered setFunction implementations
        """
        return self.features

    def getFeature(self, key):
        """
        Get the current value for the feature defined by key

        Parameters
        ----------
        key : str
            String defining the feature

        Returns
        -------
        value : str, int, float, object
            Value of the desired feature, '<NOT READABLE>' if the value could
            not be read

        Notes
        -----
        This function only works as intended if the registered callback
        function for the given key accepts a call with no arguments and returns
        the correct setting value. An example of this can be found at the
        bottom of this file.
        """
        try:
            value = self.features[key.lower()]()
        except Exception:
            value = '<NOT READABLE>'
        return value


if __name__ == '__main__':
    ########################################################
    # example for using the SettingsHandler class:
    ########################################################

    # define the class that should inherit from the settingsHandler
    class SomeClass(SettingsHandler):
        def __init__(self):
            """
            If you want to inherit from another class it is possible to give
            multiple parent classes: class SomeClass(ParentA, SettingsHandler)
            """
            # make sure all parent classes are properly initialized
            super(SomeClass, self).__init__()

            # Now register a callback function for some setting
            self.registerFeature('someSetting', self.setSomething)

            # start value for our variable
            self.someSetting = 0

        def setSomething(self, value=None):
            # allow function calls with no arguments and return current value
            # to enable use of self.getFeature.
            if value is not None:
                self.someSetting = value
            return self.someSetting

    # Using the registered callback function
    test_instance = SomeClass()

    # print value before we change it
    print(test_instance.someSetting)

    test_instance.setFeature('someSetting', 10)

    # print value after changing it
    print(test_instance.getFeature('someSetting'))

    ########################################################
    # example for getting all available cameras
    ########################################################
    print(getAllDevices())

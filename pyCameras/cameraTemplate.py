#!/usr/bin/env python
"""
Template class from which new camera implementations should be inherited.

Notes on registerFeature, setFeature and getFeature:
To allow a modular template class while keeping a generic way of getting and
settings camera features, a callback registration strategy is used. This allows
all feature setting operations to be done with the same function call and even
enables the user to change multiple settings with a single function call or
load a settings file.
All camera settings (from now on called features) should be changed by the
setFeature() function. This function expects two parameters, a key and a value.
The key describes which feature should be set and the value is the new value to
which the feature should be set. This value must be of the correct type
expected by the function implementation used to change the feature.
Besides calling the function as setFeature(key, value), it is also possible to
change multiple features at once by chaining them behind each other like:
setFeature(key1, value1, key2, value2, ..., keyN, valueN) or passing the
function a list or tuple of key - value pairs e.g.
setFeature([key1, value1, key2, value2]).
Additionally it is possible to pass the function a dict of features that should
be set e.g. setFeature({key1: value1, key2: value2, ..., keyN, valueN}).
If only a single string is passed to the setFeature function, it is assumed to
be the path to a configuration file that should be parsed and applied.
TODO: This has not yet been implemented
"""
__author__ = "Ruediger Beermann, Niklas Kroeger"
__credits__ = ["Ruediger Beermann", "Niklas Kroeger"]
__maintainer__ = "Niklas Kroeger"
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import abc
import logging

from imrpy.misc.iterators import grouper

LOGGING_LEVEL = None


class CameraControllerTemplate(object):
    """
    Template class for spectrometer controllers to inherit from if they are
    necessary to use the camera. The controller for camera devices should
    be opened and closed from the Camera.openController and
    Camera.closeController static methods. If the controller is only used to
    detect available camera devices, that logic could also be easily
    implemented in Camera.listDevices().
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.device_handles = []

    def listDevices(self):
        """
        Returns a list of available devices. One of these entries should be
        used as parameter for self.getDevice to open the corresponding device

        Returns
        -------
        device_handles : list
            List of available capture devices
        """
        self.updateDeviceHandles()
        return self.device_handles

    def updateDeviceHandles(self):
        """
        Update the list of available device handles
        """
        raise NotImplementedError

    def getDevice(self, device_handle):
        raise NotImplementedError

    def closeController(self):
        raise NotImplementedError

    def __del__(self):
        self.logger.debug('Deleting cameracontroller {self}'
                          ''.format(self=self))
        self.closeController()

    def __repr__(self):
        return "<CameraController Template: OVERLOAD THIS FUNCTION>"


class CameraTemplate(object):
    """
    Template class for camera objects to inherit from. The following methods
    have to be implemented to be able to instantiate the camera object
    (decorated with @abc.abstractmethod):
        - CameraTemplate.listDevices() (static method)
        - self.openDevice()
        - self.closeDevice()
        - self.getImage()
    """
    def __init__(self, device_handle):
        """
        Template class for camera objects. This is only meant to define the
        interface. You probably want a real camera implementation that is
        inherited from this class.

        Parameters
        ----------
        device_handle : object
            Some handle that identifies the camera and is used to open the
            device connection
        """
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.device_handle = device_handle  # Use this to identify and open the
                                            # device
        self.device = None  # Use this variable to store the device itself
        self.features = {}
        self.registerSharedFeatures()

    @staticmethod
    def openController():
        """
        Open the camera controller and prepare everything to make cameras
        accessible
        """
        raise NotImplementedError

    @staticmethod
    def closeController():
        """
        Close the camera controller
        """
        raise NotImplementedError

    @staticmethod
    @abc.abstractmethod
    def listDevices():
        """
        List all available camera devices correspponding to the class type
        """
        raise NotImplementedError

    @abc.abstractmethod
    def openDevice(self):
        """
        Open the device by using self.device_handle and store the device in
        self.device
        """
        raise NotImplementedError

    @abc.abstractmethod
    def closeDevice(self):
        """
        Close the connection to the device and reset self.device to None.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def getImage(self, *args, **kwargs):
        """
        Return a numpy array containing an image

        *args and **kwargs are optional parameters that may be ignored!
        """
        raise NotImplementedError

    def getImages(self, num):
        """
        Blocking function that waits for num images to be recorded and returns
        an iterable of numpy arrays corresponding to images. Recording of
        images is done according to the currently set trigger mode!

        If a time sensitive image acquisition task is done consider using the
        separate self.prepareRecording(num) and self.record() functions to
        achieve the same result.

        Parameters
        ----------
        num : int
            number of images to return.
        """
        self.prepareRecording(num=num)
        return self.record()

    def prepareRecording(self, num):
        """
        Prepare the camera to recorded the given number of images by setting
        up a frame buffer or doing other camera specific actions. The actual
        recording is done in self.record() and should be blocking until the
        desired number of images is recorded.

        Parameters
        ----------
        num : int
            Number of images that should be recorded
        """
        raise NotImplementedError

    def record(self):
        """
        Blocking image acquisition of a previously defined number of images
        (see prepareRecording).

        Returns
        -------
        imgs : list
            List of numpy arrays containing the recorded images
        """
        raise NotImplementedError

    def grabStart(self):
        """
        Start grabbing images in a non-blocking way and store those images in
        an internal variable

        See Also
        --------
        self.grabStop()
        """
        raise NotImplementedError

    def grabStop(self):
        """
        Stop grabbing images and return the images that have been recorded

        See Also
        --------
        self.grabStart()
        """
        raise NotImplementedError

    def listFeatures(self):
        """
        Helper function to return the properties dict
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
        CameraTemplate.setFeature() by searching the self.features dict for
        key.lower().
        """
        self.features[key.lower()] = callback

    def registerSharedFeatures(self):
        """
        Registration of shared features that should be the same for all camera
        implementations. E.g. ExposureMicrons, Resolution, Gain, Format and
        TriggerMode
        """
        self.logger.debug('Registering shared camera features')

        self.registerFeature('ExposureMicrons', self.setExposureMicrons)
        self.registerFeature('ExposureTime', self.setExposureMicrons)
        self.registerFeature('Exposure', self.setExposureMicrons)

        self.registerFeature('Resolution', self.setResolution)

        self.registerFeature('Gain', self.setGain)

        self.registerFeature('Format', self.setFormat)

        self.registerFeature('TriggerMode', self.setTriggerMode)
        self.registerFeature('Trigger', self.setTriggerMode)

    def setFeature(self, *args, **kwargs):
        """
        Update a camera setting (described by 'key') to a new value

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
        key.lower() (see CameraTemplate.registerFeature()). This means that
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
                    # assume this is the path to a settings file we should parse
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
                                  "".format(key=kwargs['key'], value=kwargs['value']))
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

    def getFeature(self, key):
        """
        Return the current value of 'key'

        Parameters
        ----------
        key : str
            String describing the feature value to return
        """
        raise NotImplementedError

    def getFeatures(self):
        """
        Returns the dictionary of registered setFunction implementations
        """
        return self.features

    def setExposureMicrons(self, microns=None):
        """
        Set the exposure time to the given value in microseconds or read the
        current value by passing None

        Parameters
        ----------
        microns : int
            Desired exposure time in microseconds that should be set, or None
            to read the current exposure time

        Returns
        -------
        microns : int
            The exposure time in microseconds after applying the passed value
        """
        raise NotImplementedError

    def setResolution(self, resolution=None):
        """
        Set the resolution of the camera to the given values in pixels or read
        the current resolution by passing None

        Parameters
        ----------
        resolution : tuple
            Desired camera resolution in the form (width, height), or None to
            read the current resolution

        Returns
        -------
        resolution : tuple
            The set camera resolution after applying the passed value
        """
        raise NotImplementedError

    def setGain(self, gain=None):
        """
        Set the gain of the camera to the given value or read the current value
        by passing None

        Parameters
        ----------
        gain : int
            Desired gain value to be set, or None to read the current gain
            value

        Returns
        -------
        gain : int
            The gain value after applying the passed value
        """
        raise NotImplementedError

    def setFormat(self, format=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        format : str
            String describing the desired image format (e.g. "mono8"), or None
            to read the current image format

        Returns
        -------
        format : str
            The image format after applying the passed value
        """
        raise NotImplementedError

    def setTriggerMode(self, mode=None):
        """
        Set the trigger mode of the camera to either "in", "out" or "off", or
        read the current trigger setting ba passing None

        Parameters
        ----------
        mode : str
            The desired trigger mode. "in" means the camera receives a trigger
            signal, "out" means the camera sends a trigger signal, "off"" means
            the camera does not react to triggers. To read the current trigger
            setting pass None

        Returns
        -------
        mode : str
            The trigger mode after applying the passed value
        """
        raise NotImplementedError

    def __del__(self):
        if self.device is not None:
            self.closeDevice()

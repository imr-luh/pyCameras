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

from pyCameras.utils import SettingsHandler

LOGGING_LEVEL = None


class ControllerTemplate(object):
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


class CameraTemplate(SettingsHandler):
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
        super(CameraTemplate, self).__init__()
        self.device_handle = device_handle  # Use this to identify and open the device
        self.device = None  # Use this variable to store the device itself
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
        Each entry of this list can be used as an argument for this class constructor
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
    def isOpen(self):
        """
        Check if the device for this instance is currently open and ready to
        communicate

        Returns
        -------
        bool
            True if the camera connection is open, False if it is not
        """
        raise NotImplementedError

    def isClosed(self):
        """
        Inverse of self.isOpen(). Checks if the device connection is currently
        closed

        Returns
        -------
        bool
            True if the camera connection is closed, False if the connection is
            opened
        """
        return not (self.isOpen())

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

        self.registerFeature('Format', self.setPixelFormat)
        self.registerFeature('PixelFormat', self.setPixelFormat)
        self.registerFeature('PixelType', self.setPixelFormat)

        self.registerFeature('TriggerMode', self.setTriggerMode)
        self.registerFeature('Trigger', self.setTriggerMode)

    @property
    def exposure(self):
        return self.setExposureMicrons()

    @exposure.setter
    def exposure(self, microns):
        self.setExposureMicrons(microns)

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

    def autoExposure(self):
        """
        Automatically sets the exposure time of the camera ONCE.
        Old exposure setting is lost during the process!

        Returns
        -------
        exposure : int
            The exposure time in microseconds after auto exposure
        """
        raise NotImplementedError

    @property
    def resolution(self):
        return self.setResolution()

    @resolution.setter
    def resolution(self, resolution):
        self.setResolution(resolution)

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

    @property
    def gain(self):
        """
        Implement get function for gain here.

        Returns
        -------
        gain : int
        """
        return self.setGain()

    @gain.setter
    def gain(self, gain):
        """
        Implement set function for gain here.

        Parameters
        -------
        gain : int
        """
        self.setGain(gain)

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

    @property
    def pixelFormat(self):
        return self.setPixelFormat()

    @pixelFormat.setter
    def pixelFormat(self, fmt):
        self.setPixelFormat(fmt)

    def setPixelFormat(self, fmt=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        fmt : str
            String describing the desired image format (e.g. "mono8"), or None
            to read the current image format

        Returns
        -------
        fmt : str
            The image format after applying the passed value
        """
        raise NotImplementedError

    @property
    def triggerMode(self):
        return self.setTriggerMode()

    @triggerMode.setter
    def triggerMode(self, mode):
        self.setTriggerMode(mode)

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

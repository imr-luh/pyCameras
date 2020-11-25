#!/usr/bin/env python3
__author__ = 'Niklas Kroeger'
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import logging
import re

import pypylon.pylon as pylon

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = None


class Controller(ControllerTemplate):
    """
    Camera Controller for Basler cameras based on pypylon
    """
    def __init__(self):
        """
        Camera Controller for Basler camera devices. This implementation uses
        pypylon as backend.
        """
        super(Controller, self).__init__()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.logger.debug('Starting Basler Camera Controller')
        self._factory = pylon.TlFactory.GetInstance()
        self.device_handles = []

        # because the defined __repr__ method for the pylon.DeviceInfo class is
        # not very human readable we overwrite it here with the supplied
        # GetFriendlyName method
        pylon.DeviceInfo.__repr__ = lambda x: x.GetFriendlyName()

    def updateDeviceHandles(self):
        """
        Refresh the list of available devices
        """
        self.logger.debug('Searching for Basler camera devices')
        self.device_handles = list(self._factory.EnumerateDevices())
        self.logger.debug('Found {num} Basler camera devices: {devices}'
                          ''.format(num=len(self.device_handles),
                                    devices=self.device_handles))

    def getDevice(self, device_handle):
        """
        Return the corresponding camera object for the given device handle

        Parameters
        ----------
        device_handle : pypylon.cython.factory.DeviceInfo
            Device handle as returned by self.listDevices()

        Returns
        -------
        cam : baslerCamera.Camera
            A camera object for Basler devices corresponding to the given
            device handle
        """
        self.logger.debug('Opening device {device_handle}'
                          ''.format(device_handle=device_handle))
        return Camera(device_handle=device_handle)

    def closeController(self):
        """
        Close the camera controller and do the necessary cleanup
        """
        for dev in self.device_handles:
            del dev

    def __repr__(self):
        return "<Basler Camera Controller>"


class Camera(CameraTemplate):
    """
    Basler Camera implementation based on pypylon
    """
    def __init__(self, device_handle):
        """
        Implementation of the basler camera device. This implementation uses the
        pypylon wrapper provided by basler.

        Parameters
        ----------
        device_handle : object
            Unique camera device handle to identify the camera
        """
        if isinstance(device_handle, pylon.DeviceInfo):
            self.device_handle = device_handle
        elif isinstance(device_handle, str):
            # Assume the string contains the 8 digit serial number of the cam
            serial = re.findall(r'\(?([0-9]{7,9})\)?',
                                device_handle)[0]
            # List available cameras and try to find one with matching serial
            for device in self.listDevices():
                if device.GetSerialNumber() == serial:
                    self.device_handle = device
        else:
            raise TypeError('device_handle should be of type '
                            'pypylon.cython.factory.DeviceInfo or subclassed '
                            'from it')
        super(Camera, self).__init__(self.device_handle)

        # Similar to the __repr__ issue in the controller class we need to make
        # the pylon swig interface use a nice human readable string for
        # __repr__ calls by patching the parent class
        pylon.InstantCamera.__repr__ = lambda x: x.GetDeviceInfo().GetFriendlyName()

        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self._expected_triggered_images = 0
        self._timeout = 2000
        self.registerFeatures()
        self.openDevice()

    def registerFeatures(self):
        """
        Function to hold all implemented feature registrations
        """
        self.logger.debug('Registering implemented camera specific features')
        self.registerFeature('TriggerSource', self._setTriggerSource)
        self.registerFeature('AcquisitionFrameRateAbs',
                             self._getAcquisitionFrameRateAbs)
        self.registerFeature('ImageWidth', self._getImageWidth)
        self.registerFeature('ImageHeight', self._getImageHeight)

    @staticmethod
    def listDevices():
        """
        List available Basler cameras

        Returns
        -------
        cams : list
            list of available Basler camera devices
        """
        return Controller().listDevices()

    def openDevice(self):
        """
        Open the device corresponding to self.device_handle and store the
        object in self.device
        """
        try:
            self.device = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.device_handle))
            self.device.Open()
            self.logger.debug('Opened camera device: {device}'
                              ''.format(device=self.device))
        except Exception as e:
            self.logger.exception('Failed to open the camera device: {e}'
                                  ''.format(e=e))

    def closeDevice(self):
        """
        Close the connection to the device
        """
        if self.device is not None:
            try:
                self.logger.debug('Closing camera device: {device}'
                                  ''.format(device=self.device))
                self.device.Close()
                del self.device
                self.device = None

            except Exception as e:
                self.logger.exception('Failed to close the camera device: '
                                      '{e}'.format(e=e))
        else:
            self.logger.info('No Device present.')

    def isOpen(self):
        if self.device is not None:
            return self.device.IsOpen()
        else:
            return False

    def getImage(self, *args, **kwargs):
        """
        Get an image from the camera

        *args and **kwargs are ignored parameters!

        Returns
        -------
        img : np.ndarray
            Current camera image
        """
        return self.device.GrabOne(self._timeout).Array

    def setExposureMicrons(self, microns=None):
        """
        Set the exposure time to the given value in microseconds or read the
        current value by passing None

        Parameters
        ----------
        microns : float
            Desired exposure time in microseconds that should be set, or None
            to read the current exposure time

        Returns
        -------
        microns : int
            The exposure time in microseconds after applying the passed value
        """
        if microns is not None:
            self.logger.debug('Setting <ExposureTimeRaw> to {microns}'
                              ''.format(microns=microns))
            self.device.ExposureTimeAbs.SetValue(microns)
        return self.device.ExposureTimeAbs.GetValue()

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
        if gain is not None:
            self.logger.debug('Setting <GainRaw> to {gain}'
                              ''.format(gain=gain))
            self.device.GainRaw.SetValue(gain)
        return self.device.GainRaw.GetValue()

    def setPixelFormat(self, fmt=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        fmt : str
            String describing the desired image format (e.g. "Mono8" or
            "Mono10"), or None to read the current image format

        Returns
        -------
        fmt : str
            The image format after applying the passed value
        """
        if fmt is not None:
            try:
                self.device.properties['PixelFormat'] = fmt
            except Exception as e:
                self.logger.exception(e)
        return self.device.properties['PixelFormat']

    def _setTriggerSource(self, source):
        """
        TODO
        """
        # TODO
        self.logger.debug('BASLER: setting trigger source to {}'
                          ''.format(source))

    def _getAcquisitionFrameRateAbs(self):
        """
        TODO
        """
        # TODO
        return 25

    def _getImageWidth(self):
        """
        Return the width of the recorded images

        Returns
        -------
        width : int
            Width of the returned images in pixel
        """
        return self.device.Width.Value

    def _getImageHeight(self):
        """
        Return the height of the recorded images

        Returns
        -------
        height : int
            Height of the returned images in pixel
        """
        return self.device.Height.Value

    def setResolution(self, resolution=None):
        self.logger.warning('setResolution currently only returns current '
                            'resolution.')
        return (self._getImageWidth(), self._getImageHeight())

    def grabStart(self):
        self.logger.error('grabStart not yet implemented for {cam}'
                          ''.format(cam=self))
        raise NotImplementedError

    def grabStop(self):
        self.logger.error('grabStop not yet implemented for {cam}'
                          ''.format(cam=self))
        raise NotImplementedError

    def prepareRecording(self, num):
        self.logger.debug('Preparing {cam} for {num} images'
                          ''.format(cam=self,
                                    num=num))
        self._expected_triggered_images = num

    def record(self):
        self.logger.debug('Recording {num} images'
                          ''.format(num=self._expected_triggered_images))
        images = list()
        self.device.StartGrabbingMax(self._expected_triggered_images)

        while self.device.IsGrabbing():
            grabResult = self.device.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            # Image grabbed successfully?
            if grabResult.GrabSucceeded():
                images.append(grabResult.Array)
            else:
                self.logger.error("Error: ", grabResult.ErrorCode, grabResult.ErrorDescription)

        return images

    def setTriggerMode(self, mode=None):
        """
        Set the trigger mode of the camera to either "in", "out", or "off", or
        read the current trigger setting by passing None

        Parameters
        ----------
        mode : str
            The desired trigger mode. "in" means the camera receives a trigger
            signal, "out" means the camera sends a trigger signal, "off"" means
            the camera does not react to triggers. To read the current trigger
            setting pass None

        Notes
        -----
        Trigger mode "out" is not supported by Basler cameras (?). A
        NotImplementedError will be raised!

        Raises
        ------
        NotImplementedError
            Raised if mode == "out" because Basler cameras don't support
            triggering other devices.

        ValueError
            Raised if mode is a string containing something other than "in",
            "out", or "off"

        TypeError
            Raised if mode is not of type str or None

        Returns
        -------
        mode : str
            The trigger mode after applying the passed value
        """
        if isinstance(mode, str):
            self.logger.debug('Setting trigger mode {}'.format(mode))
            if mode.lower() == 'in':
                self.device.TriggerMode = 'On'
                self.device.TriggerSource = 'Line1'
                self.device.TriggerSelector = 'AcquisitionStart'
            elif mode.lower() == 'out':
                # Not supported by Basler cameras (?)
                # TODO: looks like Line2 and Line 3 can be configured as output
                raise NotImplementedError('Sending triggers is not supported '
                                          'by Basler cameras. Please use a '
                                          'different device to trigger the '
                                          'camera')
            elif mode.lower() == 'off':
                self.device.TriggerMode = 'Off'
            else:
                raise ValueError('Unexpected value in setTriggerMode. '
                                 'Expected "in", "out", or "off". Got {mode}'
                                 ''.format(mode=mode))
        elif mode is not None:
            raise TypeError('Trigger Mode should be None, "in", "out", or '
                            '"off". Got {mode}'.format(mode=mode))

        mode = self.device.TriggerMode.Value
        return 'in' if mode.lower() == 'on' else 'off'

    def __del__(self):
        self.closeDevice()

    def __repr__(self):
        return repr(self.device)


def main(arguments=''):
    import argparse
    import logging

    logging.basicConfig(level=logging.DEBUG)

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    args = parser.parse_args(arguments)

    devices = Camera.listDevices()
    cam = Camera(devices[0])
    print(cam)
    print('AAA'*5)
    print(cam.getImage())
    del cam


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv[1:]))

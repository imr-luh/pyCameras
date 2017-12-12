#!/usr/bin/env python3
__author__ = 'Niklas Kroeger'
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import re
import threading

import pypylon

from pyCameras.cameraTemplate import CameraControllerTemplate, CameraTemplate


class CameraControllerBasler(CameraControllerTemplate):
    """
    Camera Controller for Basler cameras based on pypylon
    """
    def __init__(self):
        """
        Camera Controller for Basler camera devices. This implementation uses
        pypylon as backend.
        """
        super(CameraControllerBasler, self).__init__()
        self.logger.debug('Starting Basler Camera Controller')
        self.device_handles = []

    def updateDeviceHandles(self):
        """
        Refresh the list of available devices
        """
        self.logger.debug('Searching for Basler camera devices')
        self.device_handles = pypylon.factory.find_devices()
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
        return CameraBasler(device_handle=device_handle)

    def closeController(self):
        """
        Close the camera controller and do the necessary cleanup
        """
        for dev in self.device_handles:
            del dev

    def __repr__(self):
        return "<Basler Camera Controller>"


class CameraBasler(CameraTemplate):
    """
    Basler Camera implementation based on pypylon
    """
    def __init__(self, device_handle):
        """
        Implementation of the basle camera device

        Parameters
        ----------
        device_handle : object
            Unique camera device handle to identify the camera
        """
        if isinstance(device_handle, pypylon.cython.factory.DeviceInfo):
            self.device_handle = device_handle
        elif isinstance(device_handle, str):
            # Assume the string contains the 8 digit serial number of the cam
            serial = re.findall(r'\(?([0-9]{7,9})\)?',
                                device_handle)[0]
            # List available cameras and try to find one with matching serial
            for device in self.listDevices():
                if device.serial_number == serial:
                    self.device_handle = device
        else:
            raise TypeError('device_handle should be of type '
                            'pypylon.cython.factory.DeviceInfo or subclassed '
                            'from it')
        super(CameraBasler, self).__init__(self.device_handle)
        self.grabberThread = None
        self.expected_tiggered_images = 0
        self.grabbedImages = []
        self.registerFeatures()
        self.openDevice()

    def registerFeatures(self):
        """
        Function to hold all implemented feature registrations
        """
        self.logger.debug('Registering implemented camera specific features')
        self.registerFeature('TriggerSource', self.setTriggerSource)
        self.registerFeature('AcquisitionFrameRateAbs', self.getAcquisitionFrameRateAbs)
        self.registerFeature('ImageWidth', self.getImageWidth)
        self.registerFeature('ImageHeight', self.getImageHeight)

    @staticmethod
    def listDevices():
        """
        List available Basler cameras

        Returns
        -------
        cams : list
            list of available Basler camera devices
        """
        return CameraControllerBasler().listDevices()

    def openDevice(self):
        """
        Open the device corresponding to self.device_handle and store the
        object in self.device
        """
        try:
            self.device = pypylon.factory.create_device(self.device_handle)
            self.device.open()
            self.logger.debug('Opened camera device: {device}'
                              ''.format(device=self.device))
        except Exception as e:
            self.logger.exception('Failed to open the camera device: {e}'
                                  ''.format(e=e))

    def closeDevice(self):
        """
        Close the connection to the device
        """
        try:
            self.logger.debug('Closing camera device: {device}'
                              ''.format(device=self.device))
            self.device.close()
            del self.device
            self.device = None

        except Exception as e:
            self.logger.exception('Failed to close the camera device: '
                                  '{e}'.format(e=e))

    def getImage(self, *args, **kwargs):
        """
        Get an image from the camera

        *args and **kwargs are ignored parameters!

        Returns
        -------
        img : np.ndarray
            Current camera image
        """
        return self.device.grab_image()

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
        """
        try:
            value = self.features[key]()
        except Exception:
            value = '<NOT READABLE>'
        return value

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
        if microns is not None:
            self.logger.debug('Setting <ExposureTimeRaw> to {microns}'
                              ''.format(microns=microns))
            self.device.properties['ExposureTimeRaw'] = microns
        return self.device.properties['ExposureTimeRaw']

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
            self.device.properties['GainRaw'] = gain
        return self.device.properties['GainRaw']

    def setFormat(self, format=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        format : str
            String describing the desired image format (e.g. "Mono8" or
            "Mono10"), or None to read the current image format

        Returns
        -------
        format : str
            The image format after applying the passed value
        """
        if format is not None:
            try:
                self.device.properties['PixelFormat'] = format
            except Exception as e:
                self.logger.exception(e)
        return self.device.properties['PixelFormat']

    def setTriggerSource(self, source):
        """
        TODO
        """
        # TODO
        self.logger.debug('BASLER: setting trigger source to {}'
                          ''.format(source))

    def getAcquisitionFrameRateAbs(self):
        """
        TODO
        """
        # TODO
        return 25

    def getImageWidth(self):
        """
        Return the width of the recorded images

        Returns
        -------
        width : int
            Width of the returned images in pixel
        """
        return self.device.properties['Width']

    def getImageHeight(self):
        """
        Return the height of the recorded images

        Returns
        -------
        height : int
            Height of the returned images in pixel
        """
        return self.device.properties['Height']

    def grabStart(self, numberFrames):
        """
        Prepare the camera to record a number of triggered frames

        This turns on the TriggerMode of the camera and tells the background
        grabberThread to wait for numberFrames images. These can afterwards
        be read with self.getImagesFromFrameList()

        Parameters
        ----------
        numberFrames : int
            Number of images that should be recorded through triggering
        """
        self.logger.debug('Setting up Trigger with {num} images'
                          ''.format(num=numberFrames))
        self.expected_tiggered_images = numberFrames
        self.setTriggerMode('in')
        self.grabberThread = threading.Thread(target=self._grabTriggeredImages)
        self.grabberThread.start()
        return
    # Temporary fix until function calls are unified to camera template
    setupFrameList = grabStart

    def grabStop(self):
        """
        Stop grabbing images and return camera from trigger mode to normal mode
        """
        self.setTriggerMode('off')
        self.grabberThread = None

    def _grabTriggeredImages(self):
        """
        Background function that tries to gather all triggered images

        This function should run in a background thread and is used to
        constantly check if a new image has been triggered. For this the
        pypylon grab_images() generator is used. It returns an image
        for every trigger signal the camera receives. The background thread is
        responsible to gather the expected number of triggered images in a
        list. To retreive the list please use self.getImagesFromFrameList()
        """
        self.logger.debug('CAMERABASLER.PY: _grabTriggeredImages started!')
        for img in self.device.grab_images(self.expected_tiggered_images):
            self.logger.debug('CAMERABASLER.PY: got triggered Image...')
            self.grabbedImages.append(img)
        self.expected_tiggered_images = 0
        self.logger.debug('CAMERABASLER.PY: done in background thread!')
        return

    def getImages(self, num=None):
        """
        Return list of images that were grabbed from the camera due to
        triggering

        This function waits for the background thread responsible for getting
        the images off the camera to stop and then returns the list of grabbed
        images.

        Parameters
        ----------
        num : int or None
            Number of image to return. Images are returned in recorded order.
            If None is passed all recorded images are returned

        Returns
        -------
        grabbed_images : list of images
            List of grabbed images that were recorded due to triggers
        """
        self.logger.debug('GETTING IMAGES')
        try:
            while self.grabberThread.isAlive():
                pass
        except AttributeError:
            pass
        self.grabStop()

        imgs = self.grabbedImages[:num]
        self.grabbedImages = self.grabbedImages[num:] if num is not None else []

        return imgs
    # Temporary fix until function calls are unified to camera template
    getImagesFromFrameList = getImages

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
        if mode is None:
            return self.device.properties['TriggerMode']
        elif isinstance(mode, str):
            self.logger.debug('BASLER: setting trigger mode {}'.format(mode))
            if mode.lower() == 'in':
                self.device.properties['TriggerMode'] = 'On'
                self.device.properties['TriggerSource'] = 'Line1'
                self.device.properties['TriggerSelector'] = 'FrameStart'
            elif mode.lower() == 'out':
                # Not supported by Basler cameras (?)
                raise NotImplementedError('Sending triggers is not supported '
                                          'by Basler cameras. Please use a '
                                          'different device to trigger the '
                                          'camera')
            elif mode.lower() == 'off':
                self.device.properties['TriggerMode'] = 'Off'
            else:
                raise ValueError('Unexpected value in setTriggerMode. '
                                 'Expected "in", "out", or "off". Got {mode}'
                                 ''.format(mode=mode))
            return self.device.properties['TriggerMode']
        else:
            raise TypeError('Trigger Mode should be None, "in", "out", or '
                            '"off". Got {mode}'.format(mode=mode))

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

    devices = CameraBasler.listDevices()
    cam = CameraBasler(devices[0])
    print(cam)
    print(cam.getImage())
    del cam


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv[1:]))

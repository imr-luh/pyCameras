#!/usr/bin/env python3
__author__ = "Niklas Kroeger"
__credits__ = ["Niklas Kroeger"]
__maintainer__ = "Niklas Kroeger"
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import logging
import re
import subprocess
import sys

import cv2
import math

from pyCameras.cameraTemplate import CameraControllerTemplate, CameraTemplate


def v4l2ctlSet(device, key, value):
    """
    Helper function calling the v4l2-ctl system process to set a usbCamera
    setting

    Parameters
    ----------
    device : str or int
        The device ID describing the usbCamera for which settings should be
        changed. This can be either the int as used by cv2.VideoCapture(), or
        the full device path (e.g. /dev/video0) as str. (Note: the number at
        the end of the device path should be equal to the OpenCV int)

    key : str
        Key used to identify the setting that should be changed. A list of
        available keys can be found by calling
        'v4l2-ctl --device {ID} --list-ctrls'

    value : str
        Value the setting should be set to

    Returns
    -------
    returncode
        The returncode of the v4l2 process call
    """
    return subprocess.call(['v4l2-ctl',
                            '-d',
                            '{device}'
                            ''.format(device=device),
                            '-c',
                            '{key}={value}'
                            ''.format(key=key, value=value)])

def v4l2ctlGet(device, key):
    """
    Helper function calling the v4l2-ctl system process to get settings for a
    usbCamera

    Parameters
    ----------
    device : str or int
        The device ID describing the usbCamera for which settings should be
        changed. This can be either the int as used by cv2.VideoCapture(), or
        the full device path (e.g. /dev/video0) as str. (Note: the number at
        the end of the device path should be equal to the OpenCV int)

    key : str
        Key used to identify the setting that should be changed. A list of
        available keys can be found by calling
        'v4l2-ctl --device {ID} --list-ctrls'

    Returns
    -------
    value : str or None
        The current value of the desired setting, None if the setting could
        not be found
    """
    output = subprocess.check_output(['v4l2-ctl',
                                      '-d',
                                      '{device}'
                                      ''.format(device=device),
                                      '--list-ctrls']).decode('utf-8')
    try:
        found = re.findall(r'^\s*{key}.+value=([0-9]+)'.format(key=key),
                           output,
                           re.MULTILINE)
        # TODO: This is only correct if 'flags=inactive' is not shown
        # TODO: in the subprocess output for key -> check in regex?!
        return found[0]
    except IndexError:
        return None


class CameraControllerUSB(CameraControllerTemplate):
    """
    Implementation of a generic usb camera controller
    """
    def __init__(self, num_of_cams=4):
        """
        Controller to handle device detection and interaction before actually
        opening the camera

        Parameters
        ----------
        num_of_cams : int
            Expected number of cameras currently connected. Default = 4
        """
        super(CameraControllerUSB, self).__init__()
        self.logger.debug('Starting usb Camera Controller')
        self.num_of_cams = num_of_cams
        self.device_handles = []

    def updateDeviceHandles(self):
        """
        Update the list of available device handles by trying to open the first
        self.num_of_cams and stopping at the first error
        """
        self.logger.debug('Searching for usb camera devices')
        self.device_handles = []
        for i in range(self.num_of_cams):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                self.device_handles.append(i)
                cap.release()
            else:
                break
        self.logger.debug('Found {num} usb camera devices: {devices}'
                          ''.format(num=len(self.device_handles),
                                    devices=self.device_handles))

    def getDevice(self, device_handle):
        """
        Open the actual capture device to grab images

        Parameters
        ----------
        device_handle
            One entry of the device list returned by self.listDevices()

        Returns
        -------
        The capture device of type usbCamera
        """
        self.logger.debug('Opening device {device_handle}'
                          ''.format(device_handle=device_handle))
        return CameraUSB(device_handle=device_handle)

    def closeController(self):
        """Delete all detected devices"""
        for cap in self.device_handles:
            del cap

    def __repr__(self):
        return "<USB Camer Controller>"


class CameraUSB(CameraTemplate):
    """
    Capture device representing a generic usb webcam
    """
    def __init__(self, device_handle):
        """
        Open a usb camera by its handle. The handle is the integer describing
        the devices number as used by opencv. If only one camera is attached
        the handle should ususally be 0, if multiple cameras are attached the
        handle for each device is counted up.
        """
        super(CameraUSB, self).__init__(device_handle)
        self.logger.debug('usb Camera {device_handle} is ready'
                          ''.format(device_handle=self.device_handle))
        self.registerFeature(key='resolution', callback=self.setResolution)
        self.openDevice()

    @staticmethod
    def listDevices():
        """
        List available usb camera devices

        Returns
        -------
        cams : list
            list of camera device handles that can be used to create a new
            camera instance
        """
        return CameraControllerUSB().listDevices()

    def openDevice(self):
        """
        Open the device for capturing images
        """
        self.device = cv2.VideoCapture(self.device_handle)

    def closeDevice(self):
        """
        Release the device
        """
        self.logger.debug('Releasing device capture for {device_handle}'
                          ''.format(device_handle=self.device_handle))
        self.device.release()
        del self.device
        self.device = None

    def getImage(self):
        ret, img = self.device.read()
        if ret is True:
            return img
        return None

    def getFeature(self, key):
        return self.features[key]()

    def setResolution(self, resolution=(640,480)):
        """
        Set the resolution of the device object

        Parameters
        ----------
        resolution : tuple
            The desired resolution in (width, height) the device should be set
            to
        """
        self.logger.info('Setting resolution to {resolution}'
                         ''.format(resolution=resolution))
        self.device.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.device.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.logger.debug('Resolution is ({res_x}, {res_y})'
                          ''.format(res_x=self.device.get(cv2.CAP_PROP_FRAME_WIDTH),
                                    res_y=self.device.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def setAutoExposure(self, value=False):
        """
        Turn auto exposure on or off

        Parameters
        ----------
        value : bool
            Turn auto exposure on if value == True, turn off if value == False

        Raises
        ------
        NotImplementedError
            If the functionality is not implemented for the sys.platform
            variable
        """
        self.logger.debug('Turning auto exposure {val}'
                          ''.format(val='off' if value is False else 'on'))
        if sys.platform == 'linux':
            v4l2ctlSet(device=self.device_handle,
                       key='exposure_auto',
                       value='1' if value is False else '3')
        elif sys.platform == 'win32':
            # TODO: Try and implement this for windows systems
            raise NotImplementedError
        else:
            raise NotImplementedError

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
            self.logger.debug('Setting exposure time to {microns}us'
                              ''.format(microns=microns))
            if sys.platform == 'linux':
                v4l2ctlSet(device=self.device_handle,
                           key='exposure_absolute',
                           value=microns//1000)
            elif sys.platform == 'win32':
                self.device.set(propId=cv2.CAP_PROP_EXPOSURE,
                                value=math.log((1/(microns/1000000)), 2))
            else:
                raise NotImplementedError
        else:
            if sys.platform == 'linux':
                # TODO: This is only correct if 'flags=inactive' is not shown
                # TODO: in the subprocess output -> check in regex?!
                return int(v4l2ctlGet(device=self.device_handle,
                                      key='exposure_absolute'))*1000
            elif sys.platform == 'win32':
                # TODO: make this work properly. Rounds to discrete values
                return int(2**-self.device.get(propId=cv2.CAP_PROP_EXPOSURE))*1000000
            else:
                raise NotImplementedError

    def __repr__(self):
        return "<USB Camera Device {handle}>".format(handle=self.device_handle)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    available_devices = CameraUSB.listDevices()
    cam = CameraUSB(available_devices[-1])

    cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    while True:
        img = cam.getImage()
        cv2.imshow('test', img)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    del cam

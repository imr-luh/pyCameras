"""
Implementation of the template for XIMEA XiB cameras
"""

__author__ = "Tim Engelbracht"
__credits__ = ["Tim Engelbracht"]
__maintainer__ = "not assigned"
__email__ = ""
__status__ = "Development"

import abc
import logging
import numpy as np
import cv2
from ximea import xiapi, xidefs
from ximea.xiapi import Xi_error
from pyCameras.utils import SettingsHandler

LOGGING_LEVEL = None

from pyCameras.cameraTemplate import CameraTemplate


class CameraXimeaXIB(CameraTemplate):
    """
    Class to access XIMEA XiB cameras.
    The device_handle is the serial number STRING
    """

    def __init__(self, device_handle):
        """
        Parameters
        ----------
        device_handle : object
            Some handle that identifies the camera and is used to open the
            device connection
        """
        super(CameraXimeaXIB, self).__init__(device_handle)

        self.logger = logging.getLogger(__name__)
        self.device_handle = device_handle  # Use this to identify and open the device
        self.device = xiapi.Camera()  # Use this variable to store the device itself
        self.triggermode = 'off'
        self.buffer = []

    @staticmethod
    def listDevices():
        """
        List all available camera devices correspponding to the class type
        """
        cam = xiapi.Camera()
        numCameras = cam.get_number_devices()
        result = []
        for i in range(numCameras):
            cam = xiapi.Camera(i)
            result.append(cam.get_device_info_string("device_sn").decode("utf-8"))
        return result

    def openDevice(self):
        """
        Open the device by using self.device_handle and store the device in
        self.device
        """
        try:
            self.device.open_device_by_SN(self.device_handle)  # SN: serialnumber
        except Xi_error as e:
            self.logger.error(f"Camera device is already in use: {e}")
            exit(11)

    def closeDevice(self):
        """
        Close the connection to the device and reset self.device to None.
        """
        self.device.close_device()

    def getImage(self):
        """
        Return a numpy array containing an image

        *args and **kwargs are optional parameters that may be ignored!

        imgdataformat: See get_image_data_numpy()
        """

        try:
            img = xiapi.Image()
            self.device.start_acquisition()
            self.device.get_image(img)

            data = img.get_image_data_numpy()
        except:
            raise
        finally:
            self.device.stop_acquisition()

        return data

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
        self.buffer = [xiapi.Image() for i in range(0, num)]

        return self.buffer

    def record(self):
        """
        Blocking image acquisition of a previously defined number of images
        (see prepareRecording).

        Returns
        -------
        imgs : list
            List of numpy arrays containing the recorded images
        """

        self.device.start_acquisition()
        imgs = []

        for img in self.buffer:
            self.device.get_image(img, timeout=10000)
            imgs.append(img.get_image_data_numpy())

        self.device.stop_acquisition()

        return imgs

    def isOpen(self):
        """
        Check if the device for this instance is currently open and ready to
        communicate

        Returns
        -------
        bool
            True if the camera connection is open, False if it is not
        """
        return self.device.CAM_OPEN

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
        self.device.set_exposure(microns)

        return microns

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
        self.device.set_imgdataformat(fmt)

        return fmt

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

        if gain is None:
            gain = self.device.get_gain()

        self.device.set_gain(gain)

        return gain

    def setStrobeMode(self, mode: str = "off", port: int = 1):
        raise NotImplementedError

    def setTriggerMode(self, mode: str = "off", port: int = 1):
        # TODO: Implement all features (rising edge, software, off, exposure active, frame active, etc.)
        """
        Set the trigger mode of the camera to either "in", "out" or "off", or
        read the current trigger setting ba passing None

        Set Pin

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

        if mode == "off":
            self.device.set_gpo_selector(f"XI_GPO_PORT{port}")
            self.device.set_gpo_mode("XI_GPO_OFF")
            self.device.set_gpi_selector(f"XI_GPI_PORT{port}")
            self.device.set_gpi_mode("XI_GPI_OFF")
            self.device.set_trigger_source("XI_TRG_OFF")
        elif mode == "in":
            self.device.set_gpi_selector(f"XI_GPI_PORT{port}")
            self.device.set_gpi_mode("XI_GPI_TRIGGER")
            self.device.set_trigger_source("XI_TRG_EDGE_RISING")
        elif mode == "out":
            self.device.set_gpo_selector(f"XI_GPO_PORT{port}")
            self.device.set_gpo_mode("XI_GPO_EXPOSURE_ACTIVE")
            # self.device.set_gpi_selector(f"XI_GPI_PORT{port}")
            # self.device.set_gpi_mode("XI_GPI_OFF")
            # self.device.set_trigger_source("XI_TRG_OFF")
        self.triggermode = mode

        return self.triggermode

    def setLensAperture(self, aperture: int = 2.8):
        """
        Set the lens aperture

        Parameters
        ----------
        aperture : int
            Desired lens aperture (2.8, 4, 5.6, 8, 11)

        Returns
        -------
        aperture : int
        """

        if aperture != 2.8 or 4 or 5.6 or 8 or 11:
            raise Exception('lens aperture not available')
        else:
            self.device.enable_lens_mode()
            self.device.set_lens_aperture_value(aperture)
            self.device.disable_lens_mode()

            return aperture

    def setLensFocus(self):



        raise NotImplementedError

    def setResolution(self, resolution=None, downsamplingType: str = 'XI_BINNING'):
        """
        Set the resolution of the camera to the given values in pixels or read
        the current resolution by passing None

        Parameters
        ----------
        resolution : tuple
            Desired camera resolution in the form (width, height), or None to
            read the current resolution
        downsamplingType : string
            'XI_BINNING'
            'XI_SKIPPING'

        Returns
        -------
        resolution : tuple
            The set camera resolution after applying the passed value
        """

        if resolution is None:
            return

        imsize = 3072 * 4096
        res = resolution[0] * resolution[1]
        quot = round(imsize / res, 0)

        if quot == 1:
            key = 'XI_DWN_1x1'
        elif quot == 4:
            key = 'XI_DWN_2x2'
        elif quot == 9:
            key = 'XI_DWN_3x3'
        elif quot == 16:
            key = 'XI_DWN_4x4'
        elif quot == 25:
            key = 'XI_DWN_5x5'
        elif quot == 36:
            key = 'XI_DWN_6x6'
        elif quot == 49:
            key = 'XI_DWN_7x7'
        elif quot == 64:
            key = 'XI_DWN_8x8'
        elif quot == 81:
            key = 'XI_DWN_9x9'
        elif quot == 100:
            key = 'XI_DWN_10x10'
        elif quot == 256:
            key = 'XI_DWN_16x16'
        else:
            raise Exception('Resolution not available')


        # if self.device.get_downsampling() == 'XI_DWN_1x1':
        #     pass

        self.device.set_downsampling_type(downsamplingType)
        self.device.set_downsampling(key)


if __name__ == '__main__':
    devices = CameraXimeaXIB.listDevices()
    print(devices)

    cam = CameraXimeaXIB(devices[0])

    cam.openDevice()
    print(f"Cam is open: {cam.isOpen()}")

    cam.setTriggerMode(mode='in', port=6)
    cam.setTriggerMode(mode='out', port=3)

    cam.setExposureMicrons(10000)
    cam.setPixelFormat('XI_MONO8')
    cam.setGain(0)

    cam.prepareRecording(36)
    imgs = cam.record()
    # data = cam.getImage()

    # cam.closeDevice()
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 1200, 900)
    cv2.setWindowProperty('image', cv2.WND_PROP_TOPMOST, 1)
    for i in imgs:
        cv2.imshow('image', i)
        cv2.waitKey(100)

    cam.setTriggerMode(mode='off')
    for i in range(10):
        img = cam.getImage()
        cv2.imshow('image', img)
        cv2.waitKey(500)

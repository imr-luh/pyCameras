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

import serial
import threading
import copy
import time
from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = logging.DEBUG


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


class Controller(ControllerTemplate):
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

        super(Controller, self).__init__()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
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
        return Camera(device_handle=device_handle)

    def closeController(self):
        """Delete all detected devices"""
        for cap in self.device_handles:
            del cap

    def __repr__(self):
        return "<USB Camer Controller>"


class Camera(CameraTemplate):
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

        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.logger.debug('usb Camera {device_handle} is ready'
                          ''.format(device_handle=self.device_handle))
        self.registerFeature(key='resolution', callback=self.setResolution)

        self._expected_images = 0  # see prepareRecording(num), and record()
        # self._checkDeviceHandle(device_handle)
        # Controller.updateDeviceHandles()
        self.noi = 0
        # self.exposure = 1000

        self.openDevice()
        self.imgData = []
        self.current_frame = 0
        self.trigger_in = True

        v4l2ctlSet(device=self.device_handle,
                   key='power_line_frequency',
                   value='1')

        # self.openDevice()




    def _cleanup(self):
        self.imgData.clear()
        self.noi = 0




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
        return Controller().listDevices()

    def openDevice(self):
        """
        Open the device for capturing images
        """
        camcon = Controller()
        camcon.updateDeviceHandles()
        print(camcon.device_handles)
        self.device_handle= camcon.device_handles[-1]


        self.device = cv2.VideoCapture(self.device_handle)



        if self.device.isOpened()==True:
            self.logger.debug('Open device {device} successfull'
                         ''.format(device=self.device))
        else:
            self.logger.debug('Open device {device} NOT successfull'
                         ''.format(device=self.device))

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
        if self.trigger_in ==True:
            ret, frame = self.device.read()
            return ret, frame
        else:

            ret, frame = self.device.read()
            if ret is True:
                return frame
            return None

    def prepareRecording(self, num):
        self._expected_images = num
        self.logger.debug('Prepare recording {num} images'
                          ''.format(num=self._expected_images))
        self.trigger_in = True

    def record(self):

        class ardu_thread(threading.Thread):
            def __init__(self, camera):
                threading.Thread.__init__(self)
                self.usbevent = threading.Event()
                self.logger = logging.getLogger(__name__)
                self.camera = camera
                self.expected_images = self.camera._expected_images
                self.gi = False #already got image

            def run(self):
                global frame
                self.logger.debug('Starting pyUSB arduino thread')
                self.ser = serial.Serial('/dev/ttyACM0', 9600)
                start = time.time()
                while True:
                    ard = self.ser.readline()
                    # print(ard)
                    #
                    if ard == b'1\r\n' and self.camera.noi < self.expected_images and not self.gi:
                        time.sleep(5/100)
                        self.camera.imgData.append(self.camera.current_frame)
                        self.camera.noi +=1
                        self.logger.debug('cap img {noi}'.format(noi=self.camera.noi))
                        self.gi = True
                    elif ard == b'0\r\n' and self.camera.noi < self.expected_images and self.gi:
                        self.gi = False
                    elif self.camera.noi == self.expected_images:
                        self.logger.debug('Captured {noi} images'
                                          ''.format(noi=self.camera.noi))

                        self.logger.debug('Stopping pyUSB arduino thread ')
                        dur = time.time()-start
                        bps = 17/dur
                        print("Bilder pro Sekunde")
                        print(bps)
                        # print(dur)
                        break

        class camThread(threading.Thread):

            def __init__(self,camera):
                threading.Thread.__init__(self)
                self.logger = logging.getLogger(__name__)
                self.camera = camera


            def run(self):
                global frame
                self.logger.debug('Starting pyUSB camera thread')

                while (True):
                    if self.camera.noi < self.camera._expected_images-1:
                        ret, self.camera.current_frame = self.camera.getImage()

                    else:
                        self.logger.debug('Stopping pyUSB camera thread')
                        break

                return self.camera.current_frame

        arduthread = ardu_thread(self)
        camthread = camThread(self)
        arduthread.start()
        camthread.start()
        arduthread.join()
        camthread.join()
        self.returndict =  []
        for c, value in enumerate(self.imgData, 1):
            # convert rgb to grayscale
            gray_img = cv2.cvtColor(value, cv2.COLOR_BGR2GRAY)
            # rotate image
            rows, cols = gray_img.shape
            M = cv2.getRotationMatrix2D((cols / 2, rows / 2),5, 1)
            rot_img = cv2.warpAffine(gray_img, M, (cols, rows))


            s= 550
            h = round(s/2)
            # mittelpunkt
            # y=round(s/2)
            y=round(rows/2)-40
            x = round(cols/2)+70



            crop_img = rot_img[y-h:y+h, x-h:x + h]
            self.returndict.append(crop_img)
        self._cleanup()




        return copy.deepcopy(self.returndict)


        # return [self.getImage() for _ in range(self._expected_images)]

    def getFeature(self, key):
        print(key)
        if self.trigger_in:
            return 'in'
        else:
            return 'off'

        # return self.features[key]()

    def setResolution(self, resolution=None):
        """
        Set the resolution of the device object

        Parameters
        ----------
        resolution : tuple
            The desired resolution in (width, height) the device should be set
            to
        """
        if resolution is None:
            # shape entries have to be flipped to return (width, height)
            return tuple([550,550])
        self.logger.info('Setting resolution to {resolution}'
                         ''.format(resolution=resolution))
        self.device.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.device.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.logger.debug('Resolution is ({res_x}, {res_y})'
                          ''.format(res_x=self.device.get(cv2.CAP_PROP_FRAME_WIDTH),
                                    res_y=self.device.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def setAutoExposure(self, value=False):


        self.logger.debug('Turning auto exposure {val}'
                          ''.format(val='off' if value is False else 'on'))

        v4l2ctlSet(device=self.device_handle,
                   key='exposure_auto',
                   value='1')
        # value = '3' for auto exposure on

        auto = v4l2ctlGet(device=self.device_handle,
                   key='exposure_auto')
        print("Autoexposure=",auto)


    def setExposureMicrons(self, microns=None):

        # microns = self.exposure
        self.logger.debug('Setting exposure time to {microns} us'
                          ''.format(microns=microns))


        self.setAutoExposure(False)
        v4l2ctlSet(device=self.device_handle,
                   key='exposure_absolute',
                   value=microns//1000)

        ex= v4l2ctlGet(device=self.device_handle,key='exposure_absolute')
        print("Exposure = ",ex)


    def __repr__(self):
        return "<USB Camera Device {handle}>".format(handle=self.device_handle)


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
        self.logger.debug("Setting trigger mode to: {mode}".format(mode=mode))

        if mode is None:
            self.trigger_in=False
            self.logger.debug("camera triggermode is None")

        elif mode == 'in':

            self.logger.debug("Camera triggermode is in")
            self.trigger_in=True
        elif mode == 'off':
            self.logger.debug("Camera tirgermode is off")
            self.trigger_in=False


        # raise NotImplementedError

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    available_devices = Camera.listDevices()

    cam = Camera(available_devices[-1])

    cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    while True:
        img = cam.getImage()
        cv2.imshow('test', img)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    del cam

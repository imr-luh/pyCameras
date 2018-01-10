#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Tim Betker"
__copyright__ = "Copyright 2017, LUH: IMR"
__credits__ = ["Rüder Beermann"]
# __license__ = ""
__version__ = "0.2"
__maintainer__ = "Tim Betker"
__email__ = "tim.betker@imr.uni-hannover.de"
__status__ = "alpha"
__package_name__ = "AVTcamera"
__developer__ = __author__

'''
Based on AVT implementation of Rüdiger Beermann and pymba: https://github.com/morefigs/pymba.git
'''

import time
import re
import copy
import numpy as np
from pymba import Vimba
from cameraTemplate import CameraControllerTemplate, CameraTemplate

class CameraControllerAVT(CameraControllerTemplate):
    """
    Camera controller for AVT cameras based on pymba
    """
    def __init__(self):
        """
        Camera controller for AVT camera devices. This implementation uses pymba as backend.
        """
        super(CameraControllerAVT, self).__init__()
        self.logger.debug('Starting AVT Camera Controller')
        self._vimba = Vimba()
        self._vimba.startup()
        self.__system = self._vimba.getSystem()
        self.__system.runFeatureCommand('GeVDiscoveryAllOnce')
        time.sleep(0.2)

    def updateDeviceHandles(self):
        """
        Refresh the list of available devices
        """
        self.logger.debug('Searching for AVT camera devices')
        self.device_handles = []
        cams = self._vimba.getCameraIds()
        for cam_id in cams:
            tmp = self._vimba.getCamera(cam_id)
            self.device_handles.append('<AVT {model} (MAC: {mac})>'
                                       ''.format(model=tmp._info.modelName,
                                                 mac=tmp._info.cameraIdString))

    def getDevice(self, device_handle):
        """
        Return the corresponding camera object for given device handle

        Parameters
        ----------
        device_handle : can be IP address, mac address or
                        camera ID (DEV_...) as reported by vimba.getCameraIds

        Returns
        -------
        cam : CameraAVT object
            A camera object for AVT devices corresponding to the given
            device handle
        # """

        # Check if device handle is list or tuple, if so: use first entry
        if isinstance(device_handle, (list, tuple)):
            device_handle = device_handle[0]

        self.logger.debug('Opening device {device_handle}'
                          ''.format(device_handle=device_handle))
        # Search for mac addresses in form 'DEV_XXXXXXXXXXXX'
        candidates = re.findall(r'(DEV_[0-9A-Z]{11,13})', device_handle)
        if len(candidates) == 0:
            # no mac address found: search for IP
            candidates = re.findall(r'[0-9]+(?:\.[0-9]+){3}', device_handle)

        try:
            return CameraAVT(device_handle=candidates[0], vimba=self._vimba)
        except Exception as e:
            self.logger.exception('Failed to open the camera device: {e}'
                                  ''.format(e=e))
            msg = '<Was not able to open camera with given device handle!!\n' \
                  'Handle must be IP or MAC address (DEV_XXXXXXXXXXXXX)>'
            e.message = msg
            print(e.message)
            raise

    def closeController(self):
        self._vimba.shutdown()
        self.logger.info("Vimba Camera Controller shutdown")

    def __repr__(self):
        return "<AVT Camera Controller>"


class CameraAVT(CameraTemplate):
    """
    AVT Camera implementation based on pymba

    Creating this Object automatically opens the camera. It is NOT necessary to call openDevice() !!!
    This is done to set some settings to put the camera into freerun mode.
    """
    def __init__(self, device_handle, vimba):
        """
        Implementation of the AVT camera device

        Parameters
        ----------
        device_handle : object
            Unique camera device handle to identify the camera
        """
        self._vimba = vimba
        super(CameraAVT, self).__init__(device_handle)
        self.device = self._vimba.getCamera(device_handle)
        self.modelName = self.device._info.modelName

        # Open device and activate freerun mode
        self.openDevice()
        # time.sleep(0.2)
        self.device.TriggerMode = 'Off'
        self.device.GevSCPSPacketSize = 1500    # Automatic setting not yet implemented in pymba (date: 11.12.17)
        # Influences framerate, necessary if network bandwidth is not big enough
        self.device.StreamBytesPerSecond = 10000000  # 10 Mb/sec
        self.imgData = []

        # TODO: Adjust Datatype depending on PixelFormat?

    def _cleanUp(self):
        """
        Does some cleanup jobs. Call after "AcquisitionStop".
        Calls:  - endCapture()
                - flushCaptureQueue()
                - revokeAllFrames()
        """
        self.device.endCapture()
        self.device.flushCaptureQueue()
        self.device.revokeAllFrames()

    def _frameCallback(self, frame):
        """
        Callback function to fill frames with data

        Parameters
        -------
        frame : frame object
            frame created by device.getFrame()
        """
        frame.waitFrameCapture(1000)
        data = frame.getBufferByteData()
        singleImg = np.ndarray(buffer=data,
                               dtype=np.uint8,
                               shape=(frame.height, frame.width))

        self.imgList.append(singleImg)
        frame.queueFrameCapture(self._frameCallback)

    @staticmethod
    def listDevices():
        """
        List available AVT cameras

        Returns
        -------
        cams : list
            list of available AVT devices
        """
        return CameraControllerAVT().listDevices()

    def openDevice(self):
        """
        Opens a camera device with the stored self.device object
        """
        try:
            self.logger.debug('Opening camera device')
            self.device.openCamera()
        except Exception as e:
            self.logger.exception('Failed to open the camera device: '
                                  '{e}'.format(e=e))

    def closeDevice(self):
        """
        Closes camera device
        """
        try:
            self.logger.debug('Closing camera device')
            self.device.closeCamera()
        except Exception as e:
            self.logger.exception('Failed to close the camera device: '
                                  '{e}'.format(e=e))

    def getImage(self, *args, **kwargs):
        """
        Get an image from the camera device

        *args and **kwargs are ignored parameters!

        !!! Warning: Check transfer rate of your network connection !!!
        Low transfer-rates may cause incomplete image transfer with missing data


        Returns
        -------
        img : np.ndarray
            Current camera image
        """

        self.logger.debug('Creating frame and starting acquisition')
        # Create new frame for camera
        frame = self.device.getFrame()
        # Announce frame
        frame.announceFrame()
        # Capture a camera image
        self.device.startCapture()
        frame.queueFrameCapture()
        self.device.runFeatureCommand('AcquisitionStart')
        self.device.runFeatureCommand('AcquisitionStop')
        frame.waitFrameCapture()

        # Get image data ...
        imgData = np.ndarray(buffer=frame.getBufferByteData(),
                                       dtype=np.uint8,
                                       shape=(frame.height,
                                              frame.width,
                                              1))

        # Do cleanup
        self._cleanUp()
        self.logger.debug('Image acquisition finished')

        return imgData.copy()

    def getImages(self):
        """
        Returns list of images from self variable imgData. Grab has to be run first.

        Parameters
        ----------

        Returns
        -------
        grabbed_images : list of images
            List of grabbed images that were recorded
        """
        if not self.imgData:
            print('No image data stored. Use grabStart, grabStop to acquire images!')

        return self.imgData

    def grabStart(self, numberFrames, triggerMode = 'in'):
        """
        Prepare the camera to record a number of triggered frames

        This turns on the TriggerMode of the camera

        Parameters
        ----------
        numberFrames : int
            Number of images that should be recorded through triggering

        triggerMode : string
            String to determine if camera gets triggered or triggers while grabbing
        """

        # Prepare trigger settings
        self.setTriggerMode(triggerMode)

        self.device.AcquisitionMode = 'MultiFrame'
        self.device.AcquisitionFrameCount = numberFrames
        self.imgList = []

        # Creating frames
        framelist = []
        for _ in range(numberFrames):
            frame = self.device.getFrame()
            frame.announceFrame()
            frame.queueFrameCapture(self._frameCallback)
            framelist.append(frame)

        self.device.startCapture()

        self.device.runFeatureCommand('AcquisitionStart')

        return 1

    def grabStop(self):
        """
        Stop grabbing images and return camera from trigger mode to normal mode.
        Stores all grabbed images in self.imgData. Will override variable.
        """
        self.device.runFeatureCommand('AcquisitionStop')
        # Do cleanup
        self._cleanUp()
        # Set back to freerun mode
        self.device.AcquisitionMode = 'Continuous'

        # Did not find a better way to return data
        self.imgData = copy.deepcopy(self.imgList)

        self.setTriggerMode('off')

        return 1

    def _liveView(self):
        """
        Live image stream an visualization through OpenCV window

        Leave _liveView by pressing "q"
        """
        cv.startWindowThread()
        cv.namedWindow("IMG", 2)
        cv.resizeWindow("IMG", 900, 900)
        frame = self.device.getFrame()
        frame.announceFrame()

        self.device.startCapture()

        framecount = 0
        droppedframes = []

        while True:
            try:
                frame.queueFrameCapture()
                success = True
            except:
                droppedframes.append(framecount)
                success = False
            self.device.runFeatureCommand("AcquisitionStart")
            self.device.runFeatureCommand("AcquisitionStop")
            frame.waitFrameCapture(1000)
            frame_data = frame.getBufferByteData()
            if success:
                live_img = np.ndarray(buffer=frame_data,
                                   dtype=np.uint8,
                                   shape=(frame.height, frame.width, 1))

                cv.imshow("IMG", live_img)
            framecount += 1
            key = cv.waitKey(1) & 0xFF
            if key == ord("q"):
                cv.destroyAllWindows()
                self.logger.info("Frames displayed: %i" % framecount)
                self.logger.info("Frames dropped: %s" % droppedframes)
                break

        # Cleanup
        self._cleanUp()

    def listFeatures(self):
        """
        Lists camera features
        """
        try:
            self.logger.debug('Listing camera features')
            featureNames = self.device.getFeatureNames()
            print ("Printing feature names: ...\n")
            print ("\n".join(featureNames))
        except Exception as e:
            self.logger.exception('Failed to get feature names: '
                                  '{e}'.format(e=e))

    def setFeature(self, *args, **kwargs):
        pass

    def getFeature(self, key):
        """
        Get a camera setting

        Parameters
        ----------
        key : string
            keystring of camera feature

        Returns
        -------
        value : str, int, float, object
            Value of the requested feature
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
            self.logger.debug('Setting <ExposureTime> to {microns}'
                              ''.format(microns=microns))
            self.device.ExposureTimeAbs = microns
        return self.device.ExposureTimeAbs

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
        if resolution is not None:
            self.logger.debug('Setting <Width> to {width}'
                              ''.format(width=resolution[0]))
            self.device.Width = resolution[0]
            self.logger.debug('Setting <Height> to {height}'
                              ''.format(height=resolution[1]))
            self.device.Height = resolution[1]
        return self.device.Width, self.device.Height

    def setGain(self, gain=None):
        """
        Set the gain of the camera to the given value or read the current value
        by passing None

        Parameters
        ----------
        gain : float
            Desired gain value in dB to be set, or None to read the current gain
            value

        Returns
        -------
        gain : int
            The gain value after applying the passed value
        """
        if gain is not None:
            self.logger.debug('Setting <Gain> to {gain}'
                              ''.format(gain=gain))
            self.device.Gain = gain
        return self.device.Gain

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

        if format is not None:
            self.logger.debug('Setting <PixelFormat> to {format}'
                              ''.format(format=format))
            self.device.PixelFormat = format
        return self.device.PixelFormat

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
        if mode is None:
            return self.device.TriggerMode
        elif isinstance(mode, str):
            if mode.lower() == 'in':
                self.device.TriggerMode = 'On'
                self.device.TriggerSource = 'Line1'
                self.device.TriggerSelector = 'FrameStart'
                self.device.TriggerActivation = "RisingEdge"
            elif mode.lower() == 'out':
                # TODO: Implement out trigger for AVT cameras
                raise NotImplementedError('Sending triggers is not'
                                          'implemented yet!')
            elif mode.lower() == 'off':
                self.device.TriggerMode = 'Off'
                self.device.TriggerSource = 'Freerun'
                self.device.TriggerSelector = 'FrameStart'
            else:
                raise ValueError('Unexpected value in setTriggerMode. '
                                 'Expected "in", "out", or "off". Got {mode}'
                                 ''.format(mode=mode))
            return self.device.TriggerMode
        else:
            raise TypeError('Trigger Mode should be None, "in", "out", or '
                            '"off". Got {mode}'.format(mode=mode))

    def __repr__(self):
        return repr(self.device)


if __name__ == '__main__':
    import logging
    import cv2 as cv

    logging.basicConfig(level=logging.DEBUG)
    bListFeatures = False
    bLiveView = False

    contr = CameraControllerAVT()
    handle = contr.listDevices()
    print (handle)

    # Dictionary to test different connection types/inputs
    source = {'IP':'130.75.27.144', 'Handle_list': handle, 'Handle': handle[0],
              'Bad_input': 'Yo Mama is fat'}
    # Use one of source entries here:
    # cam_device = contr.getDevice(source['Handle_list'])
    cam_device = contr.getDevice('DEV_000F314D941E')


    # cam_device.openDevice()

    # Listing features of device
    if bListFeatures:
        cam_device.listFeatures()

    # Get a image
    image = cam_device.getImage()
    cv.namedWindow('Captured image', cv.WINDOW_NORMAL)
    cv.resizeWindow('Captured image', 1000, 1000)
    cv.imshow('Captured image', image)
    cv.waitKey()

    if bLiveView:
        cam_device._liveView()

    images = cam_device.getImages(10)
    print (len(images))
    for _, img in enumerate(images):
        print ('Showing image {i}'.format(i=_))
        cv.imshow('Captured image', img)
        cv.waitKey()

    # grabbedImgs = cam_device.grabStart(10)
    # cam_device.grabStop()

    cam_device.closeDevice()

    contr.closeController()



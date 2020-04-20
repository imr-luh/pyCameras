#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Tim Betker"
__copyright__ = "Copyright 2017, LUH: IMR"
__credits__ = ["Rüdiger Beermann"]
# __license__ = ""
__version__ = "0.2"
__maintainer__ = "Tim Betker"
__email__ = "tim.betker@imr.uni-hannover.de"
__status__ = "alpha"
__package_name__ = "AVTcamera"
__developer__ = __author__

'''
Based on AVT implementation of Rüdiger Beermann and pymba:
https://github.com/morefigs/pymba.git
'''

import copy
import re
import time
import logging

import numpy as np

from vimba.vimba import Vimba
from vimba.error import VimbaCameraError, VimbaFeatureError

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = None


class Controller(ControllerTemplate):
    """
    Camera controller for AVT cameras based on pymba
    """

    def __init__(self):
        """
        Camera controller for AVT camera devices. This implementation uses
        pymba as backend.
        """
        super(Controller, self).__init__()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.logger.debug('Starting AVT Camera Controller')

    def updateDeviceHandles(self):
        """
        Refresh the list of available devices
        """
        self.logger.debug('Searching for AVT camera devices')
        self.device_handles = []
        with Vimba.get_instance() as vimba:
            cams = vimba.get_all_cameras()

        for cam in cams:
            self.device_handles.append('<AVT {model} (MAC: {mac})>'
                                       ''.format(model=cam.get_name(),
                                                 mac=cam.get_id()))

    def getDevice(self, device_handle):
        """
        Return the corresponding camera object for given device handle

        Parameters
        ----------
        device_handle : can be IP address, mac address or
                        camera ID (DEV_...) as reported by Camera.get_id()

        Returns
        -------
        cam : Camera object
            A camera object for AVT devices corresponding to the given
            device handle
        """

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
            return Camera(device_handle=candidates[0], vimba=self._vimba)
        except Exception as e:
            self.logger.exception('Failed to open the camera device: {e}'
                                  ''.format(e=e))
            msg = '<Was not able to open camera with given device handle!!\n' \
                  'Handle must be IP or MAC address (DEV_XXXXXXXXXXXXX)>'
            e.message = msg
            print(e.message)
            raise

    def closeController(self):
        self.logger.info("Vimba Camera Controller shutdown")

    def __repr__(self):
        return "<AVT Camera Controller>"


class Camera(CameraTemplate):
    """
    AVT Camera implementation based on pymba

    Creating this Object automatically opens the camera. It is NOT necessary to
    call openDevice() !!! This is done to set some settings to put the camera
    into freerun mode.
    """

    def __init__(self, device_handle):
        """
        Implementation of the AVT camera device

        Parameters
        ----------
        device_handle : object
            Unique camera device handle to identify the camera
        """
        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        with Vimba.get_instance() as vimba:
            # Device handle may be MAC address (camera_id) or IP
            self.device = vimba.get_camera_by_id(device_handle)

        # Sets package sizes and transfer rates for GigE cameras
        self._setup_transfer_sizes()

        self.device_handle = device_handle

        self.triggerModeSetting = 'off'

        # Register AVT specific functions.
        # Function to set maximum transfer rate depending on used network specifications
        self.registerFeature('maxRate', self._setMaxTransferRate)
        self.registerFeature('bandwidth', self._setMaxTransferRate)
        self.registerFeature('maximumTransferRate', self._setMaxTransferRate)
        self.registerFeature('transferRate', self._setTransferRate)
        # Function to set pixel format
        self.registerFeature('pixelFormat', self.setFormat)
        self.registerFeature('pixelType', self.setFormat)
        self.registerFeature('format', self.setFormat)

        self.framelist = []
        self.imgData = []
        self._clearQueueAndFrames()

        # Init data type LUT for each PixelFormat
        self.imageFormatLUT = {'Mono8': np.uint8, 'Mono12': np.uint16}

    def __del__(self):
        pass

    def _setup_transfer_sizes(self):
        with self.device as cam:
            # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
            try:
                cam.GVSPAdjustPacketSize.run()

                while not cam.GVSPAdjustPacketSize.is_done():
                    pass

            except (AttributeError, VimbaFeatureError):
                pass

    @staticmethod
    def listDevices():
        """
        List available AVT cameras

        Returns
        -------
        cams : list
            list of available AVT devices
        """
        return Controller().listDevices()

    def openDevice(self):
        """
        Opens a camera device with the stored self.device object
        """

        try:
            self.logger.debug('Opening camera device')
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
            del self.device
            self.device = None
        except Exception as e:
            self.logger.exception('Failed to close the camera device: '
                                  '{e}'.format(e=e))

    def isOpen(self):
        """
        Check if the device for this instance is currently open and ready to
        communicate

        Returns
        -------
        bool
            True if the camera connection is open, False if it is not
        """
        # AVT cameras do not have any isOpen-function by itself.
        # Assuming that if there is a device given in self.device, device is opened.
        if self.device is not None:
            return True
        else:
            return False

    def getImage(self, *args, **kwargs):
        """
        Get an image from the camera device

        *args and **kwargs are ignored parameters!

        !!! Warning: Check transfer rate of your network connection !!!
        Low transfer-rates may cause incomplete image transfer with missing
        data

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

        frame.waitFrameCapture(1000)
        self.device.runFeatureCommand('AcquisitionStop')

        # Get image data ...
        imgData = np.ndarray(buffer=frame.getBufferByteData(),
                             dtype=self.imageFormatLUT[self.device.PixelFormat],
                             shape=(frame.height,
                                    frame.width))

        # Do cleanup
        self._cleanUp()
        self.logger.debug('Image acquisition finished')

        return imgData.copy()

    def prepareRecording(self, num):
        """ Sets the camera to MultiFrame mode and prepares frames. Use with
        "record()"-function.

        Parameters
        ----------
        num : int
            number of frames to be captured during acquisition
        """
        self._clearQueueAndFrames()
        self.device.AcquisitionMode = 'MultiFrame'
        self.device.AcquisitionFrameCount = num

        # Creating frames
        self.framelist = []
        for _ in range(num):
            frame = self.device.getFrame()
            frame.announceFrame()
            frame.queueFrameCapture(self._frameCallback)
            self.framelist.append(frame)

        self.device.startCapture()

    def record(self):
        """ Blocking image acquisition, ends acquisition when num frames are
        captured, where num is set by "prepareRecording(num)". Only use with
        "prepareRecording(num)".

        Returns
        -------
        imgData : list
            List of images
        """

        self.imgData = []
        self.device.runFeatureCommand('AcquisitionStart')
        # Block until num images are captured
        while len(self.imgData) != len(self.framelist):
            pass
        self.device.runFeatureCommand('AcquisitionStop')
        # Do cleanup
        self._cleanUp()
        # Set back to freerun mode
        self.device.AcquisitionMode = 'Continuous'

        return copy.deepcopy(self.imgData)

    # TODO: If grabStart without "num" is needed - implement threading solution with while loop (similar to _liveView())
    def grabStart(self, num):
        """
        Prepares num images to be grabbed. This function is not blocking.
        Calling "grabStop()" will end acquisition.

        Parameters
        ----------
        num : int
            Number of images that should be recorded
        """
        self.device.AcquisitionMode = 'MultiFrame'
        self.device.AcquisitionFrameCount = num

        # Creating frames
        self.framelist = []
        for _ in range(num):
            frame = self.device.getFrame()
            frame.announceFrame()
            frame.queueFrameCapture(self._frameCallback)
            self.framelist.append(frame)

        self.device.startCapture()
        self.device.runFeatureCommand('AcquisitionStart')

    def grabStop(self):
        """
        Stop grabbing images and return camera to continuous mode.
        """
        self.device.runFeatureCommand('AcquisitionStop')
        # Do cleanup
        self._cleanUp()
        # Set back to freerun mode
        self.device.AcquisitionMode = 'Continuous'

        return copy.deepcopy(self.imgData)

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
            except Exception:
                droppedframes.append(framecount)
                success = False
            self.device.runFeatureCommand("AcquisitionStart")
            self.device.runFeatureCommand("AcquisitionStop")
            frame.waitFrameCapture(1000)
            frame_data = frame.getBufferByteData()
            if success:
                live_img = np.ndarray(buffer=frame_data,
                                      dtype=self.imageFormatLUT[self.device.PixelFormat],
                                      shape=(frame.height,
                                             frame.width))

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
            with self.device as cam:
                for feature in cam.get_all_features():
                    print("-------------------")
                    print("Feature name: {}".format(feature.get_name()))
                    print("Value       : {value} {units}".format(value=feature.get(), units=feature.get_unit()))
        except Exception as e:
            self.logger.exception('Failed to get feature names: '
                                  '{e}'.format(e=e))

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
        with self.device as cam:
            if microns is not None:
                self.logger.debug('Setting <ExposureTime> to {microns}'
                                  ''.format(microns=microns))
                exposure_time_feature = cam.ExposureTimeAbs
                exposure_time_feature.set(microns)

            return exposure_time_feature.get()

    def autoExposure(self):
        """
        Automatically sets the exposure time of the camera ONCE.
        Old exposure setting is lost during the process!

        Returns
        -------
        exposure : int
            The exposure time in microseconds after auto exposure
        """
        self.logger.debug("Starting automatic exposure control")
        with self.device as cam:
            exposure_auto_feature = cam.ExposureAuto
            exposure_auto_feature.set("Once")
            # TODO: Finish
            # Save trigger settings and activate acquisition until
            # auto exposure has settled
            triggerMode_buffer = self.triggerMode

            frame = self.device.getFrame()
            frame.announceFrame()

            self.device.startCapture()

            self.triggerMode = "off"
            max_iter = 100
            iter = 0
            # Auto exposure gets stuck if the border values are reached,
            # but further adjustments are necessary
            limits = (self.device.ExposureAutoMin, self.device.ExposureAutoMax)
            limit_cnt = 0
            last_exposure = -1

            self.device.runFeatureCommand("AcquisitionStart")
            while self.device.ExposureAuto != "Off":
                if last_exposure in limits:
                    limit_cnt += 1
                else:
                    limit_cnt = 0
                try:
                    frame.queueFrameCapture()
                except Exception:
                    pass
                frame.waitFrameCapture(1000)
                iter += 1
                last_exposure = self.device.ExposureTimeAbs
                if limit_cnt > 5:
                    self.logger.info("Auto exposure has run into limits. Continuing with exposure of: {exposure} ".format(
                        exposure=last_exposure))
                    self.device.ExposureAuto = "Off"
                if iter >= max_iter:
                    try:
                        raise TimeoutError("Timeout while setting auto exposure!")
                    except NameError:
                        # Python 2 compatible Error
                        raise Exception("Timeout while setting auto exposure!")

            # Cleanup
            self.device.runFeatureCommand("AcquisitionStop")
            self._cleanUp()

            self.triggerMode = triggerMode_buffer
            self.logger.debug("Set exposure time to {exposure}"
                              "".format(exposure=self.device.ExposureTimeAbs))

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
        with self.device as cam:
            height_feature = cam.Height
            width_feature = cam.Width
            if resolution is not None:
                self.logger.debug('Setting <Width> to {width}'
                                  ''.format(width=resolution[0]))
                width_feature.set(resolution[0])
                self.logger.debug('Setting <Height> to {height}'
                                  ''.format(height=resolution[1]))
                height_feature.set(resolution[1])
            return width_feature.get(), height_feature.get()

    def setGain(self, gain=None):
        """
        Set the gain of the camera to the given value or read the current value
        by passing None

        Parameters
        ----------
        gain : float
            Desired gain value in dB to be set, or None to read the current
            gain value

        Returns
        -------
        gain : int
            The gain value after applying the passed value
        """
        with self.device as cam:
            gain_feature = cam.Gain
            if gain is not None:
                self.logger.debug('Setting <Gain> to {gain}'
                                  ''.format(gain=gain))
                gain_feature.set(gain)
            return self.device.Gain

    def setFormat(self, fmt=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        fmt : str
            String describing the desired image format (e.g. "mono8"), or None
            to read the current image format. Check camera technical manual for available formats,
            may differ from model to model.

        Returns
        -------
        fmt : str
            The image format after applying the passed value
        """

        if fmt is not None:
            self.logger.debug('Setting <PixelFormat> to {fmt}'
                              ''.format(fmt=fmt))
            self.device.PixelFormat = fmt
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
        self.logger.debug("Setting trigger mode to: {mode}".format(mode=mode))
        if mode is None:
            return self.triggerModeSetting
        elif isinstance(mode, str):
            if mode.lower() == 'in':
                self.device.TriggerMode = 'On'
                self.device.TriggerSource = 'Line1'
                self.device.TriggerSelector = 'FrameStart'
                self.device.TriggerActivation = "RisingEdge"

                self.triggerModeSetting = 'in'
            elif mode.lower() == 'out':
                # TODO: Implement out trigger for AVT cameras
                self.triggerModeSetting = 'out'
                raise NotImplementedError('Sending triggers is not'
                                          'implemented yet!')
            elif mode.lower() == 'off':
                self.device.TriggerMode = 'Off'
                self.device.TriggerSource = 'Freerun'
                self.device.TriggerSelector = 'FrameStart'

                self.triggerModeSetting = 'off'
            else:
                raise ValueError('Unexpected value in setTriggerMode. '
                                 'Expected "in", "out", or "off". Got {mode}'
                                 ''.format(mode=mode))
            return self.triggerModeSetting
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

    contr = Controller()
    handle = contr.listDevices()
    print(handle)

    # Dictionary to test different connection types/inputs
    source = {'IP': '130.75.27.144',
              'Handle_list': handle,
              'Handle': handle[0],
              'Bad_input': 'Yo Mama is fat'}
    # Use one of source entries here:
    cam_device = contr.getDevice(source['Handle_list'])
    # cam_device = contr.getDevice('DEV_000F314D941E')

    # Test auto exposure
    cam_device = Camera('DEV_000F314E2C01')

    cam_device.exposure = 400000
    print("Before: ", cam_device.exposure)
    exposure = cam_device.autoExposure()
    print("After: ", cam_device.exposure)

    # Listing features of device
    if bListFeatures:
        cam_device.listFeatures()

    # Get an image
    image = cam_device.getImage()
    cv.namedWindow('Captured image', cv.WINDOW_NORMAL)
    cv.resizeWindow('Captured image', 1000, 1000)
    cv.imshow('Captured image', image)
    cv.waitKey()

    if bLiveView:
        cam_device._liveView()

    images = cam_device.getImages(10)
    print(len(images))
    for _, img in enumerate(images):
        print('Showing image {i}'.format(i=_))
        cv.imshow('Captured image', img)
        cv.waitKey()

    # grabbedImgs = cam_device.grabStart(10)
    # cam_device.grabStop()

    cam_device.closeDevice()

    contr.closeController()

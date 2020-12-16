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
import logging
import re
import time

import numpy as np
from pymba import Vimba

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
        self._vimba.shutdown()
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

    def __init__(self, device_handle, vimba=None):
        """
        Implementation of the AVT camera device

        Parameters
        ----------
        device_handle : object
            Unique camera device handle to identify the camera
        """
        if vimba is None:
            self._vimba = Vimba()
            self._vimba.startup()
            self.__system = self._vimba.getSystem()
            self.__system.runFeatureCommand('GeVDiscoveryAllOnce')
            time.sleep(0.2)
        else:
            self._vimba = vimba
        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.device = self._vimba.getCamera(self._checkDeviceHandle(device_handle))
        self.device_handle = device_handle
        self.camId = None

        self.modelName = self.device._info.modelName
        self.triggerModeSetting = 'off'

        # Open device and activate freerun mode
        self.openDevice()
        # time.sleep(0.2)
        self.device.TriggerMode = 'Off'

        # self.device.GevSCPSPacketSize = 1500  # Automatic setting not yet implemented in pymba (date: 11.12.17)
        # self.device.GevSCPSPacketSize = 8228
        self.device.runFeatureCommand("GVSPAdjustPacketSize")
        # Influences framerate, necessary if network bandwidth is not big enough
        # NOTE: Functions self._setMaxTransferRate, self._setTransferRate and self._setNumberCams may change this value
        # self.device.StreamBytesPerSecond = 10000000  # 10 Mb/sec (without GigE)
        self.device.StreamBytesPerSecond = 115000000  # 100 Mb/sec (with GigE)

        self.maxTransferRate = 115000000
        self.numCams = 1
        self.isSet = {'rate': False, 'numCams': False}

        # Register AVT specific functions.
        # Function to set maximum transfer rate depending on used network specifications
        self.registerFeature('maxRate', self._setMaxTransferRate)
        self.registerFeature('bandwidth', self._setMaxTransferRate)
        self.registerFeature('maximumTransferRate', self._setMaxTransferRate)
        self.registerFeature('transferRate', self._setTransferRate)
        # Function to set number of cameras, may affect the available transfer rate per camera
        self.registerFeature('numCams', self._setNumberCams)
        self.registerFeature('numberCams', self._setNumberCams)
        self.registerFeature('numberOfCameras', self._setNumberCams)

        self.ReverseX = False
        self.ReverseY = False

        # Function to set the Camera Images as flipped according to X and Y
        self.registerFeature('reverseImageX', self._setReverseX)
        self.registerFeature('reverseX', self._setReverseX)
        self.registerFeature('flipX', self._setReverseX)
        self.registerFeature('flippedImageX', self._setReverseX)

        self.registerFeature('reverseImageY', self._setReverseY)
        self.registerFeature('reverseY', self._setReverseY)
        self.registerFeature('flipY', self._setReverseY)
        self.registerFeature('flippedImageY', self._setReverseY)

        self.framelist = []
        self.imgData = []
        self._clearQueueAndFrames()

        # Init data type LUT for each PixelFormat
        self.imageFormatLUT = {'Mono8': np.uint8, 'Mono12': np.uint16}

    def __del__(self):
        # self._cleanUp()
        self._vimba.shutdown()

    def _checkDeviceHandle(self, device_handle):
        """
        Return the corresponding camera object for given device handle

        Parameters
        ----------
        device_handle : can be IP address, mac address or
                        camera ID (DEV_...) as reported by vimba.getCameraIds

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
        candidates = re.findall(r'([0-9A-Z]{11,13})', device_handle)
        if len(candidates) == 0:
            # no mac address found: search for IP
            candidates = re.findall(r'[0-9]+(?:\.[0-9]+){3}', device_handle)

        return candidates[0]

    def _setMaxTransferRate(self, rate=None):
        """
        Sets the transfer rate by changing 'StreamBytesPerSecond'.
        If passed None, will return actual rate set.

        Parameters
        ----------
        rate: int
            Maximum bandwidth available. Typical values:
            - with GigE     : 115000000
            - without GigE  : 10000000

        Returns
        -------
        self.max_bandwidth: int
            If passed None: returns set bandwidth
        """
        self.logger.debug("Setting max transfer rate for device {handle} to {rate}"
                          "".format(handle=self.device_handle, rate=rate))
        if rate is None:
            return self.maxTransferRate
        self.maxTransferRate = rate
        self.isSet['rate'] = True

        # Call function if number of cams was set
        if self.isSet['numCams']:
            self._setTransferRate()
        else:
            self.device.StreamBytesPerSecond = rate

        return self.maxTransferRate

    def _setNumberCams(self, num=None):
        """
        Sets the number of AVT cameras used (this will affect the maximum transfer rate for each camera).
        If passed None, will return actual number of cameras set.

        Parameters
        ----------
        num: int
            Number of AVT cameras

        Returns
        -------
        self.numCams: int
            Number of AVT cameras set for this object
        """
        self.logger.debug("Setting number of cameras for device {handle} to {num}"
                          "".format(handle=self.device_handle, num=num))
        if num is None:
            return self.numCams
        self.numCams = num
        self.isSet['numCams'] = True

        if self.isSet['rate']:
            self._setTransferRate()

        return self.numCams

    def _setTransferRate(self):
        """
        Takes maxTransferRate and numCams to compute a viable transfer rate for the device.
        """
        transfer_rate = int(self.maxTransferRate / self.numCams)
        self.device.StreamBytesPerSecond = transfer_rate
        self.logger.debug("Setting transfer rate for {device} to {rate}"
                          "".format(device=self.device_handle, rate=transfer_rate))

    def _clearQueueAndFrames(self):
        """
        Does some cleanup jobs. Call after whenever you feel like there might be a buffer overflow.
        Calls:  - flushCaptureQueue()
                - revokeAllFrames()
        """
        self.device.flushCaptureQueue()
        self.device.revokeAllFrames()

    def _cleanUp(self):
        """
        Does some cleanup jobs. Call after "AcquisitionStop".
        Calls:  - endCapture()
                - flushCaptureQueue()
                - revokeAllFrames()
        """
        self.device.endCapture()
        self._clearQueueAndFrames()

    def _frameCallback(self, frame):
        """
        Callback function to fill frames with data

        Parameters
        -------
        frame : frame object
            frame created by device.getFrame()
        """
        frame.waitFrameCapture(1000)
        # Get image data ...
        singleImg = np.ndarray(buffer=frame.getBufferByteData(),
                               dtype=self.imageFormatLUT[self.device.PixelFormat],
                               shape=(frame.height,
                                      frame.width))

        self.imgData.append(singleImg)
        frame.queueFrameCapture(self._frameCallback)

    def _getCamId(self):
        """
        Creates a cam-specific cam id, which consists of the manufacturer and a
        4 digit number. This id makes it possible to identify the virtual
        object with real object.

        Returns
        -------
        camId : "unique" cam id
        """
        if self.camId is None:
            mfr = b'AVT'  # mfr = manufacturer
            id = self.device._info.cameraIdString[-4:]
            camId = b'_'.join((mfr, id)).decode('utf-8')

            return camId
        else:
            return self.camId

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
        incomplete_frames = 0
        incomplete_frame_limit = 20
        while frame.getReceiveStatus() != 0:
            frame.queueFrameCapture()
            frame.waitFrameCapture(1000)
            incomplete_frames += 1
            if incomplete_frames > incomplete_frame_limit:
                raise RuntimeError("More than {lim} frames in a row were incomplete! Check transfer settings!"
                                   "".format(lim=incomplete_frame_limit))
        self.device.runFeatureCommand('AcquisitionStop')
        self.logger.debug("Trashed frames: {t_frames}".format(t_frames=incomplete_frames))

        # Get image data ...
        imgData = np.ndarray(buffer=frame.getBufferByteData(),
                             dtype=self.imageFormatLUT[self.device.PixelFormat],
                             shape=(frame.height,
                                    frame.width))

        # Do cleanup
        self._cleanUp()
        self.logger.debug('Image acquisition finished')

        if self.ReverseX and self.ReverseY:
            return np.flipud(np.fliplr(imgData.copy()))
        elif self.ReverseX and not self.ReverseY:
            return np.flipud(imgData.copy())
        elif not self.ReverseX and self.ReverseY:
            return np.fliplr(imgData.copy())
        else:
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
            featureNames = self.device.getFeatureNames()
            print("Printing feature names: ...\n")
            print("\n".join(featureNames))
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
        if microns is not None:
            self.logger.debug('Setting <ExposureTime> to {microns}'
                              ''.format(microns=microns))
            self.device.ExposureTimeAbs = microns
        return self.device.ExposureTimeAbs

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
        self.device.ExposureAuto = "Once"
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
            Desired gain value in dB to be set, or None to read the current
            gain value

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

    def setPixelFormat(self, fmt=None):
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
            self.device.runFeatureCommand("GVSPAdjustPacketSize")
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

    def _setReverseX(self, reversed=None):
        """
        Flips the image sent by the camera horizontally. The Region of interest is applied after flipping. If the avt
            camera used has the hardware support with ReverseX the hardwware Interface is used. If not the device
            the self property called ReverseY to differentiate this.

        Parameters
        ----------
        reversed : bool
            If a horizontally flipped image is desired then reversed has to be set as true. Default value for
            devices ReverseX value is false. The Region of interest is applied after flipping.

        Returns
        -------
        ReverseX : bool
            Returns the current state of the ReverseX value. Default is false.
        """
        if reversed is not None:
            if hasattr(self.device, 'ReverseX'):
                self.device.ReverseX = reversed
            else:
                self.ReverseX = reversed
            self.logger.debug('Setting <ReverseX> to {reversed}'.format(reversed=reversed))

        if hasattr(self.device, 'ReverseX'):
            return self.device.ReverseX
        else:
            return self.ReverseX

    def _setReverseY(self, reversed=None):
        """
        Flips the image sent by the camera verticall. The Region of interest is applied after flipping. If the avt
        camera used has the hardware support with ReverseY the hardwware Interface is used. If not the device the self
        property called ReverseY to differentiate this.

        Parameters
        ----------
        reversed : bool
            If a vertically flipped image is desired then reversed has to be set as true. Default value for
            devices ReverseY value is false.

        Returns
        -------
        ReverseY : bool
            Returns the current state of the ReverseY value. Default is false.
        """
        if reversed is not None:
            if hasattr(self.device, 'ReverseY'):
                self.device.ReverseY = reversed
            else:
                self.ReverseY = reversed
            self.logger.debug('Setting <ReverseY> to {reversed}'.format(reversed=reversed))

        if hasattr(self.device, 'ReverseY'):
            return self.device.ReverseY
        else:
            return self.ReverseY

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

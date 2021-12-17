#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Tim Betker"
__copyright__ = "Copyright 2017, LUH: IMR"
__credits__ = ["Rüdiger Beermann"]
# __license__ = ""
__version__ = "0.3"
__maintainer__ = "Tim Betker"
__email__ = "tim.betker@imr.uni-hannover.de"
__status__ = "alpha"
__package_name__ = "AVTcamera"
__developer__ = __author__

import copy
import re
import time
import logging
import threading
import cv2

import numpy as np

from vimba.vimba import Vimba
from vimba.frame import FrameStatus, AllocationMode
from vimba.error import VimbaFeatureError, VimbaCameraError

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = None


class FrameHandler:
    def __init__(self, max_imgs=1000):
        self.img_data = list()
        self.max_imgs = max_imgs

    def __call__(self, cam, frame):
        if frame.get_status() == FrameStatus.Complete:
            if len(self.img_data) < self.max_imgs:
                # After max_imgs images all frames will be trashed
                self.img_data.append(frame.as_numpy_ndarray())

        cam.queue_frame(frame)

    def get_images(self):
        return self.img_data.copy()


class FrameHandlerBlocking(FrameHandler):
    def __init__(self, max_imgs):
        super(FrameHandlerBlocking, self).__init__(max_imgs=max_imgs)
        self.shutdown_event = threading.Event()

    def __call__(self, cam, frame):
        # print(f"{cam} Frame {self.counter}")
        # print(f"{cam} Frame Status {frame.get_status()}")
        if frame.get_status() == FrameStatus.Complete:
            if len(self.img_data) < self.max_imgs:
                # After max_imgs images all frames will be trashed
                self.img_data.append(frame.as_numpy_ndarray())
                if len(self.img_data) >= self.max_imgs:
                    self.shutdown_event.set()

        cam.queue_frame(frame)


class CVLiveViewHandler:
    def __init__(self, width, height):
        self.shutdown_event = threading.Event()
        self.window_width = width
        self.window_height = height

    def __call__(self, cam, frame):
        key = cv2.waitKey(1)
        if key in (ord('q'), 13):
            self.shutdown_event.set()
            return

        elif frame.get_status() == FrameStatus.Complete:
            print('{} acquired {}'.format(cam, frame), flush=True)

            msg = 'Stream from \'{}\'. Press <Enter> or <q> to stop stream.'
            window_title = msg.format(cam.get_name())
            cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
            # TODO: Currently resizes the window to 900 x 900 for every frame (moving to init does not work)
            cv2.resizeWindow(window_title, self.window_width, self.window_height)
            cv2.imshow(window_title, frame.as_opencv_image())

        cam.queue_frame(frame)


class Grabber(threading.Thread):
    def __init__(self, camera, frame_handler, number_of_images):
        super(Grabber, self).__init__()
        self._stop_event = threading.Event()
        self._cam = camera
        self._frame_handler = frame_handler
        self._buffer_count = number_of_images

    def run(self):
        # Start vimba and camera contexts
        with self._cam as cam:
            cam.start_streaming(handler=self._frame_handler, buffer_count=self._buffer_count)
            # Keep context alive until grabber is stopped
            while not self._stop_event.is_set():
                pass

    def stop(self):
        # with Vimba.get_instance():
        #     self._cam.stop_streaming()
        self._stop_event.set()


class FrameProducer(threading.Thread):
    def __init__(self, cam, max_imgs):
        super(FrameProducer, self).__init__()

        self.cam = cam
        self.max_imgs = max_imgs
        # self.img_data = list()
        self.frame_list = list()
        self.shutdown_event = threading.Event()
        self.ready_event = threading.Event()

    def __call__(self, cam, frame):
        # if frame.get_status() == FrameStatus.Complete:
        # print(len(self.img_data))
        # if len(self.img_data) < self.max_imgs:
        if len(self.frame_list) < self.max_imgs:
            # After max_imgs images all frames will be trashed
            # self.img_data.append(frame.as_numpy_ndarray())
            self.frame_list.append(copy.deepcopy(frame))
            # if len(self.img_data) >= self.max_imgs:

        cam.queue_frame(frame)
        if len(self.frame_list) >= self.max_imgs:
            self.stop()

    def stop(self):
        self.shutdown_event.set()

    def run(self):
        try:
            with self.cam:
                try:
                    # self.cam.start_streaming(handler=self, buffer_count=self.max_imgs)
                    self.cam.start_streaming(handler=self, buffer_count=2)
                    self.ready_event.set()
                    self.shutdown_event.wait()

                finally:
                    self.cam.stop_streaming()
        except VimbaCameraError:
            pass

    def get_images(self):
        return [f.as_numpy_ndarray() for f in self.frame_list]
        # return self.img_data.copy()


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
            return Camera(device_handle=candidates[0])
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

        self.thread_lock = threading.Lock()
        
        self.vmb = Vimba.get_instance()
        self.vmb.__enter__()

        # with Vimba.get_instance() as vimba:
        self.device = self.vmb.get_camera_by_id(device_handle)

        self.device_handle = device_handle

        self.triggerModeSetting = 'off'

        # Register AVT specific functions.
        # Function to set pixel format
        self.registerFeature('pixelFormat', self.setPixelFormat)
        self.registerFeature('pixelType', self.setPixelFormat)
        self.registerFeature('format', self.setPixelFormat)

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

        self._frame_handler = None
        self._number_of_images = None
        self._grabber = None
        self.imgData = []

        # Init data type LUT for each PixelFormat
        self.imageFormatLUT = {'Mono8': np.uint8, 'Mono12': np.uint16}

        # Sets package sizes and transfer rates for GigE cameras
        # TODO: How to set transfer rates for already running cameras?
        self._setup_transfer_sizes()

    def __del__(self):
        self.vmb.__exit__(None, None, None)

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
    def openController():
        """
        Open the camera controller and prepare everything to make cameras
        accessible
        """
        pass

    @staticmethod
    def closeController():
        """
        Close the camera controller
        """
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
        # Cameras are handled within a context.
        # There is a "open" method, but using the with statement seems more safe.
        pass

    def closeDevice(self):
        """
        Closes camera device
        """
        del self.device
        self.device = None

    def isOpen(self):
        """
        Check if the device for this instance is currently open and ready to
        communicate

        Returns
        -------
        bool
            True if the camera connection is open, False if it is not
        """
        # Cameras are handled within a context. No available isOpen function.
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
        with self.device as cam:
            frame = cam.get_frame(timeout_ms=2000)
            img = frame.as_numpy_ndarray()

        self.logger.debug('Image acquisition finished')

        if self.ReverseX and self.ReverseY:
            return np.flipud(np.fliplr(img.copy()))
        elif self.ReverseX and not self.ReverseY:
            return np.flipud(img.copy())
        elif not self.ReverseX and self.ReverseY:
            return np.fliplr(img.copy())
        else:
            return img.copy()

    def prepareRecording(self, num):
        """ Sets the camera to MultiFrame mode and prepares frames. Use with
        "record()"-function.

        Parameters
        ----------
        num : int
            number of frames to be captured during acquisition
        """
        with self.device as cam:
            cam.AcquisitionMode.set('MultiFrame')
            cam.AcquisitionFrameCount.set(num)

        self._grabber = FrameProducer(self.device, num)
        self._grabber.start()

        self._grabber.ready_event.wait()

        # self._frame_handler = FrameHandlerBlocking(max_imgs=num)
        # self._grabber = Grabber(self.device, self._frame_handler, num)
        # self._grabber.start()

        # while not self.device.is_streaming():
        #     pass

        # time.sleep(1)

    def record(self):
        """ Blocking image acquisition, ends acquisition when num frames are
        captured, where num is set by "prepareRecording(num)". Only use with
        "prepareRecording(num)".

        Returns
        -------
        imgData : list
            List of images
        """
        print("Waiting for lock")
        # self.thread_lock.acquire()

        # self._frame_handler.shutdown_event.wait()
        self._grabber.shutdown_event.wait()
        self.imgData = self._grabber.get_images()
        # print("Acquired lock")
        # Reset thread
        # self._grabber.stop()
        self._grabber.join()
        self._grabber = None

        # Get data and clear frame handler
        # self.imgData = self._frame_handler.get_images()
        # self._frame_handler = None

        with self.device as cam:
            cam.AcquisitionMode.set('Continuous')

        return self.imgData

    def grabStart(self, number_of_images):
        """
        Prepares num images to be grabbed. This function is not blocking.
        Calling "grabStop()" will end acquisition.

        Parameters
        ----------
        num : int
            Number of images that should be recorded
        """
        # TODO: Prevent starting multiple threads? - Camera is occupied anyway...
        self._frame_handler = FrameHandler(max_imgs=1000)
        self._grabber = Grabber(self.device, self._frame_handler, number_of_images=number_of_images)
        self._grabber.start()

    def grabStop(self):
        """
        Stop grabbing images and return camera to continuous mode.
        """
        # Reset thread
        while self._grabber._buffer_count != len(self._frame_handler.img_data):
            continue

        self._grabber.stop()
        self._grabber.join()
        self._grabber = None

        # Get data and clear frame handler
        self.imgData = self._frame_handler.get_images()
        self._frame_handler = None

        return self.imgData

    #@vimba_context
    def _liveView(self):
        """
        Live image stream an visualization through OpenCV window

        Leave _liveView by pressing "q"
        """
        handler = CVLiveViewHandler(400, 400)
        self.setTriggerMode("off")
        with self.device as cam:
            cam.AcquisitionMode.set('Continuous')
            try:
                cam.start_streaming(handler=handler, buffer_count=10)
                handler.shutdown_event.wait()
            finally:
                cam.stop_streaming()

    #@vimba_context
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
        except Exception as e:
            self.logger.exception('Failed to get feature names: '
                                  '{e}'.format(e=e))

    #@vimba_context
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
            exposure_time_feature = cam.ExposureTimeAbs
            if microns is not None:
                self.logger.debug('Setting <ExposureTime> to {microns}'
                                  ''.format(microns=microns))
                exposure_time_feature.set(microns)

            return exposure_time_feature.get()

    #@vimba_context
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
        with self.device as cam:
            if reversed is not None:
                if cam.get_feature_by_name("ReverseX"):
                    cam.ReverseX.set(reversed)
                else:
                    self.ReverseX = reversed
                self.logger.debug('Setting <ReverseX> to {reversed}'.format(reversed=reversed))

            if cam.get_feature_by_name("ReverseX"):
                return cam.ReverseX.get()
            else:
                return self.ReverseX

    #@vimba_context
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
        with self.device as cam:
            if reversed is not None:
                if cam.get_feature_by_name("ReverseY"):
                    cam.ReverseY.set(reversed)
                else:
                    self.ReverseY = reversed
                self.logger.debug('Setting <ReverseY> to {reversed}'.format(reversed=reversed))

            if cam.get_feature_by_name("ReverseY"):
                return cam.ReverseY.get()
            else:
                return self.ReverseY

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

    def _setTransferRate(self):
        """
        Takes maxTransferRate and numCams to compute a viable transfer rate for the device.
        Formula: StreamBytesPerSecond = Height x Width x FrameRate x BytesPerPixel
        """


        transfer_rate = int(self.maxTransferRate / self.numCams)
        self.device.StreamBytesPerSecond = transfer_rate
        self.logger.debug("Setting transfer rate for {device} to {rate}"
                          "".format(device=self.device_handle, rate=transfer_rate))

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

    #@vimba_context
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

            exposure_time_feature = cam.ExposureTimeAbs
            # Save trigger settings and activate acquisition until
            # auto exposure has settled
            trigger_mode_buffer = self.triggerMode

            self.triggerMode = "off"
            max_iter = 100
            iter = 0
            # Auto exposure gets stuck if the border values are reached,
            # but further adjustments are necessary
            limits = (cam.ExposureAutoMin.get(), cam.ExposureAutoMax.get())
            limit_cnt = 0
            last_exposure = -1

            while str(exposure_auto_feature.get()).lower() != "off":
                frame = cam.get_frame(timeout_ms=2000)
                if last_exposure in limits:
                    limit_cnt += 1
                else:
                    limit_cnt = 0

                self.logger.debug("Limit count: {}".format(limit_cnt))
                self.logger.debug("Auto Exposure feature: {}".format(exposure_auto_feature.get()))
                last_exposure = exposure_time_feature.get()
                self.logger.debug("Current exposure: {}".format(last_exposure))

                if limit_cnt > 5:
                    self.logger.info(
                        "Auto exposure has run into limits. Continuing with exposure of: {exposure} ".format(
                            exposure=last_exposure))
                    exposure_auto_feature.set("Off")
                if iter >= max_iter:
                    try:
                        raise TimeoutError("Timeout while setting auto exposure!")
                    except NameError:
                        # Python 2 compatible Error
                        raise Exception("Timeout while setting auto exposure!")

                iter += 1

            self.triggerMode = trigger_mode_buffer
            self.logger.debug("Set exposure time to {exposure}"
                              "".format(exposure=last_exposure))

            return exposure_time_feature.get()

    #@vimba_context
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

    #@vimba_context
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
            return gain_feature.get()

    #@vimba_context
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
        with self.device as cam:
            fmt_feature = cam.get_feature_by_name("PixelFormat")
            if fmt is not None:
                self.logger.debug('Setting <PixelFormat> to {fmt}'
                                  ''.format(fmt=fmt))
                fmt_feature.set(fmt)

                # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
                try:
                    cam.GVSPAdjustPacketSize.run()

                    while not cam.GVSPAdjustPacketSize.is_done():
                        pass

                except (AttributeError, VimbaFeatureError):
                    pass

            return str(fmt_feature.get())

    #@vimba_context
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
        # TODO: Not perfectly clean since the settings may differ from self.triggerModeSetting
        if mode is None:
            return self.triggerModeSetting

        with self.device as cam:
            trigger_mode_feature = cam.TriggerMode
            trigger_selector_feature = cam.TriggerSelector
            trigger_source_feature = cam.TriggerSource
            trigger_activation_feature = cam.TriggerActivation
            self.logger.debug("Setting trigger mode to: {mode}".format(mode=mode))

            if isinstance(mode, str):
                if mode.lower() == 'in':
                    trigger_selector_feature.set('FrameStart')
                    trigger_source_feature.set('Line1')
                    trigger_activation_feature.set("RisingEdge")
                    trigger_mode_feature.set('On')

                    self.triggerModeSetting = 'in'
                elif mode.lower() == 'out':
                    # TODO: Implement out trigger for AVT cameras
                    self.triggerModeSetting = 'out'
                    raise NotImplementedError('Sending triggers is not'
                                              'implemented yet!')
                elif mode.lower() == 'off':
                    trigger_selector_feature.set('FrameStart')
                    trigger_source_feature.set('Freerun')
                    trigger_mode_feature.set('Off')

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
    cam_device = Camera('000F314E5F04')

    cam_device.exposure = 1500000
    # print("Before: ", cam_device.exposure)
    # exposure = cam_device.autoExposure()
    # print("After: ", cam_device.exposure)

    # Listing features of device
    if bListFeatures:
        cam_device.listFeatures()

    # Get an image
    # image = cam_device.getImage()
    # cv2.namedWindow('Captured image', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('Captured image', 1000, 1000)
    # cv2.imshow('Captured image', image)
    # cv2.waitKey()

    if bLiveView:
        cam_device._liveView()

    print(cam_device.pixelFormat)

    cam_device.setTriggerMode("off")
    number_of_images = 10
    print("Capturing {0} images".format(number_of_images))

    # images = cam_device.getImages(number_of_images)
    # print(f"Number of Images {len(images)}")
    # for i, img in enumerate(images):
    #     cv2.imwrite(rf"F:\Messwerte\Telezentrie\image{i}.png")

        # print('Showing image {i}'.format(i=i))
        # cv2.imshow('Captured image', img)
        # cv2.waitKey()

    cam_device.grabStart(10)
    grabbedImgs = cam_device.grabStop()

    for i, img in enumerate(grabbedImgs):
        print(f"Writing Image {i}")
        cv2.imwrite(rf"F:\Messwerte\Telezentrie\image{i}.png", img)

    cam_device.closeDevice()

    # contr.closeController()

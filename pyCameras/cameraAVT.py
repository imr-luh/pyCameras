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
from vimba.frame import FrameStatus
from vimba.error import VimbaFeatureError

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = None


def vimba_context(func):
    """
    Decorator which enables a function to be executed
    inside the vimba context when not needing the vimba variable itself.

    Parameters
    ----------
    func : Decorated function
    """

    def open_vimba_context(*args, **kwargs):
        with Vimba.get_instance():
            return func(*args, **kwargs)

    return open_vimba_context


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


class LiveViewHandler:
    def __init__(self):
        self.shutdown_event = threading.Event()

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
            cv2.resizeWindow(window_title, 900, 900)
            cv2.imshow(window_title, frame.as_opencv_image())

        cam.queue_frame(frame)


class Grabber(threading.Thread):
    def __init__(self, camera, frame_handler):
        super(Grabber, self).__init__()
        self._stop_event = threading.Event()
        self._cam = camera
        self._frame_handler = frame_handler

    def run(self):
        # Start vimba and camera contexts
        with Vimba.get_instance():
            with self._cam as cam:
                cam.start_streaming(handler=self._frame_handler, buffer_count=10)
                # Keep context alive until grabber is stopped
                while not self._stop_event.is_set():
                    pass

    def stop(self):
        self._stop_event.set()


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

        with Vimba.get_instance() as vimba:
            self.device = vimba.get_camera_by_id(device_handle)

        # Sets package sizes and transfer rates for GigE cameras
        # TODO: How to set transfer rates for already running cameras?
        self._setup_transfer_sizes()

        self.device_handle = device_handle

        self.triggerModeSetting = 'off'

        # Register AVT specific functions.
        # Function to set pixel format
        self.registerFeature('pixelFormat', self.setFormat)
        self.registerFeature('pixelType', self.setFormat)
        self.registerFeature('format', self.setFormat)

        self._frame_handler = None
        self._num_imgs = None
        self._grabber = None
        self.imgData = []

        # Init data type LUT for each PixelFormat
        self.imageFormatLUT = {'Mono8': np.uint8, 'Mono12': np.uint16}

    def __del__(self):
        pass

    @vimba_context
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

    @vimba_context
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
            # TODO: There seems to be a problem with trashed frames. This loop prevents returning incomplete images.
            while frame.get_status() == FrameStatus.Incomplete:
                frame = cam.get_frame(timeout_ms=2000)
            img = frame.as_numpy_ndarray()

        return img

    def prepareRecording(self, num):
        """ Sets the camera to MultiFrame mode and prepares frames. Use with
        "record()"-function.

        Parameters
        ----------
        num : int
            number of frames to be captured during acquisition
        """
        self._num_imgs = num

    @vimba_context
    def record(self):
        """ Blocking image acquisition, ends acquisition when num frames are
        captured, where num is set by "prepareRecording(num)". Only use with
        "prepareRecording(num)".

        Returns
        -------
        imgData : list
            List of images
        """
        if self._num_imgs is None:
            raise ValueError("prepareRecording has to be called before using record!")

        self.imgData = list()
        with self.device as cam:
            for frame in cam.get_frame_generator(limit=self._num_imgs, timeout_ms=2000):
                # TODO: First frame seems to be incomplete...
                self.imgData.append(frame.as_numpy_ndarray())

        return copy.deepcopy(self.imgData)

    def grabStart(self):
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
        self._grabber = Grabber(self.device, self._frame_handler)
        self._grabber.start()

    def grabStop(self):
        """
        Stop grabbing images and return camera to continuous mode.
        """
        # Reset thread
        self._grabber.stop()
        self._grabber.join()
        self._grabber = None

        # Get data and clear frame handler
        self.imgData = self._frame_handler.get_images()
        self._frame_handler = None

        return self.imgData

    @vimba_context
    def _liveView(self):
        """
        Live image stream an visualization through OpenCV window

        Leave _liveView by pressing "q"
        """
        handler = LiveViewHandler()
        with self.device as cam:
            try:
                cam.start_streaming(handler=handler, buffer_count=10)
                handler.shutdown_event.wait()
            finally:
                cam.stop_streaming()

    @vimba_context
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

    @vimba_context
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

    @vimba_context
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

    @vimba_context
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

    @vimba_context
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

    @vimba_context
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
        with self.device as cam:
            fmt_feature = cam.PixelFormat
            if fmt is not None:
                self.logger.debug('Setting <PixelFormat> to {fmt}'
                                  ''.format(fmt=fmt))
                fmt_feature.set(fmt)
            return fmt_feature.get()

    @vimba_context
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
                    trigger_mode_feature.set('On')
                    trigger_source_feature.set('Line1')
                    trigger_selector_feature.set('FrameStart')
                    trigger_activation_feature.set("RisingEdge")

                    self.triggerModeSetting = 'in'
                elif mode.lower() == 'out':
                    # TODO: Implement out trigger for AVT cameras
                    self.triggerModeSetting = 'out'
                    raise NotImplementedError('Sending triggers is not'
                                              'implemented yet!')
                elif mode.lower() == 'off':
                    trigger_mode_feature.set('Off')
                    trigger_source_feature.set('Freerun')
                    trigger_selector_feature.set('FrameStart')

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
        with self.device as cam:
            return cam.get_name()


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

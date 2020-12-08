#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Nils Melchert, Tim Betker"
__copyright__ = "Copyright 2017, LUH: IMR"
# __license__ = ""
__version__ = "0.1"
__maintainer__ = "Nils Melchert"
__email__ = "nils.melchert@imr.uni-hannover.de"
__status__ = "alpha"
__package_name__ = "MicroEnable4"
__developer__ = __author__

'''
Based on the SDK-Wrapper from the framegrabber card microEnableIV from Silicon Software
'''
import logging
import os
import sys
import numpy as np
import cv2
import time

try:
    sys.path.append(os.path.join(os.environ['SISODIR5'],
                                 'SDKWrapper/PythonWrapper/python35/lib/'))
    sys.path.append(os.path.join(os.environ['SISODIR5'],
                                 'SDKWrapper/PythonWrapper/python35/bin/'))

    import SiSoPyInterface as s
except (ImportError, KeyError):
    raise ImportError('SiSo module not loaded successfully')

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

MAX_ITER = 10000000

LOGGING_LEVEL = logging.DEBUG


class Controller(ControllerTemplate):
    def __init__(self, maxNrOfboards=10):
        super(Controller, self).__init__()
        self.logger = logging.getLogger(__name__)

        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.device_handles = []
        self.maxNrOfboards = maxNrOfboards
        self.logger.debug('Starting microEnable4 camera controller')

    def __repr__(self):
        return "<MicroEnable Frame Grabber Controller>"

    # returns count of available boards
    def _getNrOfBoards(self):
        nrOfBoards = 0
        (err, buffer, buflen) = s.Fg_getSystemInformation(None,
                                                          s.INFO_NR_OF_BOARDS,
                                                          s.PROP_ID_VALUE, 0)
        if (err == s.FG_OK):
            nrOfBoards = int(buffer)
        return nrOfBoards

    def updateDeviceHandles(self):
        """
        Refresh the list of available devices
        """
        self.logger.debug('Searching for frame grabber devices')

        nrOfBoardsFound = 0
        nrOfBoardsPresent = self._getNrOfBoards()

        for i in range(self.maxNrOfboards):
            if s.Fg_getBoardType(i) == s.PN_MICROENABLE4VD4CL:
                self.device_handles.append(i)
                nrOfBoardsFound += 1

            if nrOfBoardsFound >= nrOfBoardsPresent:
                break

        self.logger.debug(
            'Found {num} frame grabber device(s): {devices}'.format(
                num=len(self.device_handles),
                devices=self.device_handles))

    def closeController(self):
        self.logger.info("ME4-VD4 Controller shutdown")


class Camera(CameraTemplate):
    """
    microEnable frame grabber implementation based on the Silicon Software SDK
    to work with cameras with camera link standard in FULL configuration.
    """
    def __init__(self, device_handle, applet='Acq_FullAreaGray8'):
        """
        Implementation of the microEnable4-VD4 framegrabber.
        Launches the camera in freerun mode (triggerMode 'off').

        # TODO: Init camera with previous applet or store settings?

        Parameters
        ----------
        device_handle : int
            Framegrabber device handle to identify the frame grabber

        applet : str
            String defining the used applet. Settings may differ between
            different applets
        """
        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        if s.Fg_getBoardType(device_handle) != s.PN_MICROENABLE4VD4CL:
            self.logger.error(
                'Board {0} is not supported by this package!'.format(
                    s.Fg_getBoardNameByType(s.Fg_getBoardType(device_handle),
                                            s.Fg_getBoardType(device_handle))))
            raise TypeError("Board {board} is not supported by this package!"
                            "".format(board=s.Fg_getBoardNameByType(s.Fg_getBoardType(device_handle),
                                            s.Fg_getBoardType(device_handle))))
        self._applet = applet
        self.logger.debug('Initializing Framegrabber...')

        self.device = s.Fg_InitEx(self._applet, device_handle, 0)

        # error handling
        err = s.Fg_getLastErrorNumber(self.device)
        if err < 0:
            msg = s.Fg_getErrorDescription(err)
            self.logger.error("Error", err, ":", msg)
            raise _MicroEnableException(err)
        else:
            self.logger.debug("Grabber initialized successfully!")

        self._clser_ref = s.clSerialInit(0)
        # Setting Camera link factory profile
        # Resolution: 1280x1024, Image freq.: 430, Mode: 8x8, CL-Conf.: FULL
        factory_profile = ':f7'
        self._clSerialWrite(command=factory_profile)

        s.Fg_setParameterWithInt(self.device, s.FG_BITALIGNMENT,
                                 s.FG_LEFT_ALIGNED, self.device_handle)
        s.Fg_setParameterWithInt(self.device, s.FG_GEN_ENABLE, s.FG_CAMPORT,
                                 self.device_handle)
        s.Fg_setParameterWithInt(self.device, s.FG_TRIGGER_LEGACY_MODE,
                                 s.FG_ON, self.device_handle)
        self.setTriggerMode('off')

        self._width = \
            s.Fg_getParameterWithInt(self.device, s.FG_WIDTH,
                                     self.device_handle)[1]
        self._height = \
            s.Fg_getParameterWithInt(self.device, s.FG_HEIGHT,
                                     self.device_handle)[1]

        self._free_run_buffer = self._prepareImageBuffer(10)

        self._pics_to_be_recorded = None
        self._buffer_handle = None
        self._img_list = list()

        self._apc_data = None

    def __del__(self):
        """
        Frees memory and grabber when closing.
        """
        s.Fg_FreeMemEx(self.device, self._free_run_buffer)
        if self._buffer_handle is not None:
            s.Fg_FreeMemEx(self.device, self._buffer_handle)
        s.clSerialClose(self._clser_ref[1])
        self.closeDevice()

    def _prepareImageBuffer(self, numImages=1):
        """
        Prepares a bigger buffer for e.g. more than 1 triggered image.

        Notes: This function uses a sleep function to guarantee allocated memory.

        Parameters
        ----------
        numImages : int
            Number of expected images.

        Returns
        -------
            buffer: memory_handle
                Memory handle which adresses the allocated buffer memory
        """
        # Calculate buffer size
        # TODO: bytePerSample may have to be changed when using different pixel formats
        samplePerPixel = 1
        bytePerSample = 1
        nbBuffers = numImages
        totalBufferSize = self._width * self._height * samplePerPixel * \
                          bytePerSample * nbBuffers
        buffer = s.Fg_AllocMemEx(self.device, totalBufferSize, nbBuffers)
        # Give time to allocate buffer -
        # tests without sleep/too short sleep failed
        time.sleep(.5)

        return buffer

    def _freeLiveBuffer(self):
        """
        Frees allocated memory of self.free_run_buffer. Sets handle to None.
        """
        s.Fg_FreeMemEx(self.device, self._free_run_buffer)
        self._free_run_buffer = None

    def _freeImageBuffer(self):
        """
        Frees allocated frame buffers and sets self.buffer_handle to None.
        """
        s.Fg_FreeMemEx(self.device, self._buffer_handle)
        self._buffer_handle = None

    def _getParamWithInt(self, parameter):
        """
        Short form of Fg_getParameterWithInt to reduce line length
        (since self.device and self.device_handle are always passed).

        Parameters
        ----------
        parameter : silicon software enum
            Defines the parameter, which should be read.

        Returns
        -------
        value : int
            Value of requested parameter.
        """
        err, retval = s.Fg_getParameterWithInt(self.device, parameter,
                                               self.device_handle)
        if err != s.FG_OK:
            raise _MicroEnableException(err)
        return retval

    def _getParamWithDouble(self, parameter):
        """
        Short form of Fg_getParameterWithInt to reduce line length
        (since self.device and self.device_handle are always passed).

        Parameters
        ----------
        parameter : silicon software enum
            Defines the parameter, which should be read.

        Returns
        -------
        value : double
            Value of requested parameter.
        """
        err, retval = s.Fg_getParameterWithDouble(self.device, parameter,
                                               self.device_handle)
        if err != s.FG_OK:
            raise _MicroEnableException(err)
        return retval

    def _setParamWithInt(self, parameter, value):
        """
        Short form of Fg_setParameterWithInt to reduce line length
        (since self.device and self.device_handle are always passed).

        Parameters
        ----------
        parameter : silicon software enum
            Defines the parameter, which should be read.

        value : int
            Value of parameter.
        """
        # Set retval to value which is not expected to be set a value
        # to ensure at least one iteration through the while loop
        retval = -99
        iterations = 0
        s.Fg_setParameterWithInt(self.device, parameter, value,
                                 self.device_handle)
        while retval != value:
            retval = self._getParamWithInt(parameter)
            if iterations > MAX_ITER:
                raise TimeoutError(
                    "Max iterations reached while waiting to set parameter!")
            iterations += 1
        # Sleep to fully apply setting... there have been timing issues...
        time.sleep(.1)

    def _setParamWithDouble(self, parameter, value):
        """
        Short form of Fg_setParameterWithInt to reduce line length
        (since self.device and self.device_handle are always passed).

        Parameters
        ----------
        parameter : silicon software enum
            Defines the parameter, which should be read.

        value : double
            Value of parameter.
        """
        # Set retval to value which is not expected to be set a value
        # to ensure at least one iteration through the while loop
        retval = -99
        iterations = 0
        s.Fg_setParameterWithDouble(self.device, parameter, value,
                                 self.device_handle)
        while retval != value:
            retval = self._getParamWithDouble(parameter)
            if iterations > MAX_ITER:
                raise TimeoutError(
                    "Max iterations reached while waiting to set parameter!")
            iterations += 1
        # Sleep to fully apply setting... there have been timing issues...
        time.sleep(.1)

    def _clSerialWrite(self, command):
        self.logger.debug(
            'Setting command <{0}> on camera via camera link serial interface'.format(
                command))
        s.clSerialWrite(self._clser_ref[1], command, sys.getsizeof(command),
                        100)

    # TODO: Implement the serial read functionality.
    # TODO:(Seems like Silicon Software did some mistakes wrapping the function)

    @staticmethod
    def listDevices():
        """
        List available ME4-VD4 frame grabbers
        Returns
        -------
        cams : list
            List of available grabber devices
        """
        return Controller().listDevices()

    def openDevice(self):
        """
        Opens a camera device
        """
        if not self.isOpen():
            self.logger.debug('Creating grabber object')
            self.device = s.Fg_InitEx(self._applet, self.device_handle, 0)
        else:
            self.logger.debug('Grabber object already exists')

    def closeDevice(self):
        """
        Closes camera device
        """
        self.logger.debug('Freeing the framegrabber device...')

        retval = s.Fg_FreeGrabber(self.device)
        if retval == s.FG_OK:
            self.device = None
            self.logger.debug('Freed successfully!')
        else:
            raise _MicroEnableException(retval)

    def isOpen(self):
        """
        Check if the device for this instance is currently open and ready to
        communicate

        Returns
        -------
        bool
            True if the camera connection is open, False if it is not
        """
        # ME4-VD4 frame grabbers do not have any isOpen-function by itself.
        # Assuming that if there is a device given in self.device, device is opened.
        if self.device is not None:
            return True
        else:
            return False

    def getImage(self, *args, **kwargs):
        """
        Get an image from the camera device

        *args and **kwargs are ignored parameters!

        This function uses self.free_run_buffer to write images to.

        Returns
        -------
        img : np.ndarray
            Current camera image
        """
        s.Fg_AcquireEx(self.device, self.device_handle, s.GRAB_INFINITE,
                       s.ACQ_STANDARD, self._free_run_buffer)

        cur_img_no = 0
        iterations = 0

        while cur_img_no == 0:
            cur_img_no = s.Fg_getLastPicNumberEx(self.device,
                                                 self.device_handle,
                                                 self._free_run_buffer)
            if iterations > MAX_ITER:
                raise TimeoutError(
                    "Max iterations reached while waiting for image! Missing a trigger signal?")
            iterations += 1

        img = s.Fg_getImagePtrEx(self.device, cur_img_no, self.device_handle,
                                 self._free_run_buffer)

        np_img = s.getArrayFrom(img, self._width, self._height)

        s.Fg_stopAcquireEx(self.device, self.device_handle,
                           self._free_run_buffer, s.STOP_ASYNC)

        return np_img.copy()

    def prepareRecording(self, num):
        """
        Prepare the camera to recorded the given number of images by setting
        up a frame buffer or doing other camera specific actions. The actual
        recording is done in self.record() and should be blocking until the
        desired number of images is recorded.

        Sets up the frame buffer and stores the handle in self.buffer_handle.

        Parameters
        ----------
        num : int
            Number of images that should be recorded
        """
        self._pics_to_be_recorded = num
        self._img_list = []

        self._buffer_handle = self._prepareImageBuffer(num)

    def record(self):
        """
        Blocking image acquisition of a previously defined number of images
        (see prepareRecording).

        Returns
        -------
        imgs : list
            List of numpy arrays containing the recorded images
        """
        s.Fg_AcquireEx(self.device, self.device_handle,
                       self._pics_to_be_recorded, s.ACQ_STANDARD,
                       self._buffer_handle)

        iterations = 0
        while s.Fg_getStatusEx(self.device, s.NUMBER_OF_GRABBED_IMAGES, 0,
                               self.device_handle,
                               self._buffer_handle) != self._pics_to_be_recorded:
            if iterations > MAX_ITER:
                raise TimeoutError('Maximum number of iterations reached. '
                                   'Missing a trigger signal?')
            iterations += 1

        s.Fg_stopAcquireEx(self.device, self.device_handle, self._buffer_handle,
                           s.STOP_ASYNC)

        for img_no in range(self._pics_to_be_recorded):
            img = s.Fg_getImagePtrEx(self.device, img_no + 1,
                                     self.device_handle, self._buffer_handle)
            self._img_list.append(
                s.getArrayFrom(img, self._width, self._height).copy())

        # Free buffer and set handle to None
        self._freeImageBuffer()

        self._pics_to_be_recorded = None

        return self._img_list

    def grabStart(self):
        """
        Start grabbing images in a non-blocking way and store those images in
        an internal variable

        See Also
        --------
        self.grabStop()
        """
        # Register apc control and callback function (see below this class)
        # Used to control asynchronous image acquisition
        apcCtrl = s.FgApcControl(5, s.FG_APC_DEFAULTS)
        self._apc_data = _MyApcData(self.device, self.device_handle,
                                    self._free_run_buffer,
                                    (self._width, self._height))
        s.setApcCallbackFunction(apcCtrl, _frameCallback, self._apc_data)
        err = s.Fg_registerApcHandler(self.device, self.device_handle,
                                      apcCtrl, s.FG_APC_CONTROL_BASIC)
        if err != s.FG_OK:
            raise _MicroEnableException(err)

        err = s.Fg_AcquireEx(self.device, self.device_handle, s.GRAB_INFINITE,
                             s.ACQ_STANDARD, self._free_run_buffer)
        if err != s.FG_OK:
            raise _MicroEnableException(err)

    def grabStop(self):
        """
        Stop grabbing images and return the images that have been recorded

        See Also
        --------
        self.grabStart()
        """
        # Unregister apc handler
        s.Fg_registerApcHandler(self.device, self.device_handle, None,
                                s.FG_APC_CONTROL_BASIC)

        s.Fg_stopAcquireEx(self.device, self.device_handle,
                           self._free_run_buffer,
                           s.STOP_ASYNC)
        # Get image data from data class
        self._img_list = self._apc_data.img_list

        return self._img_list

    def _liveView(self):
        """
        Live image stream an visualization through OpenCV window.

        Leave _liveView by pressing "q"
        """
        cv2.startWindowThread()
        cv2.namedWindow("IMG", 2)
        cv2.resizeWindow("IMG", 900, 900)

        s.Fg_AcquireEx(self.device, self.device_handle, s.GRAB_INFINITE,
                       s.ACQ_STANDARD, self._free_run_buffer)

        last_img = -1
        while True:
            cur_img_no = -1
            iterations = 0
            # Only refresh if new image is acquired
            while last_img == cur_img_no or cur_img_no <= 0:
                cur_img_no = s.Fg_getLastPicNumberEx(self.device,
                                                     self.device_handle,
                                                     self._free_run_buffer)
                if iterations > MAX_ITER:
                    raise TimeoutError(
                        "Max iterations reached while waiting for image! Missing a trigger signal?")
                iterations += 1
            last_img = cur_img_no

            img_data = s.Fg_getImagePtrEx(self.device, cur_img_no,
                                          self.device_handle,
                                          self._free_run_buffer)
            # Convert to numpy array
            live_img = s.getArrayFrom(img_data, self._width, self._height)

            cv2.imshow("IMG", live_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                cv2.destroyAllWindows()
                break

        # Cleanup
        s.Fg_stopAcquireEx(self.device, self.device_handle,
                           self._free_run_buffer, s.STOP_ASYNC)
        return

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
            self.logger.debug(
                'Setting exposure time to {microns} micro seconds'.format(
                    microns=microns))
            # TODO: Make sure the exposure time is less than (frame_rate)^-1

            self._setParamWithInt(s.FG_EXPOSURE, microns)
        if self.getFeature('TriggerMode') in ('in', 'off'):
            return self._getParamWithInt(s.FG_EXPOSURE)
        else:
            raise NotImplementedError(
                'Requesting exposure time for a trigger mode that is not implemented')

    def setResolution(self, resolution=None):
        """
        Set the resolution of the camera to the given values in pixels or read
        the current resolution by passing None
        This is actually not the real camera resolution but only the amount of pixels
        considered by the frame grabber.

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
            # Free live buffer to adjust to new resolution
            self._freeLiveBuffer()
            self.logger.debug(
                'Setting <Width> to {width}'.format(width=resolution[0]))
            self._setParamWithInt(s.FG_WIDTH, resolution[1])

            self.logger.debug(
                'Setting <Height> to {height}'.format(height=resolution[1]))
            self._setParamWithInt(s.FG_HEIGHT, resolution[1])

            self._width = self._getParamWithInt(s.FG_WIDTH)
            self._height = self._getParamWithInt(s.FG_HEIGHT)

            # Prepare new live buffer with new resolution
            self._free_run_buffer = self._prepareImageBuffer(10)

        return self._width, self._height

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
        # TODO: Implement when serial read works fine!!
        # There is a gain setting from the frame grabber.
        # Camera gain would be much more fitting here...
        self._clSerialWrite()
        raise NotImplementedError('Implement me!!!')

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
        # TODO: Implement
        raise NotImplementedError('Implement me!!!')

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
                # Setting camera to external trigger mode
                self._clSerialWrite(':h1')
                self._setParamWithInt(s.FG_TRIGGERMODE, s.ASYNC_TRIGGER)
                self._setParamWithInt(s.FG_EXSYNCON, s.FG_ON)
                self._setParamWithInt(s.FG_EXSYNCPOLARITY, s.FG_HIGH)
                self._setParamWithInt(s.FG_TRIGGERIN_SRC, s.TRGINSRC_0)
                self.triggerModeSetting = 'in'

                time.sleep(.2)

            elif mode.lower() == 'out':
                # TODO: Implement out trigger
                self.triggerModeSetting = 'out'
                raise NotImplementedError(
                    'Sending triggers is not implemented yet!')
            elif mode.lower() == 'off':
                # Setting camera to freerun mode
                self._clSerialWrite(':h1')
                self._setParamWithInt(s.FG_TRIGGERMODE, s.GRABBER_CONTROLLED)
                self._setParamWithInt(s.FG_EXSYNCON, s.FG_ON)
                self._setParamWithInt(s.FG_EXSYNCPOLARITY, s.FG_HIGH)
                self._setParamWithInt(s.FG_TRIGGERIN_SRC, s.TRGINSRC_0)
                self.triggerModeSetting = 'off'

                time.sleep(.2)

            else:
                raise ValueError(
                    'Unexpected value in setTriggerMode. '
                    'Expected "in", "out", or "off". Got {mode}'.format(
                        mode=mode))
            return self.triggerModeSetting
        else:
            raise TypeError(
                'Trigger Mode should be '
                'None, "in", "out", or "off". Got {mode}'.format(
                    mode=mode))

    def setTriggerDelay(self, microns=None):
        """
        Setting a delay so that the camera acquires an image after a given delay in microseconds. Not that the trigger
        mode has to be set to 'in' in order to be able to set the trigger delay.
        If no parameter is specified, the camera returns its current trigger delay.

        Parameters
        ----------
        microns: double
            Delay specified in micro seconds

        Returns
        -------
        delay: double
            Current trigger delay in microseconds

        """
        if not microns:
            return self._getParamWithDouble(s.FG_EXSYNCDELAY)
        if not self.setTriggerMode() == "in":
            raise ValueError("Triggermode is not activated. Please set to triggermode of the camera to \"in\". Current trigger mode is {mode}".format(mode=self.setTriggerMode()))
        if microns < 0 or microns > 10230:
            raise AttributeError("Trigger delay is out of range. Value \"microns\" must be in range 0 - 10230")
        self.logger.debug("Setting trigger delay to {val}".format(val=microns))
        self._setParamWithDouble(s.FG_EXSYNCDELAY, microns)

    def _setRoiOffset(self, offset=None):
        """
        Sets the offset of the current region of interest (ROI) of the camera

        Parameters
        ----------
        offset : tuple
            X and Y offsets of the acquisition ROI

        Returns
        -------
        offset : tuple
            The set ROI-offset after applying the passed value
        """
        if offset is not None:
            self.logger.debug(
                'Setting <X-ROI-Offset> to {0}'.format(offset[0]))
            self._setParamWithInt(s.FG_XOFFSET, offset[0])
            self.logger.debug(
                'Setting <Y-ROI-Offset> to {0}'.format(offset[1]))
            self._setParamWithInt(s.FG_YOFFSET, offset[1])

        x_offset = self._getParamWithInt(s.FG_XOFFSET)
        y_offset = self._getParamWithInt(s.FG_YOFFSET)

        return x_offset, y_offset


class _MicroEnableException(Exception):
    def __init__(self, err):
        super(_MicroEnableException, self).__init__()
        self.message = 'Error {err}: {msg}' \
                       ''.format(err=err, msg=s.Fg_getErrorDescription(err))

    def __str__(self):
        return self.message


class _MyApcData:
    """
    Required class for async image acquisition. Stores images.
    """
    def __init__(self, fg, port, mem, resolution):
        self.fg = fg
        self.port = port
        self.mem = mem
        self.width, self.height = resolution
        self.img_list = []


def _frameCallback(imgNr, userData):
    """
    Callback function which will be used for asynchronous acquisition.
    """
    img_data = s.Fg_getImagePtrEx(userData.fg, imgNr, userData.port, userData.mem)
    img = s.getArrayFrom(img_data, userData.width, userData.height)
    userData.img_list.append(img.copy())

    return 0


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)
    # Camera.listDevices()
    MicroEnable4 = Camera(0)
    #
  #  MicroEnable4.setExposureMicrons(1500)
    # MicroEnable4.setExposureMicrons(7300)
    #MicroEnable4.setExposureMicrons(17000)
    MicroEnable4.setExposureMicrons(2300)
    # MicroEnable4.setTriggerMode('in')


    # MicroEnable4.grabStart()
    # time.sleep(2)
    # imgs = MicroEnable4.grabStop()
    MicroEnable4.setTriggerMode('off')
    #

    # MicroEnable4.prepareRecording(5)
    # imgs = MicroEnable4.record()

    # imgs = MicroEnable4.getImages(2000)


    # imgs = MicroEnable4.getImages(10)
    # print('num images:', len(imgs))
    # for i, img in enumerate(imgs):
    #     print(i)
    #     cv2.imshow('img_test', img)
    #     cv2.waitKey(0)
        # cv2.imwrite("/home/tx90/Data/lci_data/images/lci"+str(i)+".png", img)


    # MicroEnable4.grabStart()
    # time.sleep(30)
    # imgs = MicroEnable4.grabStop()
    # for i, img in enumerate(imgs):
    #     print(i)
    #     cv2.imshow('img_test', img)
    #     cv2.waitKey(0)

    MicroEnable4._liveView()


    # img = MicroEnable4.getImage()
    # img = MicroEnable4.getImage()
    # cv2.imshow("sd", img)
    # cv2.waitKey(0)

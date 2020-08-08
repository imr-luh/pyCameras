__author__ = "Philipp Middendorf"
__credits__ = "Kolja Hedrich, Pascal Kern"
__maintainer__ = "Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

# check this repository for implementation https://github.com/AlexJinlei/python-v4l2
import logging
import signal
from abc import ABC
import numpy as np
import v4l2
import fcntl
import errno
import mmap
import select
import cv2
import matplotlib.pyplot as plt
import os
import time
import colour_demosaicing
from threading import Event
import sys
import copy
from skimage import color
from func_timeout import func_timeout
from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate
from typing import List, Tuple, Union, BinaryIO
import cython

LOGGING_LEVEL = logging.DEBUG

class Controller(ControllerTemplate):
    """ Camera Controller for the OV2740 camera."""

    def __init__(self) -> None:
        super(Controller, self).__init__()
        logging.basicConfig()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.logger.info("Starting OV2740 Camera Controller")
        self.device_handles: List[str] = []

    def updateDeviceHandles(self) -> int:
        """
        Updates the list of available device handles

        """
        self.logger.info("searching camera devices")

        cam_name = "CX3-UVC"
        v4l2path = "/sys/class/video4linux"

        for item in os.listdir(v4l2path):
            pth = os.path.join(v4l2path, item, "name")
            if os.path.exists(pth):
                with open(pth, "r") as f:
                    device_name = f.read()
                    if cam_name in device_name:
                        device_input = str(item)
                        self.device_handles.append(device_input)

        return 1

    def getDevice(self, device_handle: object) -> object:
        """
          Open the actual capture device to grab images

          Parameters
          ----------
          device_handle
              One entry of the device list returned by self.listDevices()


          :return: Camera(device_handle) : Camera object
            The capture device of type cameraRealsense
          """
        self.logger.debug(f'Opening device {device_handle}')
        try:
            return Camera(device_handle)
        except Exception as e:
            self.logger.exception(f'Failed to open the camera device: {e}')
            raise

    def __del__(self) -> None:
        """
        Camera Controller Destructor

        """
        self.logger.debug(f"Deleting Camera Controller {self}")
        self.device_handles = []

    def __repr__(self) -> str:
        """
        :return: str
        """
        return "<OV2740 Camera Controller>"


class Camera(CameraTemplate, ABC):
    """
        Capture device representing a OV2740
    """

    def __init__(self, device_handle: object) -> None:
        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        logging.basicConfig()
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.device_handle = device_handle

        if self.device_handle is None:
            raise ConnectionError

        self.img_data: List[np.ndarray] = []
        self._expected_images: int  = 0
        self.ExposureMicrons: int = 0
        self.TriggerMode: str = ""
        self.buffers: List[object] = []
        self.buf: v4l2.v4l2_buffer = v4l2.v4l2_buffer()
        self.buf_type: int = 0
        self.acquisition_calls: int = 0
        self.imageMode: str = ""
        self.streamingMode: bool = False
        self.openDevice()
        self.cameraImages: List[np.ndarray] = []
        self.measurementMode: bool = True
        self.registerFeatures()
        self.getCapability()
        self.recording_time: float = 0
        self.ImageWidth: int = 0
        self.ImageHeight: int  = 0

    @staticmethod
    def listDevices() -> list:
        """
        List available CX3 camera devices

        Controller :return: self.device_handles : list
        List of camera device handles that can be used to create a new camera instance
        """
        return Controller().listDevices()

    def registerFeatures(self) -> None:
        """
        Registration of shared features that should be the same for all camera
        implementations. E.g. ExposureMicrons, Resolution, Gain, Format and
        TriggerMode
        """
        self.logger.debug('Registering camera features')
        self.registerFeature('ExposureInfo', self.getExposureInfo)
        self.registerFeature('Resolution', self.setResolution)
        self.registerFeature('Format', self.getFormat)
        self.registerFeature('TriggerMode', self.setTriggerMode)
        self.registerFeature('Trigger', self.setTriggerMode)
        self.registerFeature('Framerate', self.setFramerate)
        self.registerFeature('Capability', self.getCapability)
        self.registerFeature('ImageMode', self.setImageMode)

    def getCapability(self) -> object:
        try:
            cp = v4l2.v4l2_capability()
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYCAP), cp)

        except Exception as e:
            self.logger.exception(f"Failed to get sensor capability: {e}")

        return cp

    def setResolution(self, resolution: Tuple[int, int] = (1928, 1088)) -> Tuple[int,int]:
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_FMT), fmt)  # get current settings
        self.ImageWidth = fmt.fmt.pix.width
        self.ImageHeight = fmt.fmt.pix.height
        return self.ImageWidth, self.ImageHeight

    def setImageMode(self, Mode: str = 'Raw') -> None:
        self.imageMode = Mode

    def getExposureInfo(self) -> Tuple[v4l2.v4l2_queryctrl, v4l2.v4l2_control]:
        try:
            # query control
            qc = v4l2.v4l2_queryctrl()
            qc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYCTRL), qc)

            # get control value
            gc = v4l2.v4l2_control()
            gc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_CTRL), gc)
            self.ExposureMicrons = gc.value * 100  # v4l2 compensation

            self.logger.debug(
                f'Min exposure time {qc.minimum * 100}us, max {qc.maximum * 100}us, default {qc.default * 100}us, Steps {qc.step * 100}us, current {gc.value * 100}us')

        except Exception as e:
            self.logger.exception(f"Failed to get ExposureInfo: {e}")

        return qc, gc

    def openDevice(self) -> None:
        """
        Open the device for capturing images.

        :return:
        """
        try:
            self.device: BinaryIO = open(str('/dev/' + str(self.device_handle)), 'rb+', buffering=0)
            self.logger.debug(f'Opened camera device: {self.device.name}')
        except Exception as e:
            self.logger.exception(f"Failed to open the camera device: {e}")

    def getFormat(self) -> str:
        """
        read the current format

        Returns
        -------
        fmt : str
            The image format after applying the passed value
        """
        try:
            fmt = v4l2.v4l2_format()
            fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_FMT), fmt)  # get current settings

        except Exception as e:
            self.logger.exception(e)

        return fmt

    def closeDevice(self) -> None:
        """
        Close the connection to the device
        """
        if self.device is not None:
            try:
                self.logger.debug('Closing camera device: {self.device}')
                del self.device

            except Exception as e:
                self.logger.exception(f"Failed to close the camera device: {e}")
        else:
            self.logger.info('No Device present.')

    def setTriggerMode(self, mode: str = None) -> str:
        """
        Set the trigger mode of the camera to either "in", "out", or "off", or
        read the current trigger setting by passing None

        """
        if mode is None:
            # self.TriggerMode = mode
            return self.TriggerMode

        elif mode.lower() == 'in':
            self.TriggerMode = mode
            self.logger.warning(f"Trigger mode not implemented: {mode}")

        elif mode.lower() == 'out':
            self.logger.info("Camera Out Trigger on")
            self.TriggerMode = mode

        elif mode.lower() == 'off':
            self.TriggerMode = mode
            self.logger.warning(f"Trigger mode not implemented: {mode}")

        return self.TriggerMode

    def prepareRecording(self, num: int = 1) -> None:
        if self.streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self.streamingMode = False
        if self.measurementMode:
            self._expected_images = num * 2 + 2
        else:
            self._expected_images = num

        self.buffers = []
        self.logger.info(f"Prepare recording {self._expected_images} images")
        try:  # empty buffer, otherwise all frames are identical

            req = v4l2.v4l2_requestbuffers()
            req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            req.memory = v4l2.V4L2_MEMORY_MMAP
            req.count = 1  # nr of buffer frames
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_REQBUFS), req)  # tell the driver that we want some buffers

            for ind in range(req.count):
                # setup a buffer
                self.buf = v4l2.v4l2_buffer()
                self.buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
                self.buf.memory = v4l2.V4L2_MEMORY_MMAP
                self.buf.index = ind
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYBUF), self.buf)

                mm = mmap.mmap(self.device.fileno(), self.buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                               offset=self.buf.m.offset)
                self.buffers.append(mm)

                # queue the buffer for capture
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMON), self.buf_type)
            self.streamingMode = True

            t0 = time.time()
            max_t = 1
            ready_to_read: list = []
            ready_to_write: list = []
            in_error: list = []

            while len(ready_to_read) == 0 and time.time() - t0 < max_t:
                ready_to_read, ready_to_write, in_error = select.select([self.device], [], [], max_t)

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

        except Exception as e:
            self.logger.exception(f"Failed to prepare recording: {e}")

    def readBuffers(self) -> List[bytearray]:
        images = list()
        for index in range(0, self._expected_images):  # capture x frames

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue
            mm = self.buffers[self.buf.index]

            image_bytestream = mm.read()
            mm.seek(0)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

            image_byte_array = bytearray(image_bytestream)
            images.append(image_byte_array)
            index += 1
        return images

    def record(self) -> List[np.ndarray]:
        try:
            self.logger.info(f"Recording {self._expected_images} images")
            start = time.time()
            byteArrays = self.readBuffers()
            end = time.time()
            self.recording_time = end - start
            self.measurementMode = True
            self.cameraImages = []
            rawImages = []
            for bytearray in byteArrays:
                image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=bytearray).astype(np.uint16)
                rawImages.append(np.right_shift(image_array, 6).astype(np.uint16))

            if self.measurementMode:
                checkFFT = True
                if checkFFT:
                    startImage = self.checkfirstPhase(rawImages)

                    image_counter = 0
                    for i in range(startImage, len(rawImages)):
                        if image_counter % 2 == 0:
                            self.cameraImages.append(rawImages[i])
                        image_counter += 1
                else:
                    calibration_time = 3.66
                    measurement_time = 3.472
                    # if self.recording_time < measurement_time:
                    if self.recording_time < calibration_time:
                        for i in range(0, len(byteArrays)):
                            if i > 1 and i % 2 != 0:
                                self.cameraImages.append(rawImages[i])
                    else:
                        for i in range(0, len(byteArrays)):
                            if i > 1 and i % 2 == 0:
                                self.cameraImages.append(rawImages[i])
            else:
                for i in range(0, len(byteArrays)):
                    self.cameraImages.append(rawImages[i])

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self.streamingMode = False

            self.logger.info(f"capturing took: {self.recording_time}")

        except Exception as e:
            self.logger.exception(f"Failed record images: {e}")

        finally:
            return self.cameraImages

    def getImages(self, num: int = 1) -> List[np.ndarray]:
        try:
            self.prepareRecording(num)
            images = self.record()
            self.acquisition_calls += 1
        except Exception as e:
            self.logger.exception(f"Failed getImages: {e}")
        return images

    def getImage(self) -> np.ndarray:
        """
        Get an image from the camera

        :return: image : np.ndarray
            Image of camera device
        """
        if self.streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self.streamingMode = False

        rawImage: list = []
        self.buffers = []
        self.logger.debug("get single frame.")
        req = v4l2.v4l2_requestbuffers()
        req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = v4l2.V4L2_MEMORY_MMAP
        req.count = 1  # nr of buffer frames
        try:
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_REQBUFS), req)  # tell the driver that we want some buffers
        except OSError as e:
            if not e.errno == errno.EBUSY:
                raise

        for ind in range(req.count):
            # setup a buffer
            self.buf = v4l2.v4l2_buffer()
            self.buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            self.buf.memory = v4l2.V4L2_MEMORY_MMAP
            self.buf.index = ind

            try:
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYBUF),
                            self.buf)  # tell the driver that we want some buffers
            except OSError as e:
                if e.errno == errno.EBUSY:
                    continue
                else:
                    raise

            mm = mmap.mmap(self.device.fileno(), self.buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                           offset=self.buf.m.offset)
            self.buffers.append(mm)

            # queue the buffer for capture
            try:
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)
            except OSError as e:
                if e.errno == errno.EBUSY:
                    continue
                else:
                    raise

            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            try:
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMON), self.buf_type)
            except OSError as e:
                if e.errno == errno.EBUSY:
                    continue
                else:
                    raise

        self.streamingMode = True

        t0 = time.time()
        max_t = 1
        ready_to_read: list = []
        ready_to_write: list = []
        in_error: list = []

        while len(ready_to_read) == 0 and time.time() - t0 < max_t:
            ready_to_read, ready_to_write, in_error = select.select([self.device], [], [], max_t)

        try:
            # black/empty image
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

            # the one below seems to bee the problem
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue

        except OSError as e:
            if not e.errno == errno.EBUSY:
                raise

        mm = self.buffers[self.buf.index]

        image_bytestream = mm.read()
        mm.seek(0)
        try:
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
        except OSError as e:
            if not e.errno == errno.EBUSY:
                raise
        self.streamingMode = False

        byteArray = bytearray(image_bytestream)

        image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=byteArray).astype(np.uint16)
        image_array = np.right_shift(image_array, 6).astype(np.uint16)
        return image_array

    def blacklevelcorrection(self, image: np.ndarray, correction_type: str = "mean") -> np.ndarray:
        try:
            image = image
            cwd = '/home/middendorf/PycharmProjects/pyCameras/pyCameras/'
            path = os.path.join(cwd + '/images/black_level_mean.npy')
            if os.path.exists(path):
                if correction_type == "pixelwise":
                    blacklevel_image = np.load(path)
                elif correction_type == "mean":
                    ref_image = np.load(path)
                    mean = np.mean(ref_image)
                    blacklevel_image = np.ones_like(ref_image) * mean

                saturation_image = np.ones_like(image) * 1023
                if (image > 1000).sum() < 10000:
                    corrected_image = np.subtract(image, blacklevel_image).astype(np.int16)
                else:
                    corrected_image = np.where(np.subtract(image, blacklevel_image).astype(np.int16) < 0,
                                               np.subtract(saturation_image, blacklevel_image).astype(np.int16),
                                               np.subtract(image, blacklevel_image).astype(np.int16))
                blacklevel_factor = 1023 / (np.subtract(saturation_image, blacklevel_image))
                image = np.multiply(corrected_image.copy(), blacklevel_factor)
                image = np.clip(image, 0, 1023)

            else:
                raise Exception

            return image

        except Exception as e:
            self.logger.exception(f"Failed black level correction: {e}")

    def demosaicImage(self, raw_image: np.ndarray, colour: bool = False, bit: int = 8) -> np.ndarray:
        raw_image[0::2, 0::2] = np.multiply(raw_image[0::2, 0::2], 1.7)
        raw_image[1::2, 1::2] = np.multiply(raw_image[1::2, 1::2], 1.5)
        raw_image = np.clip(raw_image, 0, 1023)

        demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(raw_image, "BGGR")
        demosaic_img = np.clip(demosaic_img, 0, 1023)

        norm_image = demosaic_img.copy() / 1023

        if not colour:
            norm_image = color.rgb2gray(norm_image)

        if bit == 8:
            image = norm_image.copy() * 255
            image = image.astype(np.uint8)
        elif bit == 10:
            image = norm_image.copy() * 1023
            image = image.astype(np.uint16)
        else:
            image = norm_image
        return image

    def postProcessImages(self, raw_images: Union[List[np.ndarray], np.ndarray], colour: bool = False, bit: int = 8,
                          blacklevelcorrection: bool = False) -> Union[List[np.ndarray], np.ndarray]:
        self.logger.debug('Image post processing')
        try:
            # i = 0
            images = list()
            if isinstance(raw_images, list):
                for raw_image in raw_images:
                    # i += 1
                    if blacklevelcorrection:
                        raw_image = self.blacklevelcorrection(image=raw_image, correction_type="mean")

                    image = self.demosaicImage(raw_image=raw_image, colour=colour, bit=bit)
                    # cv2.imwrite(str(i)+".png", image)
                    images.append(image)

            else:
                if blacklevelcorrection:
                    raw_images = self.blacklevelcorrection(image=raw_images, correction_type="mean")

                images = self.demosaicImage(raw_image=raw_images, colour=colour, bit=bit)
                # cv2.imwrite("Test.png", images)

        except Exception as e:
            self.logger.exception(f"Failed post processing of raw images {e}")

        return images

    def setExposureMicrons(self, microns: int = 0) -> int:
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
        if self.streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self.streamingMode = False

        if microns != 0:
            try:
                qc, gc = self.getExposureInfo()
                self.logger.debug(f'Setting exposure time to {microns}us')

                # v4l interprets 1 as 100us -> devide microns by 100 to get the correct value
                v4l2_exposure = int(microns / 100)
                if v4l2_exposure == gc.value:
                    self.logger.debug(f'Exposure time was already set to {microns}us')
                    self.ExposureMicrons = microns

                elif v4l2_exposure != gc.value and qc.minimum <= v4l2_exposure <= qc.maximum:
                    # set control value
                    gc.value = v4l2_exposure
                    fcntl.ioctl(self.device, int(v4l2.VIDIOC_S_CTRL), gc)

                    # get control value
                    fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_CTRL), gc)

                    if v4l2_exposure == gc.value:
                        self.ExposureMicrons = microns
                    else:
                        raise Exception
                else:
                    raise Exception

            except Exception as e:
                self.logger.exception(f"Failed to set exposure time: {e}")
            return microns

        else:
            return self.ExposureMicrons

    def setFramerate(self, framerate: int = 10):
        if self.streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self.streamingMode = False

        try:
            parm = v4l2.v4l2_streamparm()
            parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            parm.parm.capture.capability = v4l2.V4L2_CAP_TIMEPERFRAME
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_PARM), parm)

            self.logger.debug(
                f'Set frame rate from {parm.parm.capture.timeperframe.denominator} fps to {framerate} fps')

            if parm.parm.capture.timeperframe.denominator != framerate:
                parm.parm.capture.timeperframe.denominator = framerate
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_S_PARM), parm)
                time.sleep(0.1)
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_PARM), parm)

                if not (parm.parm.capture.timeperframe.denominator == framerate):
                    raise Exception

        except Exception as e:
            self.logger.exception(f"Failed to set frame rate {framerate} fps: {e}")

    def prepare_live(self, exposure: int):
        if self.streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self.streamingMode = False
        self.setLive(True)
        self.setExposureMicrons(microns=exposure)
        self.logger.info(f"Live View Exposure time : {exposure}µs")

    def setLive(self, Mode: bool = False):
        self.liveStream = Mode

    def __del__(self) -> None:
        """
        Camera object Destructor
        :return:
        """
        self.logger.debug(f"Deleting Camera Object {self}")

        if not self.device:
            del self.device

    def analyseBlackLevel(self, raw_images: np.ndarray, channel: str = "", display: bool = False) -> Tuple[int, int]:
        if isinstance(raw_images, list):
            index = 0
            for image in raw_images:
                image_list = list()
                channel = channel.lower()
                if channel == "b":
                    rawImage = image[0::2, 0::2]
                elif channel == "g1":
                    rawImage = image[0::2, 1::2]
                elif channel == "g2":
                    rawImage = image[1::2, 0::2]
                elif channel == "r":
                    rawImage = image[1::2, 1::2]
                elif channel == "all":
                    rawImage = image
                else:
                    print("idiot")

                print("rawImage max:", np.max(rawImage), "min:", np.min(rawImage), "devi:",
                      np.std(rawImage), "mean:", np.mean(rawImage))
                image_list.append(rawImage)
                index += 1

            mean = np.mean(image_list, axis=0).astype(np.uint16)
            if display:
                hist = cv2.calcHist([mean], [0], None, [int(np.max(mean))], [0, int(np.max(mean) + 1)])
                plt.plot(hist)
                plt.ylim([0, 10])
                plt.title("Histogram")
                plt.show()

            thresh_values = "soft"
            if thresh_values == "hard":
                thresh = [62, 70]
            elif thresh_values == "soft":
                thresh = [58, 72]

            new_mean = np.where(mean <= thresh[0], 0, mean)
            new_mean = np.where(new_mean >= thresh[1], 0, new_mean)

            dead_pixels = (new_mean < 10).sum()
            self.logger.debug(f"Found {dead_pixels} dead pixels")

            if display:
                plt.imshow(new_mean.astype(np.uint16), cmap='gray', vmin=0, vmax=1023)
                plt.colorbar()
                plt.show()

                hist_new = cv2.calcHist([new_mean], [0], None, [int(np.max(new_mean))], [0, int(np.max(new_mean) + 1)])
                plt.plot(hist_new)
                plt.ylim([0, 10])
                plt.title("Clipped Histogram")
                plt.show()

                print("mean of mean: ", np.mean(new_mean))
                print("std of mean: ", np.std(new_mean))

            cwd = os.getcwd()
            path = os.path.join(cwd + '/images/black_level_mean.npy')
            np.save(path, new_mean)

        return new_mean, dead_pixels

    def listFeatures(self) -> None:
        """
        Lists camera features
        """
        try:
            self.logger.debug('Listing camera features')
            for feature in self.features:
                print("-------------------")
                print(f"Feature name: {feature}")
        except Exception as e:
            self.logger.exception(f'Failed to get feature names: {e}')

    def checkfirstPhase(self, images: np.ndarray) -> int:
        ref_image = images[0]
        for i in range(1, len(images)):
            print(np.abs(np.mean(ref_image) - np.mean(images[i])))
            if np.abs(np.mean(ref_image) - np.mean(images[i])) > 5:
                i += 1
                firstPhase = i
                break
            else:
                firstPhase = 2

        return firstPhase


if __name__ == '__main__':
    logger = logging.getLogger(__name__)
    available_devices = Camera.listDevices()
    logger.debug(f"Available Devices {available_devices}")
    # cam = Camera(available_devices[-1])
    cam = Camera(available_devices[1])

    ##########################################################
    # Code for live view
    # cam.setTriggerMode("Out")
    # cam.setFramerate(framerate=10)
    # cam.setExposureMicrons(15000)
    #
    # cam.listFeatures()
    # refImage = cam.getImage()
    # ref_image = cam.postProcessImages(refImage, colour=True, bit=8, blacklevelcorrection=True)
    # cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    # cv2.imshow('test', ref_image)
    # i = 0
    # while True:
    #     cam.logger.debug(f'Iteration: {i}')
    #     ImageList: list = []
    #     try:
    #         img = func_timeout(2, cam.getImage)
    #     except:
    #         logger.debug('get Image failed')
    #         img = cam.getImage()
    #     finally:
    #         Image = cam.postProcessImages(img, colour=True, bit=8, blacklevelcorrection=True)
    #         if np.array_equal(ref_image, Image):
    #             print("images are identical")
    #             break
    #         ref_image = Image
    #         Image = cv2.cvtColor(Image, cv2.COLOR_BGR2RGB)
    #         cv2.imshow('test', Image)
    #         key = cv2.waitKey(1)
    #         if key & 0xFF == ord('q'):
    #             cv2.destroyAllWindows()
    #             break
    #         i += 1
    # del cam
    #########################################################

    #########################################################
    # Code for the image acquisition of x Frames
    cam.setTriggerMode("Out")
    cam.setFramerate(framerate=10)
    expectedImages = 19
    cam.setExposureMicrons(10000)
    # cam.prepareRecording(expectedImages)
    # rawImages = cam.record()
    start = time.time()

    for i in range(0,1):
        cam.prepareRecording(expectedImages)
        Images = cam.record()
        print(i)
        # time.sleep(1)

    Images = cam.postProcessImages(Images, colour=True, bit=8, blacklevelcorrection=False)
    end = time.time()

    print(end-start)
    plt.imshow(Images[3])
    plt.show()
    del cam
    #########################################################

    # ##########################################################
    # # # Code to display images
    # for i in range(0, len(Images)):
    #     print(f"average {np.average(Images[i])}")
    #     if Images[i].shape[2] > 2:
    #         plt.imshow(Images[i].astype(np.uint8))
    #     else:
    #         plt.imshow(Images[i].astype(np.uint8), cmap="gray")
    #     plt.colorbar()
    #     plt.show()
    # del cam
    # ##########################################################

    # ##########################################################
    # # Code for the analysis of dead pixels
    # rawImages = list()
    # for i in range(0, 20):
    #     path = os.path.join('/home/middendorf/PycharmProjects/pyCameras/pyCameras/images/black_level_image_' + str(i) + '.npy')
    #     rawImage = np.load(path)
    #     rawImages.append(rawImage)
    #
    # mask, dead_pixels = cam.analyseBlackLevel(rawImages, channel="All", display=False)
    # del cam
    # ##########################################################

__author__ = "Philipp Middendorf"
__credits__ = "Kolja Hedrich, Pascal Kern"
__maintainer__ = "Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

# check this repository for implementation https://github.com/AlexJinlei/python-v4l2
import logging
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
from func_timeout import func_timeout
from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate
from typing import List, Tuple, Union, BinaryIO, Dict, Any, Optional
import platform


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

        cam_name: str = "CX3-UVC"
        v4l2path: str = "/sys/class/video4linux"

        try:
            for item in os.listdir(v4l2path):
                pth = os.path.join(v4l2path, item, "name")
                if os.path.exists(pth):
                    with open(pth, "r") as f:
                        device_name = f.read()
                        if cam_name in device_name:
                            device_input = str(item)
                            self.device_handles.append(device_input)
            return 1

        except Exception as e:
            self.logger.info(f"failed updating device handle {e}")
            raise

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

        self._actual_images: int = 0
        self.requested_images: int = 0
        self.ExposureMicrons: int = 0
        self.TriggerMode: str = ""
        self.buffers: List[object] = []
        self.buf: v4l2.v4l2_buffer = v4l2.v4l2_buffer()
        self.buf_type: int = 0
        self._streamingMode: bool = False
        self.openDevice()
        self.cameraImages: List[np.ndarray] = []
        self.measurementMode: bool = False
        self.registerFeatures()
        self.getCapability()
        self.ImageWidth: int = 0
        self.ImageHeight: int = 0
        self.usb3: bool = False
        self.bit_death = 10
        self.debug: bool = False
        self.pixelFormat: str = "RGB10"
        self.gray = False


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
        self.registerFeature('MeasurementMode', self.setMeasurementMode)
        self.registerFeature('PixelFormat', self.setPixelFormat)

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

    def setPixelFormat(self, fmt: str = "Mono10"):
        self.pixelFormat = fmt
        if self.pixelFormat.lower() == "rgb8":
            self.gray = False
            self.bit_death = 8
        elif self.pixelFormat.lower() == "mono8":
            self.gray = True
            self.bit_death = 8
        elif self.pixelFormat.lower() == "rgb10":
            self.gray = False
            self.bit_death = 10
        elif self.pixelFormat.lower() == "mono10":
            self.gray = True
            self.bit_death = 10
        else:
            raise ValueError(f"Format unknown {fmt}")


    def setMeasurementMode(self, mode: bool = False) -> bool:
        self.measurementMode = mode
        return mode

    def getCapability(self) -> v4l2.v4l2_capability:
        try:
            cp = v4l2.v4l2_capability()
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYCAP), cp)

        except Exception as e:
            self.logger.exception(f"Failed to get sensor capability: {e}")
            raise

        return cp

    def setResolution(self, resolution: Tuple[int, int] = (1928, 1088)) -> Tuple[int, int]:
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_FMT), fmt)  # get current settings
        self.ImageWidth = fmt.fmt.pix.width
        self.ImageHeight = fmt.fmt.pix.height
        return self.ImageWidth, self.ImageHeight

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

            self.logger.debug(f'Min exposure time {qc.minimum * 100}us, max {qc.maximum * 100}us, '
                              f'default {qc.default * 100}us, Steps {qc.step * 100}us, current {gc.value * 100}us')

        except Exception as e:
            self.logger.exception(f"Failed to get ExposureInfo: {e}")
            raise

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
            raise

    def getFormat(self) -> v4l2.v4l2_format:
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
            raise

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
                raise
        else:
            self.logger.info('No Device present.')

    def setTriggerMode(self, mode: str = "") -> str:
        """
        Set the trigger mode of the camera to either "in", "out", or "off", or
        read the current trigger setting by passing None
        """
        if not mode:
            self.logger.debug(f"Get Trigger mode request returned: {self.TriggerMode}")
            return self.TriggerMode

        elif mode.lower() == 'in':
            self.TriggerMode = mode.lower()
            self.logger.debug(f"Trigger mode not implemented: {mode}")
            return self.TriggerMode

        elif mode.lower() == 'out':
            self.logger.debug("Camera Out Trigger on")
            self.TriggerMode = mode.lower()
            return self.TriggerMode

        elif mode.lower() == 'off':
            self.TriggerMode = mode.lower()
            self.logger.debug(f"Trigger mode not implemented: {mode}")
            return self.TriggerMode

        else:
            self.logger.warning(f"Trigger mode is unknown: {mode}")
            raise

    def prepareRecording(self, requested_images: int = 1) -> None:
        self.requested_images = requested_images
        if self._streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self._streamingMode = False
        if self.measurementMode:
            # todo: check this setup
            if not self.usb3:
                self._actual_images = requested_images * 2 + 2
            else:
                self._actual_images = requested_images * 2 + 2
        else:
            self._actual_images = requested_images

        self.buffers.clear()
        self.logger.info(f"Prepare recording {requested_images} images")
        try:  # empty buffer, otherwise all frames are identical

            req = v4l2.v4l2_requestbuffers()
            req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            req.memory = v4l2.V4L2_MEMORY_MMAP
            if platform.processor().lower() == "aarch64" or self.usb3:
                if self.debug:
                    req.count = 2  # nr of buffer frames for raspi
                else:
                    req.count = 2  # nr of buffer frames for raspi
            else:
                req.count = 1  # nr of buffer frames for usbc PC
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
            self._streamingMode = True
            self.cameraImages.clear()
            t0 = time.time()
            max_t = 1
            ready_to_read: list = []

            while len(ready_to_read) == 0 and time.time() - t0 < max_t:
                ready_to_read, ready_to_write, in_error = select.select([self.device], [], [], max_t)

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

        except Exception as e:
            self.logger.exception(f"Failed to prepare recording: {e}")

    def record(self) -> List[np.ndarray]:
        self.logger.info("Recording images")
        try:
            raw_images = self.read_buffers()

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self._streamingMode = False

            if self.measurementMode:
                start_image = self.checkFirstPhase(images=raw_images)
                image_counter: int = 0

                for i in range(start_image, len(raw_images)):
                    if self.debug:
                        self.cameraImages.append(raw_images[i])
                    elif image_counter % 2 == 0 and not self.usb3:
                        self.cameraImages.append(raw_images[i])
                    elif image_counter % 2 == 0 and self.usb3:
                        self.cameraImages.append(raw_images[i+1])
                    image_counter += 1
            else:
                for i in range(0, len(raw_images)):
                    self.cameraImages.append(raw_images[i])

            if self.measurementMode:
                self.cameraImages = self.postProcessImages(raw_images=self.cameraImages, blacklevelcorrection=True,
                                                             correction_type="mean")

            if len(self.cameraImages) != self.requested_images:
                self.logger.warning(f"requested {self.requested_images} images but got {len(self.cameraImages)}")

            self.logger.debug(f"recorded {len(self.cameraImages)} images")

            return self.cameraImages

        except Exception as e:
            self.logger.exception(f"Failed record images: {e}")
            raise

    def read_buffers(self) -> List[np.ndarray]:
        byte_arrays: List[bytearray] = []
        start_time = time.time()
        for index in range(0, self._actual_images):  # capture x frames

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue
            mm = self.buffers[self.buf.index]

            image_bytestream = mm.read()
            mm.seek(0)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

            image_byte_array = bytearray(image_bytestream)
            byte_arrays.append(image_byte_array)
            index += 1

        end_time = time.time()
        recording_time = end_time - start_time
        self.logger.info(f"capturing took: {recording_time}")
        raw_images: List = []
        for byte_array in byte_arrays:
            image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=byte_array).astype(np.uint16)
            raw_images.append(np.right_shift(image_array, 6).astype(np.uint16))

        return raw_images

    def getImages(self, requested_images: int) -> List[np.ndarray]:
        try:
            self.prepareRecording(requested_images)
            images = self.record()
            self.logger.info(f"returned {len(images)} images")
        except Exception as e:
            self.logger.exception(f"Failed getImages: {e}")
            raise
        if len(images) != requested_images:
            self.logger.warning(f"requested {requested_images} images but got {len(images)}")
        return images

    def getImage(self) -> np.ndarray:
        """
        Get an image from the camera

        :return: image : np.ndarray
            Image of camera device
        """
        if self._streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self._streamingMode = False

        self.buffers.clear()
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
                # tell the driver that we want some buffers
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYBUF), self.buf)
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

        self._streamingMode = True

        t0 = time.time()
        max_t = 1
        ready_to_read: list = []

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
        self._streamingMode = False

        byte_array = bytearray(image_bytestream)

        image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=byte_array).astype(np.uint16)
        image_array = np.right_shift(image_array, 6).astype(np.uint16)

        return image_array


    def blacklevelcorrection(self, uncorrected_image: np.ndarray, correction_type: str = "mean") -> np.ndarray:
        try:
            load_img = False
            if load_img:
                from pyCameras import __path__ as pycamera_path
                path = os.path.join(pycamera_path[0], "images/black_level_mean.npy")
                if os.path.exists(path):
                    black_level_image = np.load(path)
                else:
                    print(f"path {path} not found")
            else:
                black_level_image = np.ones_like(uncorrected_image) * 60
            max_pix_val = 2**self.bit_death - 1
            saturation_image = np.ones_like(uncorrected_image) * max_pix_val
            if correction_type.lower() == "pixelwise":
                corrected_image = np.subtract(uncorrected_image, black_level_image).astype(np.int16)
            elif correction_type.lower() == "mean":
                black_level_image = np.mean(black_level_image)*1.5
                diff = np.subtract(uncorrected_image, black_level_image).astype(np.int16)
                corrected_image = np.where(diff < 0, 0, uncorrected_image)
                corrected_image = np.where(corrected_image > max_pix_val, 0, uncorrected_image)
            else:
                print(f"no black level correction applied")
                return uncorrected_image

            blacklevel_factor = max_pix_val / (np.subtract(saturation_image, black_level_image))
            corrected_image = np.multiply(corrected_image.copy(), blacklevel_factor).astype(np.int16)
            return np.clip(corrected_image, 0, max_pix_val).astype(np.int16)

        except Exception as e:
            print(f"failed back level correction  {e}")
            raise

    def demosaicImage(self, raw_image: np.ndarray, algorithm: str = "interpolation",
                      rgb_scale: Optional[Dict[str, float]] = None) -> np.ndarray:
        if rgb_scale is not None:
            r_scale = rgb_scale['r']
            b_scale = rgb_scale['b']

        else:
            r_scale = 1.6
            b_scale = 1.5

        raw_image[0::2, 0::2] = np.multiply(raw_image[0::2, 0::2], r_scale)
        raw_image[1::2, 1::2] = np.multiply(raw_image[1::2, 1::2], b_scale)

        input_bit = 10
        max_pix_val = 2 ** input_bit - 1

        raw_image = np.clip(raw_image, 0, max_pix_val).astype(np.uint16)

        # Demosaicing using variable Number of Gradients
        if algorithm.lower() == "gradients":
            demosaic_img = cv2.demosaicing(src=raw_image, code=cv2.COLOR_BayerBG2BGR_VNG)

        # Demosaicing using Edge-Aware Demosaicing
        elif algorithm.lower() == "edge":
            demosaic_img = cv2.demosaicing(src=raw_image, code=cv2.COLOR_BayerBG2BGR_EA)

        elif algorithm.lower() == "ddfapd":
            demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(raw_image, "BGGR")

        else:
            # Demosaicing using bilinear interpolation
            if self.gray:
                demosaic_img = cv2.demosaicing(src=raw_image, code=cv2.COLOR_BayerBG2GRAY)
            else:
                demosaic_img = cv2.demosaicing(src=raw_image, code=cv2.COLOR_BayerBG2RGB)

        demosaic_img = np.clip(demosaic_img, 0, max_pix_val)
        norm_image = demosaic_img.copy() / max_pix_val

        max_output_pix_val = 2 ** self.bit_death - 1

        if self.bit_death == 8:
            image = norm_image.copy() * max_output_pix_val
            return image.astype(np.uint8)

        elif self.bit_death == 10 or 12 or 14:
            image = norm_image.copy() * max_output_pix_val
            return image.astype(np.uint16)

        else:
            return norm_image

    def postProcessImages(self, raw_images: Union[List[np.ndarray], Dict[Any, Any], np.ndarray],
                          blacklevelcorrection: bool = False, algorithm: str = 'interpolation',
                          correction_type: Optional[str] = 'mean') -> Union[List[np.ndarray], Dict[Any, Any], np.ndarray]:
        try:
            # i = 0
            img_list: list = []
            if isinstance(raw_images, list):
                for raw_image in raw_images:
                    # i += 1
                    if blacklevelcorrection:
                        raw_image = self.blacklevelcorrection(uncorrected_image=raw_image,
                                                                correction_type=correction_type)
                    image = self.demosaicImage(raw_image=raw_image, algorithm=algorithm)
                    # cv2.imwrite(str(i)+".png", image)
                    img_list.append(image)
                return img_list

            elif isinstance(raw_images, dict):
                img_dict: dict = dict()
                for k, v in raw_images.items():
                    for raw_image in v:
                        if blacklevelcorrection:
                            raw_image = self.blacklevelcorrection(uncorrected_image=raw_image,
                                                                    correction_type=correction_type)
                        image = self.demosaicImage(raw_image=raw_image, algorithm=algorithm)
                        img_list.append(image)
                    # cv2.imwrite(str(i)+".png", image)
                    img_dict[k] = img_list
                return img_dict

            else:
                if blacklevelcorrection:
                    raw_image = self.blacklevelcorrection(uncorrected_image=raw_images,
                                                            correction_type=correction_type)
                else:
                    raw_image = raw_images
                return self.demosaicImage(raw_image=raw_image, algorithm=algorithm)

        except Exception as e:
            print(f"function call prost processing failed {e}")
            raise

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
        if self._streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self._streamingMode = False

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
        if self._streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self._streamingMode = False

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
        if self._streamingMode:
            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            self._streamingMode = False
        self.setExposureMicrons(microns=exposure)
        self.logger.info(f"Live View Exposure time : {exposure}µs")

    def __del__(self) -> None:
        """
        Camera object Destructor
        :return:
        """
        self.logger.debug(f"Deleting Camera Object {self}")

        if not self.device:
            del self.device

    def analyseBlackLevel(raw_images: List[np.ndarray], channel: str = "", display: bool = False) -> Tuple:
        channel_list: list = []
        for raw_image in raw_images:
            channel = channel.lower()
            if channel == "b":
                channel_image = raw_image[0::2, 0::2]
            elif channel == "g1":
                channel_image = raw_image[0::2, 1::2]
            elif channel == "g2":
                channel_image = raw_image[1::2, 0::2]
            elif channel == "r":
                channel_image = raw_image[1::2, 1::2]
            else:
                channel_image = raw_image

            print("rawImage max:", np.max(channel_image), "min:", np.min(channel_image), "devi:",
                  np.std(channel_image), "mean:", np.mean(channel_image))
            channel_list.append(channel_image)

        mean = np.mean(channel_list, axis=0, dtype=np.uint16)
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
        else:
            thresh = [80, 80]

        new_mean = np.where(mean <= thresh[0], 0, mean)
        new_mean = np.where(new_mean >= thresh[1], 0, new_mean)

        dead_pixels: int = (new_mean < 10).sum()
        print(f"Found {dead_pixels} dead pixels")

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

    def checkFirstPhase(self, images: List[np.ndarray]) -> int:
        ref_image = images[0]
        first_phase: int = 2
        debug = False

        # todo: check if threshold can be modified depentend on the mean intensity of the image
        for i in range(1, len(images)):
            if debug:
                print(np.abs(np.mean(ref_image) - np.mean(images[i])))
            if np.abs(np.mean(ref_image) - np.mean(images[i])) > 3:
                i += 1
                first_phase = i
                if not debug:
                    break

        return first_phase

    @staticmethod
    def getCudaSupport() -> bool:
        try:
            count = cv2.cuda.getCudaEnabledDeviceCount()
            if count > 0:
                cuda_support = True
            else:
                cuda_support = False
        except Exception as e:
            print(f"Function call getCudeEnabledDeviceCount failed {e}")
            raise
        return cuda_support


if __name__ == '__main__':
    logger = logging.getLogger(__name__)
    # cuda_support = Camera.getCudaSupport()
    # print(cuda_support)
    available_devices = Camera.listDevices()
    logger.debug(f"Available Devices {available_devices}")
    cam = Camera(available_devices[1])

    cam.setTriggerMode("Out")
    cam.setFramerate(framerate=10)
    cam.setExposureMicrons(70000)
    cam.setPixelFormat(fmt = "Mono8")

    cam.listFeatures()

    ##########################################################
    # Code for live view
    # refImage = cam.getImage()
    # ref_image = cam.postProcessImages(refImage, blacklevelcorrection=True)
    # cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    # cv2.imshow('test', ref_image)
    # i = 0
    # while True:
    #     cam.logger.debug(f'Iteration: {i}')
    #     try:
    #         img = None
    #         img = func_timeout(timeout=1, func=cam.getImage)
    #     except Exception as e:
    #         logger.debug(f'get Image failed {e}')
    #
    #     finally:
    #         if img is None:
    #             img = cam.getImage()
    #         Image = cam.postProcessImages(img, blacklevelcorrection=True)
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
    # expectedImages = 10
    # start = time.time()
    #
    # while True:
    #     cam.prepareRecording(expectedImages)
    #     Images = cam.record()
    #     Images = cam.postProcessImages(Images, algorithm='interpolation')
    #     print(len(Images))
    #     end = time.time()
    #
    #     cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    #     for image in Images:
    #         # Image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #         cv2.imshow('test', image)
    #         key = cv2.waitKey(1)
    #         if key & 0xFF == ord('q'):
    #             cv2.destroyAllWindows()
    #             raise StopIteration
    #
    # del cam
    #########################################################

    # ##########################################################
    # # Code for the analysis of dead pixels
    # rawImages = list()
    # for i in range(0, 20):
    #     path_start = '/home/middendorf/PycharmProjects/pyCameras/pyCameras/images/black_level_image_'
    #     path = os.path.join(path_start + str(i) + '.npy')
    #     rawImage = np.load(path)
    #     rawImages.append(rawImage)
    #
    # mask, dead_pixels = cam.analyseBlackLevel(rawImages, channel="All", display=False)
    # del cam
    # ##########################################################

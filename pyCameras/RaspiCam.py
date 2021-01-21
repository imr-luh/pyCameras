__author__ = "Philipp Middendorf"
__credits__ = "Kolja Hedrich, Pascal Kern"
__maintainer__ = "Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

# check this repository for implementation https://github.com/AlexJinlei/python-v4l2
import logging
from abc import ABC
import numpy as np
import threading
import picamera

# import fcntl
import errno
# import mmap
# import select
import cv2
import matplotlib.pyplot as plt
import os
import time

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

        try:
            device_input = str(picamera.PiCamera.__name__)
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
        Capture device representing a RaspiCam (OV5640 or IMX219)
    """

    def __init__(self) -> None:
        self.logger = logging.getLogger(__name__)
        logging.basicConfig()
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        # self.device_handle = device_handle
        #
        # if self.device_handle is None:
        #     raise ConnectionError

        self._actual_images: int = 0
        self.requested_images: int = 0
        self.ExposureMicrons: int = 0
        self.FrameRate: int = 10
        self.TriggerMode: str = "out"
        self.PixelFormat: str = "Mono10"
        self._streamingMode: bool = False
        self.openDevice()
        self.cameraImages: List[np.ndarray] = []
        self.measurementMode: bool = False
        self.registerFeatures()
        self.getCapability()
        self.ImageWidth: int = 2592
        self.ImageHeight: int = 1944
        self.awb_mode = 'auto'
        self.videoPort: bool = False
        self.iso = 0
        self.contrast = 0
        self.saturation = 0
        self.vFlip, self.hFlip = False, False

        self.bit_death = 10
        self.debug: bool = False
        self.gray = False
        self.analysis_mode = False




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
        self.registerFeature('FrameRate', self.setFrameRate)
        self.registerFeature('Capability', self.getCapability)
        self.registerFeature('MeasurementMode', self.setMeasurementMode)

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


    def setPixelFormat(self, fmt = None ) -> str:
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
            self.logger.debug(f'Setting <PixelFormat> to {fmt}')
            if fmt.lower() == "rgb8":
                self.gray = False
                self.bit_death = 8
            elif fmt.lower() == "mono8":
                self.gray = True
                self.bit_death = 8
            elif fmt.lower() == "rgb10":
                self.gray = False
                self.bit_death = 10
            elif fmt.lower() == "mono10":
                self.gray = True
                self.bit_death = 10
            else:
                raise ValueError(f"Format unknown {fmt}")

            self.PixelFormat = fmt
        return self.PixelFormat

    def setMeasurementMode(self, mode: bool = False) -> bool:
        if mode is not None:
            self.measurementMode = mode
            self.analysis_mode = not mode
        return self.measurementMode

    def getCapability(self):
        try:
            print("not implemented")

        except Exception as e:
            self.logger.exception(f"Failed to get sensor capability: {e}")
            raise


    def setResolution(self, resolution: Tuple[int, int] = (2592, 1944)) -> Tuple[int, int]:
        try:
            try:
                self.device.resolution = (resolution[0], resolution[1])
            except Exception as e:
                self.logger(f'failed to set resolution: {resolution} with: {e}. Setting to default 2592 x 1944')
                resoultion = (2592, 1944)
                self.device.resolution = resoultion
        except Exception as e:
            self.logger(f'failed to set resolution 2592 x 1944 with {e}')

        self.ImageWidth = resolution[0]
        self.ImageHeight = resolution[1]
        return self.ImageWidth, self.ImageHeight
    #
    # def getExposureInfo(self) -> Tuple[v4l2.v4l2_queryctrl, v4l2.v4l2_control]:
    #     try:
    #         # query control
    #         qc = v4l2.v4l2_queryctrl()
    #         qc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
    #         fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYCTRL), qc)
    #
    #         # get control value
    #         gc = v4l2.v4l2_control()
    #         gc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
    #         fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_CTRL), gc)
    #         self.ExposureMicrons = gc.value * 100  # v4l2 compensation
    #
    #         self.logger.debug(f'Min exposure time {qc.minimum * 100}us, max {qc.maximum * 100}us, '
    #                           f'default {qc.default * 100}us, Steps {qc.step * 100}us, current {gc.value * 100}us')
    #
    #     except Exception as e:
    #         self.logger.exception(f"Failed to get ExposureInfo: {e}")
    #         raise
    #
    #     return qc, gc

    def openDevice(self) -> None:
        """
        Open the device for capturing images.

        :return:
        """
        try:
            self.device = picamera.PiCamera()
            self.logger.debug(f'Opened camera device: {self.device.name}')
        except Exception as e:
            self.logger.exception(f"Failed to open the camera device: {e}")
            raise

    def closeDevice(self) -> None:
        """
        Close the connection to the device
        """
        if self.device is not None:
            try:
                self.logger.debug('Closing camera device: {self.device}')
                self.device.close()
                del self.device

            except Exception as e:
                self.logger.exception(f"Failed to close the camera device: {e}")
                raise
        else:
            self.logger.info('No Device present.')

    def setTriggerMode(self, mode = None) -> str:
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
        try:
            self.camera.use_video_port = True
            data = StringIO()

        except Exception as e:
            self.logger.exception(f"Failed to prepare recording: {e}")

    def record(self) -> List[np.ndarray]:
        try:
            try:
                imgs = None
                time_out = 3 * self._actual_images / self.FrameRate
                imgs = func_timeout(time_out, self._record)
            except:
                self.logger.debug("recording failed")
            finally:
                if imgs is None:
                    self.prepareRecording(requested_images=self.requested_images)
                    imgs = func_timeout(time_out, self._record)
                return imgs

        except:
            self.logger.debug("recording failed twice")
        finally:
            if imgs is None:
                self.prepareRecording(requested_images=self.requested_images)
                imgs = self._record()
            if self.measurementMode:
                imgs = self.postProcessImages(raw_images=imgs, blacklevelcorrection=False, correction_type="mean")
            return imgs

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

    def setImageFormat(self, fmt = None) -> str:
        if fmt is None:
            return self.imageFormat

        if fmt in ('yuv', 'jpg', 'bmp', 'png'):
            self.imageFormat = fmt
            return self.imageFormat
        else:
            raise ValueError('wrong format passed')

    def getImage(self) -> np.ndarray:
        try:
            data = StringIO()
            self.camera.capture(data, self.imageFormat, use_video_port=self.video_port, bayer=True)
            data.seek(0)
            data.truncate(0)
            rawdata = data.getvalue()
        except Exception as e:
            self.logger(f"get Image failed with : {e}")
        if self.measurementMode:
            return self.postProcessImages(rawdata)
        return rawdata

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
                # lower_offset = np.mean(black_level_image) * 2.5
                lower_offset = 150
                lower_diff = np.subtract(uncorrected_image, lower_offset).astype(np.int16)
                corrected_image = np.where(lower_diff < 0, 0, uncorrected_image)
                # upper_offset = np.mean(black_level_image) * 2
                upper_offset = 900
                # upper_diff = np.subtract(saturation_image, upper_offset).astype(np.int16)
                corrected_image = np.where(corrected_image > upper_diff, 0, corrected_image)
            else:
                print(f"no black level correction applied")
                return uncorrected_image

            # blacklevel_factor = max_pix_val / (np.subtract(saturation_image, lower_offset))
            # corrected_image = np.multiply(corrected_image.copy(), blacklevel_factor).astype(np.int16)
            return np.clip(corrected_image, 0, max_pix_val).astype(np.uint16)

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
            img_list: list = []
            if isinstance(raw_images, list):
                for raw_image in raw_images:

                    if blacklevelcorrection:
                        raw_image = self.blacklevelcorrection(uncorrected_image=raw_image,
                                                                correction_type=correction_type)
                    image = self.demosaicImage(raw_image=raw_image, algorithm=algorithm)
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
        if microns != 0:
            try:
                self.device.exposure = microns
                self.ExposureMicrons = microns
            except Exception as e:
                self.logger.exception(f"Failed to set exposure time: {e}")

            return self.ExposureMicrons

        else:
            return self.ExposureMicrons

    def setFrameRate(self, frameRate = None):
        if frameRate is None:
            return self.FrameRate

        try:
            self.FrameRate = frameRate

        except Exception as e:
            self.logger.exception(f"Failed to set frame rate {framerate} fps: {e}")

        return self.FrameRate

    def prepare_live(self, exposure: int):
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

    # def analyseBlackLevel(raw_images: List[np.ndarray], channel: str = "", display: bool = False) -> Tuple:
    #     channel_list: list = []
    #     for raw_image in raw_images:
    #         channel = channel.lower()
    #         if channel == "b":
    #             channel_image = raw_image[0::2, 0::2]
    #         elif channel == "g1":
    #             channel_image = raw_image[0::2, 1::2]
    #         elif channel == "g2":
    #             channel_image = raw_image[1::2, 0::2]
    #         elif channel == "r":
    #             channel_image = raw_image[1::2, 1::2]
    #         else:
    #             channel_image = raw_image
    #
    #         print("rawImage max:", np.max(channel_image), "min:", np.min(channel_image), "devi:",
    #               np.std(channel_image), "mean:", np.mean(channel_image))
    #         channel_list.append(channel_image)
    #
    #     mean = np.mean(channel_list, axis=0, dtype=np.uint16)
    #     if display:
    #         hist = cv2.calcHist([mean], [0], None, [int(np.max(mean))], [0, int(np.max(mean) + 1)])
    #         plt.plot(hist)
    #         plt.ylim([0, 10])
    #         plt.title("Histogram")
    #         plt.show()
    #
    #     thresh_values = "soft"
    #     if thresh_values == "hard":
    #         thresh = [62, 70]
    #     elif thresh_values == "soft":
    #         thresh = [58, 72]
    #     else:
    #         thresh = [80, 80]
    #
    #     new_mean = np.where(mean <= thresh[0], 0, mean)
    #     new_mean = np.where(new_mean >= thresh[1], 0, new_mean)
    #
    #     dead_pixels: int = (new_mean < 10).sum()
    #     print(f"Found {dead_pixels} dead pixels")
    #
    #     if display:
    #         plt.imshow(new_mean.astype(np.uint16), cmap='gray', vmin=0, vmax=1023)
    #         plt.colorbar()
    #         plt.show()
    #
    #         hist_new = cv2.calcHist([new_mean], [0], None, [int(np.max(new_mean))], [0, int(np.max(new_mean) + 1)])
    #         plt.plot(hist_new)
    #         plt.ylim([0, 10])
    #         plt.title("Clipped Histogram")
    #         plt.show()
    #
    #         print("mean of mean: ", np.mean(new_mean))
    #         print("std of mean: ", np.std(new_mean))
    #
    #     cwd = os.getcwd()
    #     path = os.path.join(cwd + '/images/black_level_mean.npy')
    #     np.save(path, new_mean)
    #
    #     return new_mean, dead_pixels

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

    def setAnalysisMode(self, mode: bool = False) -> bool:
        if mode is not None:
            self.measurementMode = not mode
            self.analysis_mode = mode
        return self.analysis_mode


if __name__ == '__main__':
    logger = logging.getLogger(__name__)
    # available_devices = Camera.listDevices()
    # logger.debug(f"Available Devices {available_devices}")
    # cam = Camera(available_devices[1])
    cam = Camera()

    # cam.setTriggerMode("Out")
    # cam.setFrameRate(frameRate=10)
    cam.setExposureMicrons(70000)
    # cam.setPixelFormat(fmt = "RGB8")
    # cam.setMeasurementMode(mode=True)

    cam.listFeatures()

    ##########################################################
    # Code for live view
    refImage = cam.getImage()
    # ref_image = cam.postProcessImages(refImage, blacklevelcorrection=False)
    cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    cv2.imshow('test', ref_image)
    i = 0
    while True:
        cam.logger.debug(f'Iteration: {i}')
        img = cam.getImage()
        # Image = cam.postProcessImages(img, blacklevelcorrection=False)
        if np.array_equal(ref_image, Image):
            print("images are identical")
            break
        refImage = Image
        cv2.imshow('test', Image)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
        i += 1
    del cam
    #########################################################

    #########################################################
    # Code for the image acquisition of x Frames
    # expectedImages = 17
    # start = time.time()
    #
    # while True:
    #     cam.prepareRecording(expectedImages)
    #     Images = cam.record()
    #     print(len(Images))
    #     end = time.time()
    #
    #     # cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    #     # for image in Images:
    #     #     # Image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     #     cv2.imshow('test', image)
    #     #     key = cv2.waitKey(1)
    #     #     if key & 0xFF == ord('q'):
    #     #         cv2.destroyAllWindows()
    #     #         raise StopIteration
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

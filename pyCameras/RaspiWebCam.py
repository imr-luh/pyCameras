__author__ = "Philipp Middendorf"
__credits__ = "Kolja Hedrich, Pascal Kern"
__maintainer__ = "Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

# check this repository for implementation https://github.com/AlexJinlei/python-v4l2
import logging
from abc import ABC
import numpy as np
import picamera
from picamera.array import PiRGBArray, PiBayerArray, PiYUVArray
import io
import os
import cv2
import time
from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate
from typing import List, Tuple, Union, Dict, Any, Optional
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from pyCameras.captureThread import CaptureThread
import urllib3
import urllib
from urllib.request import urlopen

LOGGING_LEVEL = logging.DEBUG

camurl = ['http://130.75.27.197:5900/']

class Controller(ControllerTemplate):
    """ Camera Controller for RaspberryPi cameras."""

    def __init__(self) -> None:
        super(Controller, self).__init__()
        logging.basicConfig()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.logger.info("Starting Raspi Camera Controller")
        self.device_handles: List[str] = []

    def updateDeviceHandles(self) -> int:
        """
        Updates the list of available device handles

        """
        self.logger.info("searching camera devices")

        try:
            if self.testConnection(url=camurl):
                self.device_handles.append(camurl)
            else:
                self.logger.debug(f"No camera found at ip {camurl}")
            return 1

        except Exception as e:
            self.logger.info(f"failed updating device handle {e}")
            raise

    @staticmethod
    def testConnection(url=['http://130.75.27.197:5900/']):
        try:
            url = url + 'hellocamera'
            ret = urlopen(url).read()
            # req = urllib.Request(url)
            # ret = urllib.urlopen(req)
            # print "url:", ret.read()
            if not ret.getcode() == 200:
                raise IOError("Camera connection returned invalid response")
            return True
        except Exception as ex:
            print(f"cameraRaspiHTTP.testConnection failed: {ex}")
            return False

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
        return "<Raspberry Camera Controller>"


class Camera(CameraTemplate, ABC):
    """
        Capture device representing a RaspiCam (OV5640 or IMX219)
    """

    def __init__(self, device_handle) -> None:
        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        logging.basicConfig()
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.device_handle = device_handle

        if self.device_handle is None:
            raise ConnectionError

        self.url = camurl
        self.ImageWidth = 2592/2  # rpi cam hardware default
        # self.ImageWidth = 2592  # rpi cam hardware default
        self.ImageHeight = 1944/2
        # self.ImageHeight = 1944
        self.awb_mode = 'auto'
        #        self.exposure_mode = 'auto'    #not yet fully implemented
        self.saturation = 0
        self.vFlip, self.hFlip = False, False
        self.imgFormat = 'gray'
        self.lastImage = None  # buffer last image (why not?)
        self.binning = 0
        self.delay = 0
        self._actual_images: int = 0
        self.requested_images: int = 0
        self.ExposureMicrons: int = 0
        # self.FrameRate: int = 10
        self.TriggerMode: str = "out"
        self.PixelFormat: str = "Mono8"
        self._streamingMode: bool = False
        self.openDevice()
        # self.rawCap = PiYUVArray(self.device)
        # self.use_video_port = True
        self.cameraImages: List[np.ndarray] = []
        self.measurementMode: bool = False
        self.registerFeatures()
        self.bit_death = 10
        self.debug: bool = False
        self.gray = True
        self.white_balance_mode = 'off'
        self.iso = 100
        self.exposure_mode = 'off'
        self.brightness = 80
        self.contrast = 100

        # https: // picamera.readthedocs.io / en / release - 1.13 / recipes2.html  # using-a-flash-with-the-camera

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
        # self.registerFeature('FrameRate', self.setFrameRate)
        # self.registerFeature('MeasurementMode', self.setMeasurementMode)
        # self.registerFeature('ImageFormat', self.setImageFormat)

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

    def setPixelFormat(self, fmt=None) -> str:
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

    def setResolution(self, resolution=None) -> Tuple[int, int]:
        if resolution is not None:
            if resolution == [2592, 1944]:
                self.binning = 0

            elif resolution == [2592/2, 1944/2]:
                self.binning = 2

            else:
                raise ValueError(f"unknown exposure passed! {resolution}")
            self.ImageWidth, self.ImageHeight = resolution
        return self.ImageWidth, self.ImageHeight

    def openDevice(self) -> None:
        """
        Open the device for capturing images.

        :return:
        """
        try:
            self.logger.debug(f'Nothing happend to camera device at: {self.url}')
        except Exception as e:
            self.logger.exception(f"Failed to open the camera device: {e}")
            raise

    def closeDevice(self) -> None:
        """
        Close the connection to the device
        """
        try:
            self.logger.debug(f'Closing camera device: {self.url}')

        except Exception as e:
            self.logger.exception(f"Failed to close the camera device: {e}")
            raise
        else:
            self.logger.info('No Device present.')

    def setTriggerMode(self, mode=None) -> str:
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

    def setImageFormat(self, fmt=None) -> str:
        if fmt is None:
            return self.ImageFormat
        elif fmt.lower() == "bgr" or fmt.lower() =="rgb":
            self.ImageFormat = "bgr"

        elif fmt.lower() == "yuv":
            self.ImageFormat = "yuv"

        elif fmt.lower() == "jpeg":
            self.ImageFormat = "jpeg"

        else:
            raise ValueError('wrong format passed')
        return self.ImageFormat

    def setFlip(self, hflip=None, vflip=None):
        ''' set horizontal and vertical flip

            if "None" is provided there is no change
        '''
        if hflip is not None:
            self.hFlip = hflip
        if vflip is not None:
            self.vFlip = vflip

    def prepareRecording(self, requested_images: int = 1) -> None:
        self.requested_images = requested_images
        self._actual_images = 2 * requested_images + 1

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

    def record(self) -> List[np.ndarray]:
        img_list = list()
        for img in self._actual_images:
            img_list.append(self.getImage())
        return img_list

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
        try:
            self.delay = 0
            self.delay = max(int(1.5 * self.exposure / 1000), 32)

            fullurl = self.url + \
                      '%s?bytes&exposure=%i&binning=%d&delay=%d' % \
                      (self.imgFormat, self.exposure, self.binning, self.delay)
            # print(f"rpicam: {fullurl}")
            ret = urlopen(fullurl, timeout=1e4).read()

            hd, wd = self.normalizeRes((self.ImageHeight, self.ImageWidth))
            pix = np.fromstring(ret, dtype=np.uint8).reshape((hd, wd))[0:self.ImageHeight, 0:self.ImageWidth]
            return np.array(pix, dtype=np.uint8)
        except Exception as e:
            self.logger.debug(f'get Image failed with : {e}')

            # if self.imgFormat in ['raw', 'gray']:
            #     #    w,h = (2592/2, 1944/2)
            #     w, h = (1296, 972)
            #     # w,h = (1312, 992)
            # else:
            #     w, h = self.imgWidth, self.imgWidth

            # if self.imgFormat == 'jpeg':
            #     cStr = StringIO(raw)
            #     pix = np.array(Image.open(cStr))
            # else:
            #     if self.imgFormat == 'gray':
            #
            #         hd, wd = self.normalizeRes((h, w))
            #         pix = np.fromstring(raw, dtype=np.uint8).reshape((hd, wd))[0:h, 0:w]
            #     else:
            #         hd, wd = self.normalizeRes(h, w)
            #         pix = np.fromstring(raw, dtype=np.uint8).reshape((hd, wd, 3))[0:h, 0:w, :]

            # print np.max(np.max(pix, axis = 0), axis = 0)
            # result = pix
            # if (gray or self.saturation <= -100) and len(
            #         pix.shape) == 3:  # abolute no saturatio practically equals to grayscale
            #     result = np.zeros((pix.shape[0], pix.shape[1]), dtype=np.int16)
            #     num = 0
            #     if 'r' in self.rawChannels:
            #         num += 1
            #         result += pix[:, :, 0]
            #     if 'g' in self.rawChannels:
            #         num += 1
            #         result += pix[:, :, 1]
            #     if 'b' in self.rawChannels:
            #         num += 1
            #         result += pix[:, :, 2]
            #     result = np.array(result / num, dtype=np.uint8)
            #
            # self.lastImage = result
            # return result

    def normalizeRes(self, res):
        w, h = res
        dw = np.ceil(w/32.)*32
        dh = np.ceil(h/32.)*32
        return dw, dh

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
            demosaic_img = cv2.demosaicing(src = raw_image, code = cv2.COLOR_BayerBG2BGR_VNG)

        # Demosaicing using Edge-Aware Demosaicing
        elif algorithm.lower() == "edge":
            demosaic_img = cv2.demosaicing(src = raw_image, code = cv2.COLOR_BayerBG2BGR_EA)

        else:
            # Demosaicing using bilinear interpolation
            if self.gray:
                demosaic_img = cv2.demosaicing(src = raw_image, code = cv2.COLOR_BayerBG2GRAY)
            else:
                demosaic_img = cv2.demosaicing(src = raw_image, code = cv2.COLOR_BayerBG2RGB)

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

    def YUVtoIMG(self, img, opencv=False):
        if opencv:
            if self.gray:
                return cv2.cvtColor(src = img, code = cv2.COLOR_YUV2GRAY_420)
            else:
                return cv2.cvtColor(src = img, code = cv2.COLOR_YUV2RGB)
            # return cv2.cvtColor(src = img, dstCn = cv2.COLOR_YUV2RGB_I420)
        else:

            fwidth = (img.shape[0] + 31) // 32 * 32
            fheight = (img.shape[1] + 15) // 16 * 16
            stream = self.rawCap
            # Load the Y (luminance) data from the stream
            Y = np.fromfile(stream, dtype = np.uint8, count = fwidth * fheight). \
                reshape((fheight, fwidth))
            # Load the UV (chrominance) data from the stream, and double its size
            U = np.fromfile(stream, dtype = np.uint8, count = (fwidth // 2) * (fheight // 2)). \
                reshape((fheight // 2, fwidth // 2)). \
                repeat(2, axis = 0).repeat(2, axis = 1)
            V = np.fromfile(stream, dtype = np.uint8, count = (fwidth // 2) * (fheight // 2)). \
                reshape((fheight // 2, fwidth // 2)). \
                repeat(2, axis = 0).repeat(2, axis = 1)
            # Stack the YUV channels together, crop the actual resolution, convert to
            # floating point for later calculations, and apply the standard biases
            YUV = np.dstack((Y, U, V))[:img.shape[0], :img.shape[1], :].astype(np.float)
            YUV[:, :, 0] = YUV[:, :, 0] - 16  # Offset Y by 16
            YUV[:, :, 1:] = YUV[:, :, 1:] - 128  # Offset UV by 128
            # YUV conversion matrix from ITU-R BT.601 version (SDTV)
            #              Y       U       V
            M = np.array([[1.164, 0.000, 1.596],  # R
                          [1.164, -0.392, -0.813],  # G
                          [1.164, 2.017, 0.000]])  # B
            # Take the dot product with the matrix to produce RGB output, clamp the
            # results to byte range and convert to bytes

    def postProcessImages(self, raw_images: Union[List[np.ndarray], Dict[Any, Any], np.ndarray]) -> \
            Union[List[np.ndarray], Dict[Any, Any], np.ndarray]:
        try:
            img_list: list = []
            if isinstance(raw_images, list):
                for raw_image in raw_images:
                    if self.ImageFormat == 'yuv':
                        img_list.append(self.YUVtoIMG(raw_image))
                    else:
                        img_list.append(raw_image)
                return img_list

            elif isinstance(raw_images, dict):
                img_dict: dict = dict()
                for k, v in raw_images.items():
                    for raw_image in v:
                        if self.ImageFormat == 'yuv':
                            img_list.append(self.YUVtoIMG(raw_image))
                        else:
                            img_list.append(raw_image)
                    img_dict[k] = img_list
                return img_dict

            else:
                if self.ImageFormat == 'yuv':
                    return self.YUVtoIMG(raw_images)
                else:
                    return raw_images

        except Exception as e:
            self.logger.debug(f"function call prost processing failed {e}")
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
        if microns is not None:
            try:
                self.ExposureMicrons = microns
            except Exception as e:
                self.logger.exception(f"Failed to set exposure time: {e}")

        return self.ExposureMicrons

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

    def prepare_live(self, exposure: int):
        self.setExposureMicrons(microns = exposure)
        self.logger.info(f"Live View Exposure time : {exposure}µs")

    def __del__(self) -> None:
        """
        Camera object Destructor

        """
        self.logger.debug(f"Deleting Camera Object {self}")

    def setAnalysisMode(self, mode: bool = False) -> bool:
        if mode is not None:
            self.measurementMode = not mode
            self.analysis_mode = mode
        return self.analysis_mode

    def saveImages(self, imgs: List[np.ndarray], path = None):
        for i, img in enumerate(imgs):
            img_name = f"img{i}.png"
            if path is not None:
                img_name = os.path.join(path, img_name)
            if not cv2.imwrite(img_name, img):
                raise ValueError


if __name__ == '__main__':
    logger = logging.getLogger(__name__)
    available_devices = Camera.listDevices()
    logger.debug(f"Available Devices {available_devices}")
    # cam = Camera(available_devices[-1])
    cam = Camera(available_devices)
    # cam = Camera()

    # cam.setTriggerMode("Out")
    # cam.setFrameRate(frameRate=30)
    cam.setExposureMicrons(60000)

    # cam.setResolution([640, 480])
    # cam.setResolution([1296, 730])
    cam.setResolution([1296, 972])
    # cam.setResolution([2592, 1944])
    # cam.setPixelFormat(fmt = "RGB8")

    cam.listFeatures()

    ##########################################################
    # # Code for live view
    # refImage = cam.getImage()
    # # ref_image = cam.postProcessImages(refImage, blacklevelcorrection=False)
    # # cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    # # cv2.imshow('test', refImage)
    # i = 0
    # while True:
    #     cam.logger.debug(f'Iteration: {i}')
    #     img = cam.getImage()
    #     # Image = cam.postProcessImages(img, blacklevelcorrection=False)
    #     if np.array_equal(refImage, img):
    #         print("images are identical")
    #         break
    #     refImage = img
    #     # cv2.imshow('test', img)
    #     key = cv2.waitKey(1)
    #     if key & 0xFF == ord('q'):
    #         cv2.destroyAllWindows()
    #         break
    #     i += 1
    # del cam
    #########################################################

    #########################################################
    # Code for the image acquisition of x Frames
    expectedImages = 20
    start = time.time()
    cam.prepareRecording(expectedImages)
    Images = cam.record()
    end = time.time()
    print(f"recording {len(Images)} took {end - start}")
    # cam.saveImages(Images, path = "/home/pi/Pictures/")

    # cv2.namedWindow('test', cv2.WINDOW_NORMAL)
    # for image in Images:
    #     # Image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     cv2.imshow('test', image)
    #     key = cv2.waitKey(1)
    #     if key & 0xFF == ord('q'):
    #         cv2.destroyAllWindows()
    #         raise StopIteration

    del cam
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

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

from picamera import mmal, mmalobj, exc
from picamera.mmalobj import to_rational

MMAL_PARAMETER_ANALOG_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x59
MMAL_PARAMETER_DIGITAL_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x5A
LOGGING_LEVEL = logging.DEBUG


class Controller(ControllerTemplate):
    """ Camera Controller for RaspberryPi cameras."""

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
            # cam = picamera.PiCamera()
            # device_input = str(cam.revision)
            # self.device_handles.append(device_input)
            # cam.close()
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

        self._actual_images: int = 0
        self.requested_images: int = 0
        self.ExposureMicrons: int = 0
        self.FrameRate: int = 10
        self.TriggerMode: str = "out"
        self.PixelFormat: str = "Mono10"
        self._streamingMode: bool = False
        self.openDevice()
        self.rawCap = PiYUVArray(self.device)
        c= 'yuv'
        self.use_video_port = True
        self.cameraImages: List[np.ndarray] = []
        self.measurementMode: bool = False
        self.registerFeatures()
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
        self.setImageFormat("bgr")
        # self.device.flash_mode = 'on'

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
        self.registerFeature('FrameRate', self.setFrameRate)
        self.registerFeature('MeasurementMode', self.setMeasurementMode)
        self.registerFeature('ImageFormat', self.setImageFormat)

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

    def setVideoPort(self, videoPort=None) -> bool:
        if videoPort is not None and type(videoPort) is bool:
            self.videoPort = videoPort
            return self.videoPort
        else:
            self.videoPort = self.device.CAMERA_VIDEO_PORT

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

    def set_gain(self, gain, value):
        """Set the analog gain of a PiCamera.

        camera: the picamera.PiCamera() instance you are configuring
        gain: either MMAL_PARAMETER_ANALOG_GAIN or MMAL_PARAMETER_DIGITAL_GAIN
        value: a numeric value that can be converted to a rational number.
        """

        if gain not in [MMAL_PARAMETER_ANALOG_GAIN, MMAL_PARAMETER_DIGITAL_GAIN]:
            raise ValueError("The gain parameter was not valid")
        ret = mmal.mmal_port_parameter_set_rational(self.device._camera.control._port, gain, to_rational(value))
        if ret == 4:
            raise exc.PiCameraMMALError(ret, "Are you running the latest version of the userland libraries? "
                                             "Gain setting was introduced in late 2017.")
        elif ret != 0:
            raise exc.PiCameraMMALError(ret)

    def set_analog_gain(self, value):
        """Set the gain of a PiCamera object to a given value."""
        self.set_gain(MMAL_PARAMETER_ANALOG_GAIN, value)

    def set_digital_gain(self, value):
        """Set the digital gain of a PiCamera object to a given value."""
        self.set_gain(MMAL_PARAMETER_DIGITAL_GAIN, value)

    def setMeasurementMode(self, mode: bool = False) -> bool:
        if mode is not None:
            self.measurementMode = mode
            self.analysis_mode = not mode
        return self.measurementMode

    def setResolution(self, resolution=None) -> Tuple[int, int]:
        if resolution is None:
            try:
                self.ImageWidth = self.device.resolution.width
                self.ImageHeight = self.device.resolution.height
            except Exception as e:
                self.logger.debug(f'failed to get resolution with: {e}.')
        else:
            try:
                try:
                    self.device.resolution = resolution
                    self.ImageWidth, self.ImageHeight = resolution
                except Exception as e:
                    self.logger.debug(
                        f'failed to set resolution: {resolution} with: {e}. Setting to default 2592 x 1944')
                    self.device.resolution = (2592, 1944)
            except Exception as e:
                self.logger.debug(f'failed to set resolution 2592 x 1944 with {e}')
        # self.ImageWidth, self.ImageHeight = resolution
        return self.ImageWidth, self.ImageHeight

    def openDevice(self) -> None:
        """
        Open the device for capturing images.

        :return:
        """
        try:
            self.device = picamera.PiCamera()
            self.logger.debug(f'Opened camera device: {self.device.revision}')
        except Exception as e:
            self.logger.exception(f"Failed to open the camera device: {e}")
            raise

    def closeDevice(self) -> None:
        """
        Close the connection to the device
        """
        if self.device is not None:
            try:
                self.logger.debug(f'Closing camera device: {self.device}')
                self.device.close()
                self.rawCap.close()
                del self.device
                del self.rawCap

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
            self.rawCap = PiRGBArray(self.device)

        elif fmt.lower() == "yuv":
            self.ImageFormat = "yuv"
            self.rawCap = PiYUVArray(self.device)

        elif fmt.lower() == "jpeg":
            self.ImageFormat = "jpeg"
            self.rawCap = PiBayerArray(self.device)

        else:
            raise ValueError('wrong format passed')
        return self.ImageFormat

    def prepareRecording(self, requested_images: int = 1) -> None:
        self.requested_images = requested_images
        self._actual_images = 2 * requested_images + 1

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

    def record(self) -> List[np.ndarray]:
        capT = CaptureThread(cam = self.device, rawCap=self.rawCap, continous_capture = True,
                             fmt = self.ImageFormat, frame_goal = self._actual_images).start()
        time.sleep(2.0)
        img_list = list()
        fps = FPS().start()
        # frame_counter = 0

        while fps._numFrames < self._actual_images:
        # while frame_counter < self._actual_images:
        # while True:
            # grab the frame from the threaded video stream and resize it
            # to have a maximum width of 400 pixels
            img_list.append(capT.read())
            # update the FPS counter
            # frame_counter += 1
            fps.update()

        # stop the timer and display FPS information
        fps.stop()
        capT.stop()
        # print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        # print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        return img_list

    # def record(self) -> List[np.ndarray]:
        # try:
        #     img_list = list()
        #     i = 0
        #     start = time.time()
        #     for i in range(self._actual_images):
        #         self.device.capture(self.rawCap, self.ImageFormat, use_video_port=self.use_video_port)
        #         # img_list.append(self.rawCap.rgb_array)
        #         img_list.append(self.rawCap.array)
        #         self.rawCap.seek(0)
        #         # self.rawCap.truncate(0)
        #         self.rawCap.truncate()
        #         i += 1
        #         if i == self._actual_images:
        #             break
        #
        #     end = time.time()
        #     print(f"capture of {self.ImageFormat}: {end - start}")
        #
            # i = 0
        # img_list = list()
        # stream = io.BytesIO()
        # start = time.time()
        # # self.ImageFormat = "jpeg"
        # for i in range(self._actual_images):
        #     self.device.capture(stream, self.ImageFormat, use_video_port=self.use_video_port)
        #     img_list.append(np.frombuffer(stream.getvalue(), dtype=np.uint8))
        #     stream.truncate(0)
        #     i += 1
        #     if i == self._actual_images:
        #         break
        #
        # end = time.time()
        # print(f"byte stuff {self.ImageFormat}: {end - start}")
        #
        # i = 0
        # img_list = list()
        # start = time.time()
        # for frame in self.device.capture_continuous(self.rawCap, format=self.ImageFormat,
        #                                             use_video_port=self.use_video_port):
        #     img_list.append(frame.rgb_array)
        #     # clear the stream in preparation for the next frame
        #     self.rawCap.truncate(0)
        #     i += 1
        #     if i == self._actual_images:
        #         break
        # end = time.time()
        # print(f"capture continuous of {self.ImageFormat}: {end - start}")
        # except Exception as e:
        #     self.logger.debug(f'record failed with : {e}')
        # if self.measurementMode:
        #     return self.postProcessImages(img_list)
        # return img_list

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
            self.setVideoPort(videoPort = False)
            self.device.capture(self.rawCap, self.ImageFormat, use_video_port = self.videoPort)
            self.rawCap.truncate(0)
            img = self.rawCap.array

        except Exception as e:
            self.logger.debug(f'get Image failed with : {e}')
        if self.measurementMode:
            return self.postProcessImages(img)
        return img

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
                self.device.awb_mode = "off"
                self.device.awb_gains = (195 / 128, 127 / 64)
                self.device.image_denoise = False
                self.device.iso = 0
                self.set_analog_gain(8)
                self.set_digital_gain(1)
                self.device.exposure_mode = 'off'
                self.device.brightness = 80
                self.device.contrast = 100
                self.device.shutter_speed = microns
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

    def setFrameRate(self, frameRate=None):
        if frameRate is None:
            self.FrameRate = self.device.framerate
        else:
            try:
                self.device.framerate = frameRate
                self.FrameRate = frameRate
            except Exception as e:
                self.logger.exception(f"Failed to set frame rate {frameRate} fps: {e}")
        # self.FrameRate = frameRate
        return self.FrameRate

    def prepare_live(self, exposure: int):
        self.setExposureMicrons(microns = exposure)
        self.logger.info(f"Live View Exposure time : {exposure}µs")

    def __del__(self) -> None:
        """
        Camera object Destructor

        """
        self.logger.debug(f"Deleting Camera Object {self}")

        if not self.device:
            del self.device

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
    print(f"framerate: {cam.setFrameRate(10)}")
    # print(cam.setFrameRate(30))

    cam.setMeasurementMode(mode=True)

    # cam.listFeatures()

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

__author__ = "Philipp Middendorf"
__credits__ = "Kolja Hedrich, Pascal Kern"
__maintainer__ = "Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

import logging
import numpy as np
import v4l2
import fcntl
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

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = logging.DEBUG

# todo edit v4l2.py list(range(1,9)) + [0x80] and list(range(0, 4)) + [2]


class Controller(ControllerTemplate):
    """ Camera Controller for the OV2740 camera."""

    def __init__(self):
        super(Controller, self).__init__()
        logging.basicConfig()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.logger.info("Starting OV2740 Camera Controller")
        # Todo: what does this do?
        # self.context = rs.context()
        self.devices = []
        self.device_handles = []


    def updateDeviceHandles(self):
        """
        Updates the list of available device handles

        """
        self.logger.info("searching camera devices")
        index = 0

        cam_name = "CX3-UVC"
        v4l2path = "/sys/class/video4linux"
        # for base, subs, filenames in os.walk(v4l2path, followlinks=True):
        # for filenames in os.walk(v4l2path, followlinks=True):
        #     for filename in filenames:
        #         if filename == "name":
        #             pth = os.path.join(base, filename)
        #             with open(pth, "r") as f:
        #                 device_name = f.read()
        #
        #                 if cam_name in device_name:
        #                     device_input = os.path.split(base)[1]
        #                     self.device_handles.append(device_input)
        # return 1

        for item in os.listdir(v4l2path):
            pth = os.path.join(v4l2path, item, "name")
            if os.path.exists(pth):
                with open(pth, "r") as f:
                    device_name = f.read()
                    if cam_name in device_name:
                        device_input = str(item)
                        self.device_handles.append(device_input)

        return 1

    def getDevice(self, device_handle):
        """
          Open the actual capture device to grab images

          Parameters
          ----------
          device_handle
              One entry of the device list returned by self.listDevices()


          :return: Camera(device_handle) : Camera object
            The capture device of type cameraRealsense
          """
        # self.logger.debug('Opening device {device_handle}'
        #                   ''.format(device_handle=device_handle))
        self.logger.debug('Opening device')
        try:
            return Camera(device_handle)
        except Exception as e:
            self.logger.exception('Failed to open the camera device: {e}'
                                  ''.format(e=e))
            msg = '<Was not able to open camera with given device handle!!'
            e.message = msg
            raise

    def closeController(self):
        """
            Delete all detected devices
        """
        for dev in self.device_handles:
            del dev
        self.logger.info("OV2740 Camera Controller shutdown")

    def __del__(self):
        """
        Camera Controller Destructor

        """
        self.logger.debug(f"Deleting Camera Controller {self}")
        self.closeController()

    def __repr__(self):
        """

        :return: str
        """
        return "<OV2740 Camera Controller>"


class Camera(CameraTemplate):
    """
        Capture device representing a OV2740
    """

    def __init__(self, device_handle):
        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        logging.basicConfig()
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.device_handle = device_handle

        self.devices = self.device_handle
        self.stream_started = False

        if len(self.devices) == 0:
            raise ConnectionError

        self.img_data = []
        self._expected_images = 0

        self.openDevice()
        self.registerFeatures()




    @staticmethod
    def listDevices():
        """
        List available realsense camera devices


        Controller :return: self.device_handles : list
        List of camera device handles that can be used to create a new camera instance
        """
        return Controller().listDevices()

    def registerFeatures(self):
        """
        Registration of shared features that should be the same for all camera
        implementations. E.g. ExposureMicrons, Resolution, Gain, Format and
        TriggerMode
        """
        self.logger.debug('Registering camera features')

        self.registerFeature('ExposureMicrons', self.setExposureMicrons)
        # self.registerFeature('ExposureTime', self.setExposureMicrons)
        # self.registerFeature('Exposure', self.setExposureMicrons)

        self.registerFeature('Resolution', self.setResolution)

        # self.registerFeature('Gain', self.setGain)
        self.registerFeature('Format', self._getFormat)

        self.registerFeature('TriggerMode', self.setTriggerMode)
        # self.registerFeature('Trigger', self.setTriggerMode)

        self.registerFeature('Framerate', self.setFramerate)

    def openDevice(self):
        """
        Open the device for capturing images.

        :return:
        """
        try:
            self.device = open(str('/dev/' + self.devices[-1]), 'rb+', buffering=0)

            print(">> get device capabilities")
            self.cp = v4l2.v4l2_capability()
            fcntl.ioctl(self.device, v4l2.VIDIOC_QUERYCAP, self.cp)

            print("Driver:", "".join((chr(c) for c in self.cp.driver)))
            print("Name:", "".join((chr(c) for c in self.cp.card)))
            print("Is a video capture device?", bool(self.cp.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE))
            print("Supports read() call?", bool(self.cp.capabilities & v4l2.V4L2_CAP_READWRITE))
            print("Supports streaming?", bool(self.cp.capabilities & v4l2.V4L2_CAP_STREAMING))

            print(">>> streamparam")  ## somewhere in here you can set the camera framerate
            self.parm = v4l2.v4l2_streamparm()
            self.parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            self.parm.parm.capture.capability = v4l2.V4L2_CAP_TIMEPERFRAME
            fcntl.ioctl(self.device, v4l2.VIDIOC_G_PARM, self.parm)
            fcntl.ioctl(self.device, v4l2.VIDIOC_S_PARM, self.parm)  # just got with the defaults

            self.logger.debug('Opened camera device: {device}'
                                  ''.format(device=self.device.name))
        except Exception as e:
            self.logger.exception('Failed to open the camera device: {e}'
                                  ''.format(e=e))

            # stream_type = self.stream_profile.stream_type()
            # format = self.stream_profile.format()
            # fps = self.stream_profile.fps()

    def _getFormat(self):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        fmt : str
            String describing the desired image format (e.g. "Mono8" or
            "Mono10"), or None to read the current image format

        Returns
        -------
        fmt : str
            The image format after applying the passed value
        """
        try:
            print(">> device setup")
            fmt = v4l2.v4l2_format()
            fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            fcntl.ioctl(self.device, v4l2.VIDIOC_G_FMT, self.fmt)  # get current settings
            self.device.ImageWidth = fmt.fmt.pix.width
            self.device.ImageHeight = fmt.fmt.pix.height
            self.device.PixelFormat = fmt.fmt.pix.pixelformat
            self.device.BytesPerLine = fmt.fmt.pix.bytesperline
            self.device.SizeImage = fmt.fmt.pix.sizeimage

            # fcntl.ioctl(self.device, v4l2.VIDIOC_S_FMT, self.fmt)

        except Exception as e:
            self.logger.exception(e)
        return self.device.PixelFormat


    def closeDevice(self):
        """
        Close the connection to the device
        """
        if self.device is not None:
            try:
                self.logger.debug('Closing camera device: {device}'
                                  ''.format(device=self.device))
                # self.device.Close()
                del self.device
                self.device = None

            except Exception as e:
                self.logger.exception('Failed to close the camera device: '
                                      '{e}'.format(e=e))
        else:
            self.logger.info('No Device present.')


    def setTriggerMode(self, mode=None):
        """
        Set the trigger mode of the camera to either "in", "out", or "off", or
        read the current trigger setting by passing None

        """

        if mode is None:
            return self.TriggerMode

        elif mode == r'In':
                self.TriggerMode = mode
                self.logger.warning(f"Trigger mode not implemented: {mode}")

        elif mode == r'Out':
            self.logger.info("Camera Out Trigger on")
            self.TriggerMode = mode

        elif mode == r'Off':
            self.TriggerMode = mode
            self.logger.warning(f"Trigger mode not implemented: {mode}")

        return self.TriggerMode

    def prepareRecording(self, num):
        print(">> init mmap capture")
        req = v4l2.v4l2_requestbuffers()
        req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = v4l2.V4L2_MEMORY_MMAP
        req.count = 1  # nr of buffer frames
        fcntl.ioctl(self.device, v4l2.VIDIOC_REQBUFS, req)  # tell the driver that we want some buffers
        print("req.count", req.count)

        self.buffers = []
        print(">>> VIDIOC_QUERYBUF, mmap, VIDIOC_QBUF")
        for ind in range(req.count):
            # setup a buffer
            self.buf = v4l2.v4l2_buffer()
            self.buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            self.buf.memory = v4l2.V4L2_MEMORY_MMAP
            self.buf.index = ind
            fcntl.ioctl(self.device, v4l2.VIDIOC_QUERYBUF, self.buf)

            mm = mmap.mmap(self.device.fileno(), self.buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                           offset=self.buf.m.offset)
            self.buffers.append(mm)

            # queue the buffer for capture
            fcntl.ioctl(self.device, v4l2.VIDIOC_QBUF, self.buf)

        print(">> Start streaming")
        self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.device, v4l2.VIDIOC_STREAMON, self.buf_type)

        print(">> Capture image")
        t0 = time.time()
        max_t = 1
        ready_to_read, ready_to_write, in_error = ([], [], [])
        print(">>> select")
        while len(ready_to_read) == 0 and time.time() - t0 < max_t:
            ready_to_read, ready_to_write, in_error = select.select([self.device], [], [], max_t)

        self._expected_images = num
        self.logger.info(f"Prepare recording {num} images")



    def record(self):
        fcntl.ioctl(self.device, v4l2.VIDIOC_DQBUF, self.buf)  # get image from the driver queue
        fcntl.ioctl(self.device, v4l2.VIDIOC_QBUF, self.buf)  # request new image
        self.logger.info(f"Recording {self._expected_images}, ignore empty image")
        start = time.time()
        images = list()
        index = 0
        for i in range(self._expected_images):  # capture 50 frames
            print(f"Frame : {index}")

            fcntl.ioctl(self.device, v4l2.VIDIOC_DQBUF, self.buf)  # get image from the driver queue
            # print("buf.index", buf.index)

            mm = self.buffers[self.buf.index]

            image_bytestream = mm.read()
            mm.seek(0)
            fcntl.ioctl(self.device, v4l2.VIDIOC_QBUF, self.buf)  # request new image

            image_bytearray = bytearray(image_bytestream)
            images.append(image_bytearray)
            index += 1

        fcntl.ioctl(self.device, v4l2.VIDIOC_STREAMOFF, self.buf_type)
        end = time.time()
        print("capturing took: ", end-start)
        # self.closeDevice()

        return images

    def getImage(self):
        """
        Get an image from the camera

        :return: image : np.ndarray
            Image of camera device
        """
        fcntl.ioctl(self.device, v4l2.VIDIOC_DQBUF, self.buf)  # get image from the driver queue
        fcntl.ioctl(self.device, v4l2.VIDIOC_QBUF, self.buf)  # request new image
        self.logger.info(f"get single image, ignore empty image")

        fcntl.ioctl(self.device, v4l2.VIDIOC_DQBUF, self.buf)  # get image from the driver queue
        # print("buf.index", buf.index)

        mm = self.buffers[self.buf.index]

        image_bytestream = mm.read()
        mm.seek(0)
        fcntl.ioctl(self.device, v4l2.VIDIOC_QBUF, self.buf)  # request new image

        image_bytearray = bytearray(image_bytestream)

        image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=image_bytearray).astype(np.uint16)
        image = np.right_shift(image_array, 6).astype(np.uint16)

        return image

    def postProcessImage(self, images):
        if isinstance(images, list):
            rgbImages= list()
            for image in images:
                image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=image).astype(np.uint16)
                rawImage = np.right_shift(image_array, 6).astype(np.uint16)
                rawImage[0::2, 0::2] = np.multiply(rawImage[0::2, 0::2], 1.8)
                rawImage[1::2, 1::2] = np.multiply(rawImage[1::2, 1::2], 1.7)
                demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(rawImage, "BGGR")

                demosaic_norm = demosaic_img.copy() / np.max(demosaic_img)

                image = demosaic_img.copy().astype(np.uint16)
                rgbImages.append(image)
        else:
            image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=images).astype(np.uint16)
            rawImage = np.right_shift(image_array, 6).astype(np.uint16)
            rawImage[0::2, 0::2] = np.multiply(rawImage[0::2, 0::2], 1.8)
            rawImage[1::2, 1::2] = np.multiply(rawImage[1::2, 1::2], 1.7)
            demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(rawImage, "BGGR")

            demosaic_norm = demosaic_img.copy() / np.max(demosaic_img)

            rgbImages = demosaic_img.copy().astype(np.uint16)


        return rgbImages

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
            self.logger.debug(f'Setting exposure time to {microns}us')
            # query control
            qc = v4l2.v4l2_queryctrl()
            qc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
            fcntl.ioctl(self.device, v4l2.VIDIOC_QUERYCTRL, qc)
            print("exposure defalut: ", qc.default)

            # get control value
            gc = v4l2.v4l2_control()
            gc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
            fcntl.ioctl(self.device, v4l2.VIDIOC_G_CTRL, gc)
            print("exposure before: ", gc.value)

            # set control value
            gc.value = microns
            fcntl.ioctl(self.device, v4l2.VIDIOC_S_CTRL, gc)
            print("exposure set to: ", gc.value)

            # get control value
            fcntl.ioctl(self.device, v4l2.VIDIOC_G_CTRL, gc)
            print("exposure is", gc.value)

            self.ExposureMicrons = microns

        return microns

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
        if self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False

        # TODO check if better way to get resolution and fps from current camera stream
        self.stream_profile = self.device.get_stream_profiles()
        self.stream_profile = self.stream_profile[12]

        # TODO hardcoded resolution parameters check if getting resolution from stream is possible
        return 1920, 1080

    def setFramerate(self, framerate=6):
        print(">>> streamparam")  ## somewhere in here you can set the camera framerate
        parm = v4l2.v4l2_streamparm()
        parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        parm.parm.capture.capability = v4l2.V4L2_CAP_TIMEPERFRAME
        fcntl.ioctl(self.device, v4l2.VIDIOC_G_PARM, parm)
        print("fps bevor: ", parm.parm.capture.timeperframe.denominator)
        if (parm.parm.capture.timeperframe.denominator != framerate):
            parm.parm.capture.timeperframe.denominator = framerate
            if framerate == 30:
                parm.parm.capture.timeperframe.numerator = 1
            elif framerate == 15:
                parm.parm.capture.timeperframe.numerator = 2
            elif framerate == 10:
                parm.parm.capture.timeperframe.numerator = 3
            elif framerate == 6:
                parm.parm.capture.timeperframe.numerator = 4

            print("fps to set: ", parm.parm.capture.timeperframe.denominator)
            fcntl.ioctl(self.device, v4l2.VIDIOC_S_PARM, parm)  # just got with the defaults


            # fcntl.ioctl(self.device, v4l2.VIDIOC_ENUM_FRAMEINTERVALS, parm)  # just got with the defaults



            fcntl.ioctl(self.device, v4l2.VIDIOC_G_PARM, parm)
            print("fps is: ", parm.parm.capture.timeperframe.denominator)
            if not (parm.parm.capture.timeperframe.denominator == framerate):
                return False
        return True

    def prepare_live(self, exposure):
        """Define live view exposure time to be less. So no overlighting occurs"""
        live_exposure = exposure * 0.05
        self.setExposureMicrons(microns=live_exposure)
        self.logger.info(f"Live View Exposure time : {live_exposure}µs")


    def __del__(self):
        """
        Camera object Destructor
        :return:
        """
        self.logger.debug(f"Deleting Camera Object {self}")

        if self.device is not None:
            del self.device

        if self.stream_started:
            self.stream_started = False
            # self.pipeline.stop()


if __name__ == '__main__':
    logger = logging.getLogger(__name__)
    available_devices = Camera.listDevices()
    logger.debug(f"Available Devices {available_devices}")

    cam = Camera(available_devices)

    cam.setTriggerMode("Out")
    cam.setFramerate(30)
    cam.setExposureMicrons(500)

    cv2.namedWindow('OV2740 RAW Image', cv2.WINDOW_AUTOSIZE)
    index = 0
    cam.prepareRecording(10)
    images = cam.record()
    rgbImages = cam.postProcessImage(images)

    # while True:
    #     color_img = cam.getImage()
    #     index += 1
    #
    #     if color_img is not None:
    #         if color_img.shape[0] > 640:
    #             cv2.imshow('OV2740 RAW Image', cv2.resize(color_img, (640, 480)))
    #     key = cv2.waitKey(1)
    #     if key & 0xFF == ord('q'):
    #         cv2.destroyAllWindows()
    #         break

    del cam

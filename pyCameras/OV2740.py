__author__ = "Philipp Middendorf"
__credits__ = "Kolja Hedrich, Pascal Kern"
__maintainer__ = "Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

import logging
from abc import ABC

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
from skimage import color

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
        self.device_handles = []

    def updateDeviceHandles(self):
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

    def __del__(self):
        """
        Camera Controller Destructor

        """
        self.logger.debug(f"Deleting Camera Controller {self}")
        self.device_handles = None

    def __repr__(self):
        """

        :return: str
        """
        return "<OV2740 Camera Controller>"


class Camera(CameraTemplate, ABC):
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

        if self.device_handle is None:
            raise ConnectionError

        self.img_data = []
        self._expected_images = 0
        self.ExposureMicrons = 0
        self.TriggerMode = None
        self.buffers = []
        self.buf = None
        self.buf_type = None

        self.openDevice()
        self.registerSharedFeatures()
        self.getCapability()

    @staticmethod
    def listDevices():
        """
        List available CX3 camera devices

        Controller :return: self.device_handles : list
        List of camera device handles that can be used to create a new camera instance
        """
        return Controller().listDevices()

    def registerSharedFeatures(self):
        """
        Registration of shared features that should be the same for all camera
        implementations. E.g. ExposureMicrons, Resolution, Gain, Format and
        TriggerMode
        """
        self.logger.debug('Registering camera features')

        self.registerFeature('ExposureMicrons', self.setExposureMicrons)
        self.registerFeature('ExposureTime', self.setExposureMicrons)
        self.registerFeature('Exposure', self.setExposureMicrons)

        self.registerFeature('ExposureInfo', self.getExposureInfo)


        self.registerFeature('Resolution', self.getResolution)

        # self.registerFeature('Gain', self.setGain)
        self.registerFeature('Format', self.getFormat)

        self.registerFeature('TriggerMode', self.setTriggerMode)
        self.registerFeature('Trigger', self.setTriggerMode)

        self.registerFeature('Framerate', self.setFramerate)

        self.registerFeature('Capability', self.getCapability)

    def getCapability(self):
        try:
            cp = v4l2.v4l2_capability()
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QUERYCAP), cp)

            # print("Driver:", "".join((chr(c) for c in cp.driver)))
            # print("Name:", "".join((chr(c) for c in cp.card)))
            # print("Is a video capture device?", bool(cp.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE))
            # self.logger.debug(f'Sensor supports read() call: {bool(cp.capabilities & v4l2.V4L2_CAP_READWRITE)}')
            # self.logger.debug(f'Sensor supports streaming: {bool(cp.capabilities & v4l2.V4L2_CAP_STREAMING)}')
            return cp

        except Exception as e:
            self.logger.exception(f"Failed to get sensor capability: {e}")

    def getResolution(self):
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_FMT), fmt)  # get current settings
        self.device.ImageWidth = fmt.fmt.pix.width
        self.device.ImageHeight = fmt.fmt.pix.height
        return self.device.ImageWidth, self.device.ImageHeight

    def getExposureInfo(self):
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
                f'Min exposure time {qc.minimum *100}us, max {qc.maximum *100}us, default {qc.default*100}us, Steps {qc.step*100}us, current {gc.value*100}us')
            return qc, gc

        except Exception as e:
            self.logger.exception(f"Failed to get ExposureInfo: {e}")

    def openDevice(self):
        """
        Open the device for capturing images.

        :return:
        """
        try:
            self.device = open(str('/dev/' + str(self.device_handle)), 'rb+', buffering=0)
            self.logger.debug('Opened camera device: {device}'
                              ''.format(device=self.device.name))
        except Exception as e:
            self.logger.exception(f"Failed to open the camera device: {e}")

            # stream_type = self.stream_profile.stream_type()
            # format = self.stream_profile.format()
            # fps = self.stream_profile.fps()

    def getFormat(self):
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
            fmt = v4l2.v4l2_format()
            fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_FMT), fmt)  # get current settings
            # self.device.PixelFormat = fmt.fmt.pix.pixelformat
            # self.device.BytesPerLine = fmt.fmt.pix.bytesperline
            # self.device.SizeImage = fmt.fmt.pix.sizeimage

            # fcntl.ioctl(self.device, v4l2.VIDIOC_S_FMT, self.fmt)
            return fmt

        except Exception as e:
            self.logger.exception(e)

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
                self.logger.exception(f"Failed to close the camera device: {e}")
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
        try:
            self.logger.debug(f"init mmap capture, creating buffer etc.")
            req = v4l2.v4l2_requestbuffers()
            req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            req.memory = v4l2.V4L2_MEMORY_MMAP
            req.count = 1  # nr of buffer frames
            # req.count = self._expected_images  # nr of buffer frames
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_REQBUFS), req)  # tell the driver that we want some buffers
            # print("req.count", req.count)


            # print(">>> VIDIOC_QUERYBUF, mmap, VIDIOC_QBUF")
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

            self.logger.info("Sensor starts streaming")

            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMON), self.buf_type)

            t0 = time.time()
            max_t = 1
            ready_to_read, ready_to_write, in_error = ([], [], [])

            while len(ready_to_read) == 0 and time.time() - t0 < max_t:
                ready_to_read, ready_to_write, in_error = select.select([self.device], [], [], max_t)

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

            self._expected_images = num
            self.logger.info(f"Prepare recording {num} images")

        except Exception as e:
            self.logger.exception(f"Failed to prepare recording: {e}")

    def record(self):
        try:
            self.logger.info(f"Recording {self._expected_images}, ignore empty image")
            start = time.time()
            images = list()
            for index in range(0, self._expected_images):  # capture 50 frames
                print(f"Frame : {index}")

                fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue
                mm = self.buffers[self.buf.index]

                image_bytestream = mm.read()
                mm.seek(0)
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

                image_byte_array = bytearray(image_bytestream)
                images.append(image_byte_array)
                index += 1

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMOFF), self.buf_type)
            end = time.time()
            self.logger.info(f"capturing took: {end - start}")
            return images

        except Exception as e:
            self.logger.exception(f"Failed to record images: {e}")

    def getImage(self):
        """
        Get an image from the camera

        :return: image : np.ndarray
            Image of camera device
        """
        try:
            images = list()
            self.logger.debug(f"init mmap capture, creating buffer etc.")
            req = v4l2.v4l2_requestbuffers()
            req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            req.memory = v4l2.V4L2_MEMORY_MMAP
            req.count = 1  # nr of buffer frames
            # req.count = self._expected_images  # nr of buffer frames
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_REQBUFS), req)  # tell the driver that we want some buffers
            # print("req.count", req.count)

            # print(">>> VIDIOC_QUERYBUF, mmap, VIDIOC_QBUF")
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

            self.logger.info("Sensor starts streaming")

            self.buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_STREAMON), self.buf_type)

            t0 = time.time()
            max_t = 1
            ready_to_read, ready_to_write, in_error = ([], [], [])

            while len(ready_to_read) == 0 and time.time() - t0 < max_t:
                ready_to_read, ready_to_write, in_error = select.select([self.device], [], [], max_t)

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

            # fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue
            # fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image
            self.logger.info(f"get single image, ignore empty image")

            fcntl.ioctl(self.device, int(v4l2.VIDIOC_DQBUF), self.buf)  # get image from the driver queue
            # print("buf.index", buf.index)

            mm = self.buffers[self.buf.index]

            image_bytestream = mm.read()
            mm.seek(0)
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_QBUF), self.buf)  # request new image

            image_byte_array = bytearray(image_bytestream)

            image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=image_byte_array).astype(np.uint16)
            image = np.right_shift(image_array, 6).astype(np.uint16)
            images.append(image)
            return images

        except Exception as e:
            self.logger.exception(f"Failed to get an image from Sensor: {e}")

    def postProcessImage(self, raw_images, colour=False, bit=8):
        images = list()
        if isinstance(raw_images, list):
            for image in raw_images:
                image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=image).astype(np.uint16)
                rawImage = np.right_shift(image_array, 6).astype(np.uint16)
                path = '/home/middendorf/PycharmProjects/pyCameras/pyCameras/images/black_level_mean.npy'
                # path = 'pyCameras/pyCameras/images/black_level_mean.npy'
                if os.path.exists(path):
                    print("True")
                    blacklevel_image = np.load(path)
                    lin_rawImage = np.where(rawImage - blacklevel_image <= 0, 1023 - blacklevel_image, rawImage - blacklevel_image) / \
                                   (np.ones([1088, 1928])*1023 - blacklevel_image) * 1023
                    # rawImage[0::2, 0::2] = np.multiply(rawImage[0::2, 0::2], 2.07)
                    # rawImage[1::2, 1::2] = np.multiply(rawImage[1::2, 1::2], 1.675)
                    lin_rawImage[0::2, 0::2] = np.multiply(lin_rawImage[0::2, 0::2], 1.9)
                    lin_rawImage[1::2, 1::2] = np.multiply(lin_rawImage[1::2, 1::2], 1.6)
                    demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(lin_rawImage, "BGGR")
                else:
                    print("False")
                    rawImage[0::2, 0::2] = np.multiply(rawImage[0::2, 0::2], 1.9)
                    rawImage[1::2, 1::2] = np.multiply(rawImage[1::2, 1::2], 1.6)
                    demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(rawImage, "BGGR")

                # demosaic_norm_max_value = demosaic_img.copy() / np.max(demosaic_img)
                # norm_image = cv2.normalize(demosaic_img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                norm_image = demosaic_img.copy() / 1023
                norm_image = norm_image.copy().astype(np.float32)
                # norm_image = image / 1023
                if not colour:
                    norm_image = color.rgb2gray(norm_image)

                if bit == 8:
                    image = norm_image * 255
                    image = image.copy().astype(np.uint8)
                if bit == 10:
                    image = norm_image * 1023
                    image = image.copy().astype(np.uint16)

                images.append(image)
        return images


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
                        return microns
                else:
                    raise Exception

            except Exception as e:
                self.logger.exception(f"Failed to set exposure time: {e}")

    def setFramerate(self, framerate=6):

        try:
            parm = v4l2.v4l2_streamparm()
            parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            parm.parm.capture.capability = v4l2.V4L2_CAP_TIMEPERFRAME
            fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_PARM), parm)

            self.logger.debug(f'Set frame rate from {parm.parm.capture.timeperframe.denominator} fps to {framerate} fps')

            if parm.parm.capture.timeperframe.denominator != framerate:
                parm.parm.capture.timeperframe.denominator = framerate
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_S_PARM), parm)
                time.sleep(0.2)
                fcntl.ioctl(self.device, int(v4l2.VIDIOC_G_PARM), parm)

                if not (parm.parm.capture.timeperframe.denominator == framerate):
                    raise Exception

        except Exception as e:
            self.logger.exception(f"Failed to set frame rate {framerate} fps: {e}")

    def prepare_live(self, exposure):
        # Todo: edit this
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


    def analyseBlackLevel(self, raw_images, channel=None):
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
            hist = cv2.calcHist([mean], [0], None, [int(np.max(mean))], [0, int(np.max(mean))])
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

            hist_new = cv2.calcHist([new_mean], [0], None, [int(np.max(new_mean))], [0, int(np.max(new_mean))])
            plt.plot(hist_new)
            plt.ylim([0, 10])
            plt.title("Clipped Histogram")
            plt.show()

            print("mean of mean: ", np.mean(new_mean))
            print("std of mean: ", np.std(new_mean))
            np.save('/home/middendorf/PycharmProjects/pyCameras/pyCameras/images/black_level_mean', new_mean)

        return new_mean


if __name__ == '__main__':
    logger = logging.getLogger(__name__)
    available_devices = Camera.listDevices()
    logger.debug(f"Available Devices {available_devices}")

    cam = Camera(available_devices[-1])
    # #
    # # cam.setTriggerMode("Out")
    # cam.setFramerate(framerate=6)
    # expectedImages = 20
    #
    #
    # # i = 15000
    # #
    # # name = str(i)
    # cam.setExposureMicrons(100000)
    # cam.prepareRecording(expectedImages)
    # rawImages = cam.record()
    # processedImages = list()
    # Images = cam.postProcessImage(rawImages, colour=True, bit=8)
    # del cam
    # plt.imshow(Images[3])
    # plt.show()

    # #
    # #
    # for i in range(0, len(Images)):
    #     print(f"average {np.average(Images[i])}")
    #     # plt.imshow(Images[i].astype(np.uint8), cmap="gray")
    #     plt.imshow(Images[i].astype(np.uint8))
    #     plt.colorbar()
    #     plt.show()
    rawImages = list()
    for i in range(0, 20):
        path = os.path.join('/home/middendorf/PycharmProjects/pyCameras/pyCameras/images/black_level_image_' + str(i) + '.npy')
        rawImage = np.load(path)
        rawImages.append(rawImage)

    cam.analyseBlackLevel(rawImages, channel="All")


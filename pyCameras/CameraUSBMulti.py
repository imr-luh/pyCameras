#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Moritz von Wrangel"
__copyright__ = "Copyright 2018, LUH: IMR"
__credits__ = ["Philip Middendorf"]
# __license__ = ""
__version__ = "0.1"
__maintainer__ = "Moritz von Wrangel"
__email__ = "moritz.wrangel@imr.uni-hannover.de"
__status__ = "alpha"
__package_name__ = "RScamera"
__developer__ = __author__

'''
Based on AVT implementation of Rüdiger Beermann and pymba:
https://github.com/morefigs/pymba.git
'''

import copy
import re
import time
import logging

import numpy as np
import pyrealsense2 as rs
import cv2

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = None


class Controller(ControllerTemplate):
    """
    Camera controller for AVT cameras based on pymba
    """

    def __init__(self):
        """
        Camera controller for AVT camera devices. This implementation uses
        pymba as backend.Out
        """
        super(Controller, self).__init__()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.logger.debug('Starting RS Camera Controller')
        self._config = rs.config
        time.sleep(0.2)


    # def __repr__(self):
    #     return "<AVT Camera Controller>"


class Camera(CameraTemplate):
    """
    Intel Realsense D415 Camera implementation

    Creating this Object automatically opens the camera. It is NOT necessary to
    call openDevice() !!! This is done to set some settings to put the camera
    into freerun mode.

    Good parameters for sync:
    Framerate 15
    exposure 1000
    """

    def __init__(self, device_handle):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self._expected_images = 0
        self.logger = logging.getLogger('Debug')
        self.stream_enabled = False
        self.resolution = [1920,1080]
        self.Width = 1920
        self.Height = 1080
        self.framerate = 6
        self.pipeline_started = False
        self.Exposure = 1000

        self.framelist=[]
        self.imgData = []


        self.init_sensors()


        LOGGING_LEVEL= logging.DEBUG
        super(Camera, self).__init__(device_handle)

        # self.enable_stream()

        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)


        self.device_handle = device_handle
        self.camId = None

        # Open device and activate freerun mode
        self.openDevice()
        self.TriggerMode = None
        self.setTriggerMode(None)
        self.logger.debug('Camera initilised')



    def __del__(self):
        print("stop")

    def _checkDeviceHandle(self, device_handle):

        if isinstance(device_handle, (list, tuple)):
            device_handle = device_handle[0]

        self.logger.debug('Opening device {device_handle}'
                          ''.format(device_handle=device_handle))
        # Search for mac addresses in form 'DEV_XXXXXXXXXXXX'
        candidates = re.findall(r'([0-9A-Z]{11,13})', device_handle)
        if len(candidates) == 0:
            # no mac address found: search for IP
            candidates = re.findall(r'[0-9]+(?:\.[0-9]+){3}', device_handle)

        return candidates[0]


    def _getCamId(self):
        return self.device._info

    @staticmethod


    def isOpen(self):

        return self.stream_enabled

    def getFrames(self, *args, **kwargs):
        #records expected images +5, because the projector need some (uncertain) time to respond after the start of the camera pipeline.
        # the additional images are removed in the "convert_to_image" function based on their brightness
        return(self.getFrame() for _ in range(self._expected_images+10))


    def convert_to_image(self,framelist):
        brightness = []
        arraylist = []

        for c, frame in enumerate(framelist):
            image = np.asanyarray(frame.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # gray = image
            # img_name = "images/color_" + str(c) + ".png"
            # cv2.imwrite(img_name, image)
            array = np.asanyarray(gray)
            brightness.append(np.mean(array))
            arraylist.append(array)
        th = np.mean(brightness[:])
        self.logger.debug('Mean brightness is {br}'
                          ''.format(br=th))

        for array in arraylist:
            self.logger.debug('Brightness is {br}'
                              ''.format(br=np.mean(array)))

            if np.mean(array)>(0.9*th):
            # if np.mean(array) > 19:
                flipimg = array[90:990, 550:1450]
            #     flipimg = array[:, 450:1530]
            # flipimg = array[20:1060, 470:1510]
                self.imgData.append(flipimg)


        if len(self.imgData)<self._expected_images:
            print("Number of images is",len(self.imgData))
            raise Exception('Number of captured images below number of expected_images. Try lowering the brightness threshold!')

        return self.imgData

    def saveimages(self,imgData):

        for c, value in enumerate(imgData, 1):
            img_name = "images/image_" + str(c) + ".png"
            # print(np.mean(value))
            # value = cv2.equalizeHist(value)
            cv2.imwrite(img_name, value)
        self.logger.debug('saved {num} images '
                          ''.format(num=len(imgData)))

    def getFrame(self, *args, **kwargs):
        self.color_sensor.set_option(rs.option.exposure, (self.Exposure))
        # self.depth_sensor.set_option(rs.option.exposure, (self.Exposure))
        frames = self.pipeline.wait_for_frames()

        frame= frames.get_color_frame()

        return frame


    def record(self):
        time.sleep(3)
        self.logger.debug('Recording {num} images'
                          ''.format(num=self._expected_images))
        self.depth_sensor.set_option(rs.option.output_trigger_enabled, 1.0)
        # self.depth_sensor.set_option(rs.option.output_trigger_enabled, 0.0)



        framelist = self.getFrames()
        self.imgData = self.convert_to_image(framelist)
        self.saveimages(self.imgData)
        self.depth_sensor.set_option(rs.option.output_trigger_enabled, 0.0)
        return copy.deepcopy(self.imgData)

    def prepareRecording(self, num):
        self.stop()
        self.imgData = []
        self._expected_images = num
        self.init_sensors()

        self.color_sensor.set_option(rs.option.exposure, (self.Exposure))
        self.depth_sensor.set_option(rs.option.exposure, (self.Exposure))

        self.setTriggerMode('Out')
        self.logger.debug('Prepare recording {num} images'
                          ''.format(num=self._expected_images))
        time.sleep(1)


    def listFeatures(self):
        """
        Lists camera features
        """
        try:
            self.logger.debug('Listing camera features')
            featureNames = self.device.getFeatureNames()
            print("Printing feature names: ...\n")
            print("\n".join(featureNames))
        except Exception as e:
            self.logger.exception('Failed to get feature names: '
                                  '{e}'.format(e=e))

    def setExposureMicrons(self, microns=None):

        self.logger.debug('Setting <ExposureTime> to {microns}'
                          ''.format(microns=microns))
        self.Exposure = microns




        return self.Exposure

    def setResolution(self, resolution=None):

        if resolution is not None:
            self.resolution[0]= resolution[0]
            self.resolution[1] = resolution[1]
            self.Width=resolution[0]
            self.Height=resolution[1]

            # self.stop()
            # self.enable_stream()
            # self.setTriggerMode()
            self.logger.debug('Setting <Resolution> to {Resolution}'
                              ''.format(Resolution=resolution))


        return self.Width, self.Height


    def stop(self):
        if self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False
        self.config.disable_all_streams()
        self.stream_enabled = False



    def setTriggerMode(self, mode=None):

        if mode == 'Out':

            if self.depth_sensor.supports(rs.option.output_trigger_enabled):
                self.depth_sensor.set_option(rs.option.output_trigger_enabled, 0.0)

                time.sleep(0.5)
                self.TriggerMode=mode
                print(self.depth_sensor.get_option(rs.option.output_trigger_enabled))
            else:
                 self.logger.exception('Failed to set Triggermode to {mode}'
                                  ''.format(mode=mode))

        if mode == 'off':
            if self.depth_sensor.supports(rs.option.output_trigger_enabled):
                self.depth_sensor.set_option(rs.option.output_trigger_enabled, 0.0)
                time.sleep(0.5)
                self.TriggerMode=mode
            else:
                self.logger.exception('Failed to set Triggermode to {mode}'
                                      ''.format(mode=mode))

        self.logger.debug("Setting trigger mode to: {mode}".format(mode=mode))


        return self.TriggerMode



    def _checkDeviceHandle(self, device_handle):

        if isinstance(device_handle, (list, tuple)):
            device_handle = device_handle[0]

        self.logger.debug('Opening device {device_handle}'
                          ''.format(device_handle=device_handle))
        # Search for mac addresses in form 'DEV_XXXXXXXXXXXX'
        candidates = re.findall(r'([0-9A-Z]{11,13})', device_handle)
        if len(candidates) == 0:
            # no mac address found: search for IP
            candidates = re.findall(r'[0-9]+(?:\.[0-9]+){3}', device_handle)

        return candidates[0]

    def openDevice(self):
        """
        Opens a camera device with the stored self.device object
        """

        try:
            self.logger.debug('Opening camera device')
        except Exception as e:
            self.logger.exception('Failed to open the camera device: '
                                  '{e}'.format(e=e))


    def prepare_live(self):
        self.init_sensors()
        self.setTriggerMode('Off')



        # self.color_sensor.set_option(rs.option.exposure, (self.Exposure)*0.02)
        # self.depth_sensor.set_option(rs.option.exposure, (self.Exposure) * 0.02)


    def getImage(self):
        frame = self.getFrame()
        # print(self._expected_images)
        self.color_sensor.set_option(rs.option.exposure, (self.Exposure))
        self.depth_sensor.set_option(rs.option.exposure, (self.Exposure))

        array = np.asanyarray(frame.get_data())
        # flipimg = cv2.flip(array,-1)
        # flipimg = flipimg[80:1080, 500:1500]
        # flipimg = flipimg[40:1040, 500:1500]
        # flipimg[500, :] = 0
        # flipimg[:, 500] = 0
        # flipimg = array
        # flipimg = array[:, 400:1480]
        # flipimg = array[20:1060, 470:1510]
        flipimg = array[90:990, 550:1450]
        # print(np.mean(flipimg))

        return flipimg


    def init_sensors(self):
        # Low Resolution of depth sensor
        self.config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16,
                                  self.framerate)


        # self.config.enable_stream(rs.stream.color, self.resolution[0], self.resolution[1], rs.format.bgr8,
        #                           self.framerate)
        #                           self.framerate)

        self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8,
                                  self.framerate)
        # self.profile = self.pipeline.stop(self.config)

        if self.pipeline_started:
           self.pipeline.stop()
           self.pipeline_started = False

        self.profile = self.pipeline.start(self.config)
        self.pipeline_started =True


        self.dev = self.profile.get_device()

        print(self.dev.query_sensors()[0])
        self.depth_sensor = self.dev.query_sensors()[0]
        print("Deph Sensor:",self.depth_sensor)
        self.color_sensor = self.dev.query_sensors()[1]


        if self.depth_sensor.supports(rs.option.laser_power):
            self.depth_sensor.set_option(rs.option.laser_power, 0)
            time.sleep(0.5)
        else:
            print("laser off not possible")
            pass

        def cleanUp(self):
            """
            Does some cleanup jobs. Call after "AcquisitionStop".
            Calls:  - endCapture()
                    - flushCaptureQueue()
                    - revokeAllFrames()
            """
            self.depth_sensor.set_option(rs.option.output_trigger_enabled, 0.0)

            self.device.endCapture()
            self._clearQueueAndFrames()



if __name__ == '__main__':
    import logging
    import cv2 as cv

    # logging.basicConfig(level=logging.DEBUG)/home/wrangel/dev/usbCamera
    # logging.basicC/home/wrangel/dev/ownsplibonfig(level=logging.DEBUG)
    logging.basicConfig(level=logging.DEBUG)
    bListFeatures = False
    bLiveView = False

    contr = Controller()

    cam = Camera(contr)

    # cam.setTriggerMode('Out')
    cam.Exposure =2500
    # cam.color_sensor.set_option(rs.option.exposure, 400)

    W = 800


    cam.init_sensors()
    cam.prepare_live()
    img = cam.getImage()
    # cam.pipeline.start()


    height, width, depth = img.shape
    print(height)
    print(width)
    print(depth),
    imgScale = W / width
    newX, newY = img.shape[1] * imgScale, img.shape[0] * imgScale

    cv2.namedWindow("cam", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("cam", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    while True:

        img = cam.getImage()
        img[int(height/2)-2:int(height/2)+2,:,0]=100
        img[:,int(width / 2) - 2:int(width / 2) + 2, 0] = 100

        # newimg = cv2.resize(img, (int(newX), int(newY)))
        cv2.imshow('cam', img)


        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

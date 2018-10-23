#!/usr/bin/env python
'''
    Implementation of Intel Realsense Cameras D4xx  (D415 and D435)
'''

__author__ = 'Benjamin Fehlandt'
__email__ = 'benjamin.fehlandt@imr.uni-hannover.de'
__status__ = 'Development'

# format = setFeature({key1: value1, key2: value2, ..., keyN, valueN})

import abc
import logging
import pyrealsense2 as rs

import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as op  # (pip install open3d-python)

import os
import numpy as np
import tables
import sys

from cameraTemplate import ControllerTemplate, CameraTemplate
# installed by this description: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
LOGGING_LEVEL=None

class Controller(ControllerTemplate):
    """
        Template class for spectrometer controllers to inherit from if they are
        necessary to use the camera. The controller for camera devices should
        be opened and closed from the Camera.openController and
        Camera.closeController static methods. If the controller is only used to
        detect available camera devices, that logic could also be easily
        implemented in Camera.listDevices().
    """

    def __init__(self, num_of_cams=1):
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.logger.debug('Starting Intel Realsense D435 Camera Controller')
        self.num_of_cams = num_of_cams
        self.device_handles = {}

        self.use_color = False
        self.use_depth = False
        self.use_ir = False

    def listDevices(self, streams=('color', 'depth', 'ir_left', 'ir_right', 'registered', 'pointcloud')):
        """
            Returns a list of available devices. One of these entries should be
            used as parameter for self.getDevice to open the corresponding device

            :param streams: str
                    if streams == 'color', 'depth', 'ir_left', 'ir_right', 'registered', 'pointcloud'  or a few of them
                                  :return device_handle will configure a single camera for each stream
                    if streams == 'alltogether',
                                  :return device_handler will configure every stream in a single camera
            :return device_handles : list
                     List of available devices
        """
        self.updateDeviceHandles(streams=streams)
        return self.device_handles

    def updateDeviceHandles(self, streams):
        """
            Update the list of available device handles
        """
        self.logger.debug('\nSearching for camera devices')
        self.device_handles = {}
        self.streams = streams
        if not self.streams == 'alltogether':
            frames = ('color', 'depth', 'ir_left', 'ir_right', 'registered', 'pointcloud')

        for i in range(self.num_of_cams):
            if streams == 'alltogether':
                # configure channels
                self.camera = Camera(device_handle=[], stream=streams)
                config = self.camera.configuration(stream=streams)
                self.device_handles[streams] = config
            else:
                for idx, stm in enumerate(frames):
                    # configure channels
                    self.camera = Camera(device_handle=[], stream=stm)
                    config = self.camera.configuration(stream=stm)
                    self.device_handles[stm] = config

        self.logger.debug('\tFound {num} camera stream(s): {devices}'
                          ''.format(num=len(self.device_handles),
                                    devices=self.device_handles))

    def closeController(self):
        '''
            Delete all detected devices
        '''
        for cap in self.device_handles:
            del cap

    def __del__(self):
        self.logger.debug('Deleting cameracontroller {self}'
                          ''.format(self=self))
        self.closeController()

    def __repr__(self):
        return "<Intel Realsense D4xx Controller>"


class Camera(CameraTemplate):
    """
    Template class for camera objects to inherit from. The following methods
    have to be implemented to be able to instantiate the camera object
    (decorated with @abc.abstractmethod):
        - CameraTemplate.listDevices() (static method)
        - self.openDevice()
        - self.closeDevice()
        - self.getImage()
    """
    def __init__(self, device_handle, stream,
                       color_res=(1080, 1920, 3), registered_res=(720, 1280, 3), depth_res=(720, 1280), ir_res=(720, 1280),
                       colorFormat='rgb8', depthFormat='z16', irFormat='y8',
                       color_fps=30, depth_fps=30, ir_fps=30, using_open3d=False,
                       record_mode=False, dir_store=None):
        '''
            interface for D415 and D435 intel realsense camera

        Parameters
        ----------
            device_handle : object

            resolution (default values)
                color_res = (1080, 1920)
                ir_res = (720, 1280)
                depth_res = (1280, 720)
                (recommended by intel: for D415 - 720x1280
                                       for D435 - 420x848 )
            format (default values)
                check available formats here: https://github.com/toinsson/pyrealsense/blob/master/docs/rs.h
                color_format = 'rgb8', 3 channeles 8 Bit each
                depth_fomat = 'z16', 1 channel with 16 Bit, depth stream in mm
                ir_format = 'y8', 1 channel with 8 Bit
            time resolution (default values)
                available values are 15, 30, 60, 90
                color_fps = 30
                depth_fps = 30
                ir_fps = 30
        '''
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        super(CameraTemplate, self).__init__()
        self.device_handle = device_handle  # Use this to identify and open the
                                            # device
        self.device = rs.pipeline()  # Use this variable to store the device itself
        # (open pipeline)
        self.start = True

        # certify resolution
        self.color_res = color_res
        self.depth_res = depth_res
        self.registered_res = registered_res
        self.ir_res = ir_res

        self.stream = stream
        self.frames = {}
        try:
            if self.stream == 'color' or self.stream == 'alltogether':
                self.frames['color'] = np.zeros((self.color_res[0], self.color_res[1], self.color_res[2]))
            if self.stream == 'registered' or self.stream == 'alltogether':
                self.frames['color_aligned'] = np.zeros((self.registered_res[0], self.registered_res[1], self.registered_res[2]))
        except:
            if self.stream == 'color' or self.stream == 'alltogether':
                self.frames['color'] = np.zeros((self.color_res[0], self.color_res[1]))
            if self.stream == 'registered' or self.stream == 'alltogether':
                self.frames['color_aligned'] = np.zeros((self.registered_res[0], self.registered_res[1]))
        if self.stream == 'depth' or self.stream == 'alltogether':
            self.frames['depth'] = np.zeros((self.depth_res[0], self.depth_res[1]))
        # if self.stream == 'registered' or self.stream == 'alltogether':
        #     self.frames['depth_aligned'] = np.zeros((self.registered_res[0], self.registered_res[1]))
        if self.stream == 'ir_left' or self.stream == 'alltogether':
            self.frames['ir_left'] = np.zeros((self.ir_res[0], self.ir_res[1]))
        if self.stream == 'ir_right' or self.stream == 'alltogether':
            self.frames['ir_right'] = np.zeros((self.ir_res[0], self.ir_res[1]))


        # format  # check available format here: https://github.com/toinsson/pyrealsense/blob/master/docs/rs.h
        if colorFormat != 'rgb8':
            if   colorFormat == 'any':    self.color_format = rs.format.any  # default value
            elif colorFormat == 'bgr8':   self.color_format = rs.format.rgb8
            elif colorFormat == 'rgba8':  self.color_format = rs.format.bgra8
            elif colorFormat == 'bgra8':  self.color_format = rs.format.rgba8
            elif colorFormat == 'yuyv':   self.color_format = rs.format.yuyv
            elif colorFormat == 'raw16':  self.color_format = rs.format.raw16;
            else:  raise('choose other color format')
            self.colorFormat = colorFormat

            if np.logical_or(colorFormat=='yuyv', colorFormat=='raw16'):
                self.color_res = (self.color_res[0], self.color_res[1])  # correct resolution from 3 to 2 channels
        else:
            self.colorFormat = 'rgb8'
            self.color_format = rs.format.bgr8

        if depthFormat != 'z16':
            self.depthFormat = depthFormat
            if depthFormat == 'any':  self.depth_format = rs.format.any  # default value
            else:  raise('choose other depth format')
        else:
            self.depthFormat = 'z16'
            self.depth_format = rs.format.z16

        if irFormat != 'y8':
            self.irFormat = irFormat
            if irFormat == 'any':  self.ir_format = rs.format.any  # default value
            else:  raise('choose other ir format')
        else:
            self.irFormat = 'y8'
            self.ir_format = rs.format.y8

        # time resolution
        self.color_fps = color_fps
        self.depth_fps = depth_fps
        self.ir_fps = ir_fps

        self.using_open3d = using_open3d
        if self.using_open3d:
            self.pcd = op.PointCloud()

        self.record_mode = record_mode
        if record_mode:
            self.filename = 'test.hdf'
            self.record_number = 1
            if dir_store != None:
                self.record_mode = record_mode
                self.dir_store = dir_store
                self.atomColor = tables.UInt8Atom()  #ToDo: check atom for 2-channel color array, check atom for point cloud
                self.atomDepth = tables.UInt16Atom()  #= tables.Float32Atom()
                self.atomIr = tables.UInt8Atom()
            else:
                raise('add a storage directory before start recording')

    @staticmethod
    @abc.abstractmethod
    def listDevices(streams=('color', 'depth', 'ir_left', 'ir_right', 'registered', 'pointcloud')):
        """
            List all available camera devices correspponding to the class type
            Each entry of this list can be used as an argument for this class constructor

            :param streams: str
                    if streams == ('color', 'depth', 'ir_left', 'ir_right', 'registered', 'pointcloud') or a few of them
                                  :return object will configure a single camera for each stream
                    if streams == 'alltogether',
                                  :return object will configure every stream in a single camera
        """
        return Controller().listDevices(streams=streams)

    @abc.abstractmethod
    def openDevice(self, config, stream=None):
        """
        Open the device by using self.device_handle and store the device in
        self.device
        """
        # # look for result
        # self.ctx = rs.context()
        # self.cam = self.ctx.query_devices()
        #
        # # configure channels
        # config = self.configuration(stream=stream)
        self.config = config
        if stream != None:
             stream = self.stream

        if stream == 'registered' or stream == 'alltogether':
            # # registration of color onto depth frame
            # self.registration = rs.stream.color
            # self.registration = rs.align(self.registration)

            # register depth onto color stream
            self.registration = rs.stream.depth
            self.registration = rs.align(self.registration)

        if stream == 'pointcloud' or stream == 'alltogether':
            self.pc = rs.pointcloud()
            self.points = rs.points

        try:
            self.profile = self.device.start(self.config)

            ### set laser power to value set_laser
            dev = self.profile.get_device()
            sensor = dev.query_sensors()[0]
            self.laser_pwr = sensor.get_option(rs.option.laser_power)
            print("laser power = ", self.laser_pwr)
            laser_range = sensor.get_option_range(rs.option.laser_power)
            print("laser power range =", laser_range.min, "~", laser_range.max)
            self.set_laser = 20
            sensor.set_option(rs.option.laser_power, self.set_laser)
            self.laser_pwr = sensor.get_option(rs.option.laser_power)
            print("laser power = ", self.laser_pwr)
            return self.profile.get_device().first_depth_sensor()

        except:
            raise('close other camera before starting a new one')

    # individual method
    def configuration(self, stream):
        '''
            configuration of each data stream
            including data resolution and format

            :param stream: str
        '''
        config = rs.config()

        if stream == 'color' or stream == 'pointcloud' or stream == 'alltogether':
            config.enable_stream(rs.stream.color, self.color_res[1], self.color_res[0],
                                                  self.color_format, self.color_fps)
        if stream == 'depth' or stream == 'pointcloud' or stream == 'alltogether':
            config.enable_stream(rs.stream.depth, self.depth_res[1], self.depth_res[0],
                                                  self.depth_format, self.depth_fps)
        if stream == 'ir_left' or stream == 'alltogether':
            config.enable_stream(rs.stream.infrared, 1, self.ir_res[1], self.ir_res[0],
                                                        self.ir_format, self.ir_fps)  # left ir stream
        if stream == 'ir_right' or stream == 'alltogether':  # right ir stream
            config.enable_stream(rs.stream.infrared, 2, self.ir_res[1], self.ir_res[0],
                                                        self.ir_format, self.ir_fps)  # right ir stream
        if stream == 'registered' or stream == 'alltogether':
            config.enable_stream(rs.stream.color, self.color_res[1], self.color_res[0],
                                 self.color_format, self.color_fps)
            config.enable_stream(rs.stream.depth, self.depth_res[1], self.depth_res[0],
                                 self.depth_format, self.depth_fps)

        return config

    @abc.abstractmethod
    def closeDevice(self):
        """
        Close the connection to the device and reset self.device to None.
        """
        self.device.stop()

    def getImage(self, pc_threshold_min=0, pc_threshold_max=None):
        """
            Blocking function that waits for num images to be recorded and returns
            an iterable of numpy arrays corresponding to images. Recording of
            images is done according to the currently set trigger mode!

            If a time sensitive image acquisition task is done consider using the
            separate self.prepareRecording(num) and self.record() functions to
            achieve the same result.

            Parameters
            ----------
            pc_threshold : int
                threshold for z coordinate

                if pc_threshold == None, there is no threshold applied
        """
        self.images = self.device.wait_for_frames()  #ToDo: change resolution if desired (others than default)
        if self.start == True:
            for i in self.images:  # print profile of each stream
                print(i.profile)
            print('\n')
            self.start = False

        if self.stream == 'registered' or self.stream == 'alltogether':
            # self.registeredD = self.registration.process(self.images)
            # self.registeredD.first(rs.stream.depth)
            # self.frames['depth_aligned'] = self.get_npImage(self.registeredD.get_color_frame())

            self.registered = self.registration.process(self.images)
            self.registered.first(rs.stream.color)
            self.frames['color_aligned'] = self.get_npImage(self.registered.first(rs.stream.color))
            self.frames['registered'] = self.frames['color_aligned']

        if self.stream == 'color' or self.stream == 'alltogether' or \
           np.logical_and(self.stream == 'pointcloud', self.using_open3d):
            self.frames['color'] = self.get_npImage(self.images.get_color_frame())

        if self.stream == 'depth' or self.stream == 'alltogether' or \
           (self.stream == 'pointcloud' and self.using_open3d):
            self.frames['depth'] = self.get_npImage(self.images.get_depth_frame())

        if self.stream == 'ir_left' or self.stream == 'alltogether':
            self.frames['ir_left'] = self.get_npImage(self.images.get_infrared_frame())  # for left frame, index may be omitted, 0 or 1

        if self.stream == 'ir_right' or self.stream == 'alltogether':
            self.frames['ir_right'] = self.get_npImage(self.images.get_infrared_frame(2))  # index has to be 2

        if self.stream == 'pointcloud' or self.stream == 'alltogether':

            self.pc.map_to(self.images.get_color_frame())
            self.points = self.pc.calculate(self.images.get_depth_frame())

            #self.frames['vertices'] = np.array(np.asanyarray(self.points.get_vertices()).tolist())
            #self.frames['texture'] = np.array(np.asanyarray(self.points.get_texture_coordinates()).tolist())
            vtx = np.asanyarray(self.points.get_vertices())
            vtx = vtx.tolist()
            self.frames['vertices'] = np.array(vtx)

            if self.using_open3d:
                self.pcd.points = op.Vector3dVector(self.frames['vertices'])

            if pc_threshold_max != None:
                assert(pc_threshold_min < pc_threshold_max)
                dist = np.linalg.norm(self.frames['vertices'], axis=1)
                # idx = np.where(np.logical_and(pc_threshold_min < dist, dist < pc_threshold_max))[0]

                idx = np.where(np.logical_and(pc_threshold_min < self.frames['vertices'][:, 2],
                                              self.frames['vertices'][:, 2] < pc_threshold_max))[0]

                self.frames['vertices'] = self.frames['vertices'][idx,:]
                #self.frames['texture'] = self.frames['texture'][idx,:]


        if self.stream == 'pointcloud' or self.stream == 'alltogether':
            if self.using_open3d:
                return self.frames, self.pcd
            else:
                return self.frames
        else:
            return self.frames[self.stream]

    @abc.abstractmethod
    def get_npImage(self, frame):
        '''
            Return a numpy array containing an image
        '''
        return np.asanyarray(frame.get_data())

    def prepareRecording(self, filename=None, record=None, num=1):
        """
        if mode == None
            set record_mode = True, if False
                record_mode = False, if True
        otherwise,
            record_mode = mode

        Parameters
        ----------
        mode: bool
            set record_mode
        num : int
            Number of images that should be recorded
        """
        if filename != None:
            self.filename = filename + '.hdf'

        if record == None:
            if self.record_mode:  # == True
                self.record_mode = False
            else:  # self.record_mode == False
                self.record_mode = True
        else:
            self.record_mode = record

        self.atomColor = tables.UInt8Atom()  # ToDo: check atom for 2-channel color array
        self.atomDepth = tables.UInt16Atom()  #= tables.Float32Atom()
        self.atomIr = tables.UInt8Atom()
        self.atomPc = tables.Float64Atom()

        if num > 0:
            self.record_number = num-1
            return self.record_number

    def set_filename(self, filename):
        self.filename = filename + '.hdf'
        stop = 1

    def set_directory(self, dir):
        self.dir_store = dir  # + '/'

    def record(self, stream='alltogether'):
        """
        Blocking image acquisition of a previously defined number of images
        (see prepareRecording).

        Returns
        -------
        imgs : list
            List of numpy arrays containing the recorded images
        """
        if self.dir_store != None:
            print(os.path.join(self.dir_store, self.filename))
            h5file = tables.open_file(os.path.join(self.dir_store, self.filename), 'w')
        else:
            h5file = tables.open_file(self.filename, 'w')

        gcolumn = h5file.create_group(h5file.root, 'SensorData', 'SensorData')
        try:
            fmtColor = (self.color_res[0], self.color_res[1], self.color_res[2])
            fmtReg = (self.registered_res[0], self.registered_res[1], self.registered_res[2])
        except:
            fmtColor = (self.color_res[0], self.color_res[1])
            fmtReg = (self.registered_res[0], self.registered_res[1])

        if stream == 'color' or stream == 'alltogether':
            h5file.create_carray(gcolumn, 'color',
                                 self.atomColor, fmtColor,
                                 obj=self.frames['color'])
        if stream == 'registered' or stream == 'alltogether':
            h5file.create_carray(gcolumn, 'color_aligned',
                                 self.atomColor, fmtReg,
                                 obj=self.frames['color_aligned'])
        if stream == 'depth' or stream == 'alltogether':
            h5file.create_carray(gcolumn, 'depth',
                                 self.atomDepth, (self.depth_res[0], self.depth_res[1]),
                                 obj=self.frames['depth'])
        if stream == 'ir_left' or stream == 'alltogether':
            h5file.create_carray(gcolumn, 'ir_left',
                                 self.atomIr, (self.ir_res[0], self.ir_res[1]),
                                 obj=self.frames['ir_left'])
        if stream == 'ir_right' or stream == 'alltogether':
            h5file.create_carray(gcolumn, 'ir_right',
                                 self.atomIr, (self.ir_res[0], self.ir_res[1]),
                                 obj=self.frames['ir_right'])
        if stream == 'pointcloud' or stream == 'alltogether':
            num_img = self.frames['vertices'].shape[0]
            h5file.create_carray(gcolumn, 'vertices',
                                 self.atomPc, (num_img, 3),
                                 obj=self.frames['vertices'])
            h5file.create_carray(gcolumn, 'texture',
                                 self.atomPc, (num_img, 2),
                                 obj=self.frames['texture'])
        h5file.close()


    def registerSharedFeatures(self):
        """
        Registration of shared features that should be the same for all camera
        implementations. E.g. ExposureMicrons, Resolution, Gain, Format and
        TriggerMode
        """
        self.logger.debug('Registering shared camera features')

        self.registerFeature('ExposureMicrons', self.setExposureMicrons)
        self.registerFeature('ExposureTime', self.setExposureMicrons)
        self.registerFeature('Exposure', self.setExposureMicrons)

        self.registerFeature('Resolution', self.setResolution)

        self.registerFeature('Gain', self.setGain)

        self.registerFeature('Format', self.setFormat)

        self.registerFeature('TriggerMode', self.setTriggerMode)
        self.registerFeature('Trigger', self.setTriggerMode)

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
        raise NotImplementedError

    def setResolution(self, resolution=None, stream=None):
        """
            Set the resolution of the camera to the given values in pixels or read
            the current resolution by passing None

        Parameters
        ----------
        resolution : tuple
            if None - resolution is given as a cell
            else: variable channel is required
        channel : string
            'color', 'depth' or 'ir'

        Returns
        -------
        resolution : tuple
            The set camera resolution after applying the passed value
        """
        if resolution == None:
            resolution = {}
            resolution['color'] = self.color_res
            resolution['depth'] = self.depth_res
            resolution['registered'] = self.registered_res
            resolution['ir'] =    self.ir_res
            print('resolution: '
                  '#\n\tcolor stream: ' + str(self.color_res) +
                  '#\n\tdepth stream: ' + str(self.color_res) +
                  '#\n\tinfrared stream: ' + str(self.color_res) +
                  '#\n\tregistered stream: ' + str(self.color_res))
            return resolution
        else:
            if stream != None:
                if stream == 'color':
                    if np.logical_or(self.colorFormat=='yuyv', self.colorFormat=='raw16'):
                        assert(np.shape(resolution)[0] == 2)  # check, if resolution has only 2 channelshttps://keras.io/why-use-keras/#keras-has-broad-adoption-in-the-industry-and-the-research-community
                    self.color_res = resolution
                elif stream == 'registered':  self.registered_res = resolution
                elif stream == 'depth':  self.depth_res = resolution
                elif stream == 'ir':     self.ir_res = resolution
            else:
                raise('add stream before set resolution')


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
        raise NotImplementedError

    def setFormat(self, fmt=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        fmt : str
            has to be rs format according to https://github.com/toinsson/pyrealsense/blob/master/docs/rs.h
            String describing the desired image format (e.g. "mono8"), or None
            to read the current image format
        channel : str
            'color', 'depth' or 'ir'
        Returns
        -------
        fmt : str
            The image format after applying the passed value
        """
        if fmt == None:
            fmt = {}
            fmt['color'] = self.frames['color'].dtype
            fmt['depth'] = self.frames['depth'].dtype
            fmt['ir'] =    self.frames['ir'].dtype
            fmt['color rs format'] = self.colorFormat
            fmt['depth rs format'] = self.depthFormat
            fmt['ir rs format'] =    self.irFormat
            return fmt
        else:
            if np.logical_or(fmt=='rgb8', fmt=='bgr8', fmt=='rgba8', fmt=='bgra8', fmt=='yuyv', fmt=='raw16'):
                self.colorFormat = fmt
                #ToDo: change functions for 3- and 1-channel rgb frames
                if fmt == 'rgb8':     self.color_format = rs.format.bgr8
                elif fmt == 'bgr8':   self.color_format = rs.format.rgb8
                elif fmt == 'rgba8':  self.color_format = rs.format.bgra8
                elif fmt == 'bgra8':  self.color_format = rs.format.rgba8
                elif fmt == 'yuyv':   self.color_format = rs.format.yuyv
                elif fmt == 'raw16':  self.color_format = rs.format.raw16

                if np.logical_or(fmt == 'yuyv', fmt == 'raw16'):
                    self.color_res = (self.color_res[0], self.color_res[1])  # correct resolution from 3 to 2 channels
                else:
                    self.color_res = (self.color_res[0], self.color_res[1], 3)
            elif fmt=='z16':  # for possible adaption
                self.depthFormat = fmt
                if fmt == 'z16':  self.depth_format = rs.format.z16
            elif fmt=='y8':  # for possible adaption
                self.irFormat = fmt
                if fmt == 'y8':  self.ir_format = rs.format.y8
            else:
                raise('choose other rs format')


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
        raise NotImplementedError


    def __del__(self):
        if self.device is not None:
            self.closeDevice()


if __name__ == '__main__':

    from time import time

    # choose a stream
    streams = ['color', 'depth', 'ir_left', 'ir_right', 'registered', 'pointcloud', 'alltogether']
    idx = 4

    logging.basicConfig(level=logging.DEBUG)
    devices = Camera.listDevices(streams[idx])

    using_open3d = True
    if streams[idx] == 'pointcloud' and using_open3d:
        d4xx = Camera(device_handle=devices[streams[idx]], stream=streams[idx], using_open3d=using_open3d)
        vis = op.Visualizer()
        vis.create_window()
    else:
        d4xx = Camera(device_handle=devices[streams[idx]], stream=streams[idx])

    # if streams[idx] == 'alltogether':
    #     d4xx.openDevice(config=devices, stream=streams[idx])
    # else:
    d4xx.openDevice(config=devices[streams[idx]], stream=streams[idx])

    #d4xx.setResolution(resolution=(500, 888), stream='ir')  #ToDo: enable change resolution, def getImage()
    print('Devices: ', devices)

    # for record mode
    directory = '../test_Camera Calibration'
    d4xx.set_directory(directory)
    frame_number = 0; init_record_number = 1; record_number = 0

    if streams[idx] == 'pointcloud' and using_open3d == False:
        fig = plt.figure()
        axes = fig.add_subplot(111, projection='3d')

    print(streams[idx])
    while True:
        if streams[idx] == 'pointcloud':
            frame = d4xx.getImage(pc_threshold_min=0, pc_threshold_max=2)
            step = 100
        else:
            frame = d4xx.getImage()

        if streams[idx] == 'color' or streams[idx] == 'alltogether':  # color frame
            cv2.namedWindow('color frame', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('color frame', 640, 360)
            cv2.moveWindow('color frame', 10, 10)  #576, 476)
            try:
                cv2.imshow('color frame', frame['color'])
            except:
                cv2.imshow('color frame', frame)

        if streams[idx] == 'depth' or streams[idx] == 'alltogether':  # depth frame
            # change depth stream's colormap
            try:
                depth_display = cv2.applyColorMap(cv2.convertScaleAbs(frame['depth'], alpha=0.03), cv2.COLORMAP_JET)
            except:
                depth_display = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)
            cv2.namedWindow('depth frame', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('depth frame', 512, 424)
            cv2.moveWindow('depth frame', 660, 10)
            cv2.imshow('depth frame', depth_display)

        if streams[idx] == 'registered' or streams[idx] == 'alltogether':
            # 'registered color frame'
            cv2.namedWindow('aligned color frame', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('aligned color frame', 512, 424)
            cv2.moveWindow('aligned color frame', 10, 500)  #576, 476)
            try:
                cv2.imshow('aligned color frame', frame['color_aligned'])
            except:
                cv2.imshow('aligned color frame', frame)
            # # 'registered depth frame'
            # cv2.namedWindow('aligned depth frame', cv2.WINDOW_NORMAL)
            # cv2.resizeWindow('aligned depth frame', 512, 425)
            # cv2.moveWindow('aligned depth frame', 60, 500)  #576, 476)
            # try:
            #     cv2.imshow('aligned depth frame', frame['depth_aligned'])
            # except:
            #     cv2.imshow('aligned depth frame', frame)
        if streams[idx] == 'ir_left' or streams[idx] == 'alltogether':  # left infrared frame
            cv2.namedWindow('left infrared frame', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('left infrared frame', 512, 424)
            cv2.moveWindow('left infrared frame', 700, 500)
            try:
                cv2.imshow('left infrared frame', frame['ir_left'])
            except:
                cv2.imshow('left infrared frame', frame)

        if streams[idx] == 'ir_right' or streams[idx] == 'alltogether':  # right infrared frame
            cv2.namedWindow('right infrared stream', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('right infrared stream', 512, 424)
            cv2.moveWindow('right infrared stream', 750, 500)
            try:
                cv2.imshow('right infrared stream', frame['ir_right'])
            except:
                cv2.imshow('right infrared stream', frame)

        if streams[idx] == 'pointcloud':
            if using_open3d:
                vis.add_geometry(frame[1])
                vis.update_geometry()
                vis.poll_events()
                vis.update_renderer()
            else:
                plt.cla()
                axes.plot(frame['vertices'][:,0], frame['vertices'][:,1], frame['vertices'][:,2], 'bx')
                plt.pause(0.001)
                #plt.draw()

                # arg = plt.waitforbuttonpress(0)  # 0 == run endless
                # # if keyboard button ==> True
                # # if mouse button ==>    False
                # #                        None, otherwise
                #
                # if arg == False:  # press mouse button to exit
                #     plt.close()
                #     break
                # elif arg == True or record_number > 0:  # press any key to save pointcloud in hdf file
                #     if record_number == 0:
                #         record_number = init_record_number
                #         print('\nrecording is started')
                #         t = time()
                #     record_number = d4xx.prepareRecording(record=True, num=record_number)
                #     print('\trecord frame: ' + str(record_number))
                #     filename = str(frame_number); d4xx.set_filename(filename=filename)
                #     d4xx.record(stream=streams[idx])
                #     if record_number == 0:
                #         print('\nrecording is finished after ' + str(time() - t) + ' seconds')
                #     frame_number += 1
        else:
            # display images
            key = cv2.waitKey(1) & 0xFF  # for 64Bit operating systems, for 32Bit: k = cv2.waitKey(0)
            if key == 27 or key == ord('q'):  # press ESC or 'q' key to exit
                cv2.destroyAllWindows()
                break
            elif key == ord('s') or record_number > 0:  # press 's' key to save frame
                if record_number == 0:
                    record_number = init_record_number
                    print('\nrecording is started')
                    t = time()
                filename = str(frame_number); d4xx.set_filename(filename=filename)
                record_number = d4xx.prepareRecording(record=True, num=record_number)
                print('\trecord frame: ' + str(record_number))
                d4xx.record(stream=streams[idx])
                if record_number == 0:
                    print('\nrecording is finished after ' + str(time()-t) + ' seconds')
                frame_number += 1

    d4xx.closeDevice()
    sys.exit(0)

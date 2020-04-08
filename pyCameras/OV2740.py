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

        cam = "CX3-UVC"
        v4l2path = "/sys/class/video4linux"
        for base, subs, filenames in os.walk(v4l2path, followlinks=True):
            for filename in filenames:
                if filename == "name":
                    pth = os.path.join(base, filename)
                    with open(pth, "r") as f:
                        name = f.read()
                        if cam in name:
                            device_input = os.path.split(base)[1]
                            self.device_handles.append(device_input)
                            return 1

        # cameras = ['/dev/video0']
        # for camera in cameras:
        #     open(camera, 'rb+', buffering=0)
        #     self.device_handles.append(index)


        # self.logger.debug('Found {num} OV2740 camera devices: {devices}'
        #                   ''.format(num=len(self.device_handles),
        #                             devices=self.device_handles))


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
        self.Exposure = 0
        self._expected_images = 0
        self.framerate = 6

        self.aligned_frames = 0

        self.TriggerMode = None
        self.openDevice()


    @staticmethod
    def listDevices():
        """
        List available realsense camera devices


        Controller :return: self.device_handles : list
        List of camera device handles that can be used to create a new camera instance
        """
        return Controller().listDevices()

    def openDevice(self):
        """
        Open the device for capturing images.

        :return:
        """
        try:
            self.device = open(str('/dev/' + self.devices[0]), 'rb+', buffering=0)

            print(">> get device capabilities")
            self.cp = v4l2.v4l2_capability()
            fcntl.ioctl(self.device, v4l2.VIDIOC_QUERYCAP, self.cp)

            print("Driver:", "".join((chr(c) for c in self.cp.driver)))
            print("Name:", "".join((chr(c) for c in self.cp.card)))
            print("Is a video capture device?", bool(self.cp.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE))
            print("Supports read() call?", bool(self.cp.capabilities & v4l2.V4L2_CAP_READWRITE))
            print("Supports streaming?", bool(self.cp.capabilities & v4l2.V4L2_CAP_STREAMING))

            print(">> device setup")
            self.fmt = v4l2.v4l2_format()
            self.fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            fcntl.ioctl(self.device, v4l2.VIDIOC_G_FMT, self.fmt)  # get current settings
            print("width:", self.fmt.fmt.pix.width, "height", self.fmt.fmt.pix.height)
            print("pxfmt:", "V4L2_PIX_FMT_YUYV" if self.fmt.fmt.pix.pixelformat == v4l2.V4L2_PIX_FMT_YUYV else self.fmt.fmt.pix.pixelformat)
            print("bytesperline:", self.fmt.fmt.pix.bytesperline)
            print("sizeimage:", self.fmt.fmt.pix.sizeimage)
            fcntl.ioctl(self.device, v4l2.VIDIOC_S_FMT, self.fmt)

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

    def closeDevice(self):
        """
        Close the connection to the device
        """
        if self.device is not None:
            try:
                self.logger.debug('Closing camera device: {device}'
                                  ''.format(device=self.device))
                self.device.Close()
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

        # Get first depth sensor -> for enabling settings like trigger mode
        depth_sensor = self.profile.get_device().first_depth_sensor()

        if mode is None:
            return self.TriggerMode

        elif mode == r'In':
            if depth_sensor.supports(rs.option.output_trigger_enabled):
                depth_sensor.set_option(rs.option.output_trigger_enabled, 1.0)
                self.logger.info("Camera In Trigger on")
                self.TriggerMode = mode
            else:
                self.logger.warning(f"Failed to set Triggermode {mode}")

            if depth_sensor.supports(rs.option.inter_cam_sync_mode):
                depth_sensor.set_option(rs.option.inter_cam_sync_mode, 2.0)
                self.logger.info("In Camera in inter cam mode Slave ")

        elif mode == r'Out':
            if depth_sensor.supports(rs.option.output_trigger_enabled):
                depth_sensor.set_option(rs.option.output_trigger_enabled, 1.0)
                self.logger.info("Camera Out Trigger on")
                self.TriggerMode = mode
            else:
                self.logger.warning(f"Failed to set Triggermode {mode}")

            if depth_sensor.supports(rs.option.inter_cam_sync_mode):
                depth_sensor.set_option(rs.option.inter_cam_sync_mode, 1.0)
                self.logger.info("Out Camera in inter cam mode Master ")

        elif mode == r'Off':
            if depth_sensor.supports(rs.option.output_trigger_enabled):
                depth_sensor.set_option(rs.option.output_trigger_enabled, 0.0)
                self.logger.info("Camera Trigger Off")
                self.TriggerMode = mode
            else:
                self.logger.warning(f"Failed to set Triggermode {mode}")

            if depth_sensor.supports(rs.option.inter_cam_sync_mode):
                depth_sensor.set_option(rs.option.inter_cam_sync_mode, 0.0)
                self.logger.info("Inter Camera mode default")

        return self.TriggerMode

    def prepareRecording(self, num):
        print(">> init mmap capture")
        req = v4l2.v4l2_requestbuffers()
        req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = v4l2.V4L2_MEMORY_MMAP
        req.count = 1  # nr of buffer frames
        fcntl.ioctl(vd, v4l2.VIDIOC_REQBUFS, req)  # tell the driver that we want some buffers
        print("req.count", req.count)

        buffers = []
        print(">>> VIDIOC_QUERYBUF, mmap, VIDIOC_QBUF")
        for ind in range(req.count):
            # setup a buffer
            buf = v4l2.v4l2_buffer()
            buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = v4l2.V4L2_MEMORY_MMAP
            buf.index = ind
            fcntl.ioctl(vd, v4l2.VIDIOC_QUERYBUF, buf)

            mm = mmap.mmap(vd.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                           offset=buf.m.offset)
            buffers.append(mm)

            # queue the buffer for capture
            fcntl.ioctl(vd, v4l2.VIDIOC_QBUF, buf)

        print(">> Start streaming")
        buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(vd, v4l2.VIDIOC_STREAMON, buf_type)

        print(">> Capture image")
        t0 = time.time()
        max_t = 1
        ready_to_read, ready_to_write, in_error = ([], [], [])
        print(">>> select")
        while len(ready_to_read) == 0 and time.time() - t0 < max_t:
            ready_to_read, ready_to_write, in_error = select.select([vd], [], [], max_t)

        print(">>> download buffers")
        index = 1
        buf = v4l2.v4l2_buffer()
        buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = v4l2.V4L2_MEMORY_MMAP

        # if self.pipeline_started:
        #     self.pipeline.stop()
        #     self.pipeline_started = False
        #
        # self.openDevice()
        #
        # self._expected_images = num + 2

        self.logger.info(f"Prepare recording {num} images")


    def record(self):
        self.logger.info(f"Recording {self._expected_images}")
        index = 1
        for i in range(Frames):  # capture 50 frames
            if index == 1:
                print("empyt frame passed")
                fcntl.ioctl(vd, v4l2.VIDIOC_DQBUF, buf)  # get image from the driver queue
                fcntl.ioctl(vd, v4l2.VIDIOC_QBUF, buf)  # request new image
                print(f"Frame : {index}")
            else:
                print(f"Frame : {index}")

                fcntl.ioctl(vd, v4l2.VIDIOC_DQBUF, buf)  # get image from the driver queue
                # print("buf.index", buf.index)

                mm = buffers[buf.index]

                image_bytestream = mm.read()
                mm.seek(0)
                fcntl.ioctl(vd, v4l2.VIDIOC_QBUF, buf)  # request new image

                image_bytearray = bytearray(image_bytestream)

                # image_struct = struct.unpack('>'+'H'*(1928*1088), image_bytes)

                image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=image_bytearray).astype(np.uint16)
                image = np.right_shift(image_array, 6).astype(np.uint16)

                # LibRaw Test:
                # proc = libraw.LibRaw()  # create RAW processor
                # proc.imgdata = image_array_right_shift
                # proc.open_file(image_array_right_shift)  # open file
                # proc.unpack()  # extract mosaic from file
                # mosaic = proc.imgdata.rawdata.raw_image

                # AndoridCamera Test:

                # cap = {"width": 1928,
                #        "height": 1088,
                #        "format": "raw",
                #        }
                # props = {}
                # r, gr, gb, b = convert_capture_to_planes(cap, image_array_right_shift, props)
                # # image = convert_raw_to_rgb_image(r, gr, gb, b, props, cap_res=None)
                # h = r.shape[0]
                # w = r.shape[1]
                # image = np.dstack([b, (gr + gb) / 2.0, r])
                #
                # norm_img = np.float64(image / 1024.0)

                # image = np.int16(image)
                # image = np.ndarray([r[:,:,0], (gb[:,:,0] + gr[:,:,0]) / 2, b[:,:,0]])
                # img = (((img.reshape(h, w, 3) - black_levels) * scale) * gains).clip(0.0, 1.0)
                # img = numpy.dot(img.reshape(w * h, 3), ccm.T).reshape(h, w, 3).clip(0.0, 1.0)

                # image = image_array_right_shift.reshape(1088, 1928)

                # with rawpy.imread(img) as raw:
                #     rgb = raw.postprocess()
                # imageio.imsave('default.tiff', rgb)

            index += 1

        # if self.pipeline_started is False:
        #     self.openDevice()
        #
        # self.img_data = []
        # img_bytes = []
        # color_img = None
        #
        # for sensor in self.profile.get_device().query_sensors():
        #     sensor.set_option(rs.option.frames_queue_size, 0)
        #
        # for _ in range(self._expected_images):
        #     frame = self.pipeline.wait_for_frames()
        #     img_bytes = frame.get_color_frame()
        #     color_image = np.asanyarray(img_bytes.get_data())
        #     color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        #     self.img_data.append(color_image)
        #

        fcntl.ioctl(vd, v4l2.VIDIOC_STREAMOFF, buf_type)
        self.closeDevice()

        return copy.deepcopy(self.img_data)

    def getImage(self):
        """
        Get an image from the camera

        :return: image : np.ndarray
            Image of active sensor from realsense device
        """

        if self.pipeline_started is False:
            self.openDevice()

        frames = self.pipeline.wait_for_frames()
        img_bytes = frames.get_color_frame()
        color_image = np.asanyarray(img_bytes.get_data())
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        return color_image#4mm endo

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
            print("exposure_curr", gc.value)

            # set control value
            gc.value = microns
            fcntl.ioctl(self.device, v4l2.VIDIOC_S_CTRL, gc)
            print("exposure set to: ", gc.value)

            # get control value
            fcntl.ioctl(self.device, v4l2.VIDIOC_G_CTRL, gc)
            print("exposure is", gc.value)

            self.Exposure = microns

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

    def setFps(self):
        pass


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

    cam.setExposureMicrons(500)
    cam.setTriggerMode("Out")
    # cam.setFramerate(6)

    cv2.namedWindow('OV2740 RAW Image', cv2.WINDOW_AUTOSIZE)
    index = 0
    while True:
        color_img = cam.getImage()
        index += 1

        if color_img is not None:
            if color_img.shape[0] > 640:
                cv2.imshow('OV2740 RAW Image', cv2.resize(color_img, (640, 480)))
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    del cam

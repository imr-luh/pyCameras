__author__ = "Pascal Kern"
__credits__ = "Niklas Kroeger"
__maintainer__ = "Pascal Kern, Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

import logging
import numpy as np
import v4l2
import fcntl
import mmap
import select
import time
import cv2
import matplotlib.pyplot as plt

import time
from threading import Event
import sys
import copy

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = logging.DEBUG


class Controller(ControllerTemplate):
    """ Camera Controller for the OV2740 camera."""

    def __init__(self):
        super(Controller, self).__init__()
        logging.basicConfig()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.logger.info("Starting Ov2740 Camera Controller")
        # Todo: what does this do?
        # self.context = rs.context()
        self.devices = []
        self.device_handles = []

    def listDevices(self):
        """
        Returns a list of available realsense devices. One or more of these devices can
        be used as parameter for self.getDevice to open corresponding Device

        :return: self.device_handles : list
            List of available capture devices
        """
        # Using the context we can get all connected devices in a device list
        self.logger.info("Searching for OV2740 Camera devices")
        self.devices = self.context.query_devices()
        if self.devices.size() == 0:
            self.logger.error("No device connected, please connect a camera device")
            raise ConnectionError

        self.logger.info("Found the following devices:")

        index = 0
        serial_number = str
        name = str

        for device in self.devices:
            if device.supports(rs.camera_info.name):
                name = device.get_info(rs.camera_info.name)

            if device.supports(rs.camera_info.serial_number):
                serial_number = device.get_info(rs.camera_info.serial_number)
            self.logger.info(f"Device Index {index} : {name} {serial_number}")
            index += 1

        self.updateDeviceHandles()
        return self.device_handles

    def updateDeviceHandles(self):
        """
        Updates the list of available device handles

        """
        # TODO Change this for multi camera setup
        sensors = self.devices[0].query_sensors()
        self.logger.info(f"Device consists of {len(sensors)} sensors:")

        index = 0
        for sensor in sensors:
            if sensor.supports(rs.camera_info.name):
                name = sensor.get_info(rs.camera_info.name)
            else:
                name = "Unknown Sensor"

            self.logger.info(f"Sensor Index {index} : {name}")
            self.device_handles.append(index)
            index += 1

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
        self.logger.debug('Opening device {device_handle}'
                          ''.format(device_handle=device_handle))
        return Camera(device_handle=device_handle)

    def closeController(self):
        """
            Delete all detected devices
        """
        for sensor in self.device_handles:
            del sensor
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
        return "<Realsense Camera Controller>"


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

        self.context = rs.context()
        self.devices = self.context.query_devices()
        self.pipeline_started = False
        if len(self.devices) == 0:
            raise ConnectionError

        #TODO Change this for multi realsense setup
        self.device = self.devices[0].query_sensors()[self.device_handle]

        self.pipeline = rs.pipeline(self.context)

        self.config = rs.config()

        self.stream_profile = self.device.get_stream_profiles()[0]

        self.profile = rs.pipeline_profile

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
        Open the device for capturing images. For Realsense camera starting pipeline with specific config.

        :return: self.pipeline.start(self.config) : pipeline_profile_object
        """

        if self.stream_profile is None:
            #TODO Change here for specifig streamprofiles
            pass

        else:
            vd = open('/dev/video0', 'rb+', buffering=0)
            print(">>> streamparam")  ## somewhere in here you can set the camera framerate
            parm = v4l2.v4l2_streamparm()
            parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            parm.parm.capture.capability = v4l2.V4L2_CAP_TIMEPERFRAME
            fcntl.ioctl(vd, v4l2.VIDIOC_G_PARM, parm)
            fcntl.ioctl(vd, v4l2.VIDIOC_S_PARM, parm)  # just got with the defaults

            # stream_type = self.stream_profile.stream_type()
            # format = self.stream_profile.format()
            # fps = self.stream_profile.fps()


        #TODO Change this for multicamera setup
        if len(self.context.query_devices()[0].query_sensors()) == 2:
            self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, self.framerate)
            self.logger.info(f"Start displaying Color profile {rs.stream.color, 1920, 1080, rs.format.bgr8, self.framerate}")

        self.config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 6)

        self.logger.info(f"Start displaying Depth profile {rs.stream.depth, 480, 270, rs.format.z16, self.framerate}")
        if self.pipeline_started is False:
            self.profile = self.pipeline.start(self.config)
            self.pipeline_started = True

    def closeDevice(self):
        """
        Release the device
        """
        self.logger.debug(f'Releasing device {self.device} capture for sensor {self.device_handle}')

        vd.close()


        del self.device
        self.device = None

        if self.pipeline_started:
            self.pipeline_started = False
            self.pipeline.stop()


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
            fcntl.ioctl(vd, v4l2.VIDIOC_QUERYCTRL, qc)
            print("exposure defalut: ", qc.default)

            # get control value
            gc = v4l2.v4l2_control()
            gc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
            fcntl.ioctl(vd, v4l2.VIDIOC_G_CTRL, gc)
            print("exposure_curr", gc.value)

            # set control value
            gc.value = exposure
            fcntl.ioctl(vd, v4l2.VIDIOC_S_CTRL, gc)
            print("exposure set to: ", gc.value)

            # get control value
            fcntl.ioctl(vd, v4l2.VIDIOC_G_CTRL, gc)
            print("exposure is", gc.value)

            self.device = self.profile.get_device().query_sensors()[1]
            self.device.set_option(rs.option.exposure, microns)
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

    def set_laser_off(self):
        """Setting the laser off"""
        depth_sensor = self.profile.get_device().first_depth_sensor()

        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, 0)
            self.logger.info("Setting Laser off")
        else:
            self.logger.info("Setting Laser off is not possible")

    def prepare_live(self, exposure):
        """Define live view exposure time to be less. So no overlighting occurs"""
        live_exposure = exposure * 0.05
        self.setExposureMicrons(microns=live_exposure)
        self.logger.info(f"Live View Exposure time : {live_exposure}µs")

    def get_sensor_options(self, sensor=None):
        """Function for getting the possible sensor options"""
        self.logger.info("Sensor supports following options:")

        # Todo: implement v4l2
        # cp = v4l2.v4l2_capability()
        # fcntl.ioctl(vd, v4l2.VIDIOC_QUERYCAP, cp)
        # print("Driver:", "".join((chr(c) for c in cp.driver)))
        # print("Name:", "".join((chr(c) for c in cp.card)))
        # print("Is a video capture device?", bool(cp.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE))
        # print("Supports read() call?", bool(cp.capabilities & v4l2.V4L2_CAP_READWRITE))
        # print("Supports streaming?", bool(cp.capabilities & v4l2.V4L2_CAP_STREAMING))

        # options = list(rs.option.__members__)
        # for i in range(len(options)):
        #     option_type = options[i]
        #     option = rs.option(i)
        #
        #     self.logger.info(f"{i} : {option_type}")
        #
        #     if self.device.supports(option):
        #         self.logger.info(f"Description : {self.device.get_option_description(option)}")
        #         self.logger.info(f"Current Value : {self.device.get_option(option)}")
        #
        #     if sensor is not None and sensor.supports(option):
        #         self.logger.info(f"Description : {sensor.get_option_description(option)}")
        #         self.logger.info(f"Current Value : {sensor.get_option(option)}")

    def __del__(self):
        """
        Camera object Destructor
        :return:
        """
        self.logger.debug(f"Deleting Camera Object {self}")

        if self.device is not None:
            del self.device

        if self.pipeline_started:
            self.pipeline_started = False
            self.pipeline.stop()


if __name__ == '__main__':
    Frames = 86
    img = getFrames(Frames, exposure=800)
    print(img.shape)
    print(img.dtype)
    img[0::2, 0::2] = np.multiply(img[0::2, 0::2], 1.8)
    img[1::2, 1::2] = np.multiply(img[1::2, 1::2], 1.7)
    # img[0::2, 0::2] *= 1.969 # Blue
    # img[1::2, 1::2] *= 1.707 # Red
    # img = img.astype(np.uint16)

    # cv2.imwrite("OV2740_UVC_Camera/RawBilder/bayer_rightshifted.tiff", img)
    # np.save('OV2740_UVC_Camera/NumpyRaw/frame_testpattern2', img)
    plt.imshow(img, cmap="gray", vmin=0, vmax=1024)
    plt.show()



    # result = cv2.cvtColor(img_norm, cv2.COLOR_BGR2LAB)
    # avg_a = np.average(result[:, :, 1])
    # avg_b = np.average(result[:, :, 2])
    # result[:, :, 1] = result[:, :, 1] - ((avg_a - 0.5) * (result[:, :, 0] / 1.0) * 1.1)
    # result[:, :, 2] = result[:, :, 2] - ((avg_b - 0.5) * (result[:, :, 0] / 1.0) * 1.1)
    # img_whitebalanced = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)


    # demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_bilinear(img, "BGGR")
    demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(img, "BGGR")
    # demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_Malvar2004(img, "BGGR")
    # demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_Menon2007(img, "BGGR")
    # demosaic_norm = np.dstack([demosaic_img[:,:,0] / np.max(demosaic_img[:,:,0]), demosaic_img[:,:,1] / np.max(demosaic_img[:,:,1]),demosaic_img[:,:,2] / np.max(demosaic_img[:,:,2])])

    demosaic_norm = demosaic_img.copy() / np.max(demosaic_img)
    img = demosaic_img.copy().astype(np.uint16)
    img2 = demosaic_norm.copy()

    # img_cc = stretch(img)

    # images = np.concatenate((demosaic_norm, demosaic_img), axis=1)
    plt.imshow(demosaic_norm)
    plt.show()

    # img_cc_norm = img_cc.copy() / np.max(img_cc)

    # plt.imshow(img_cc_norm)
    # plt.show()





    logger = logging.getLogger(__name__)
    available_devices = Camera.listDevices()
    logger.debug(f"Available Devices {available_devices}")

    # device_index = 1
    # if device_index <= len(available_devices):
    cam = Camera(1)
    # cv2.namedWindow('Color Image', cv2.WINDOW_AUTOSIZE)
    # cv2.namedWindow('Depth Image', cv2.WINDOW_AUTOSIZE)
    cam.set_laser_off()

    #cam.setResolution((640, 480))
    # cam.setExposureMicrons()
    cam.get_board_temperature()
    cam.setTriggerMode("Out")
    cam.setExposureMicrons(500)
    #cam.get_sensor_options()
    index = 0
    while True:
        color_img = cam.getImage()

        if index == 50:
            cam.get_board_temperature()
            index = 0

        index += 1

        if color_img is not None:
            if color_img.shape[0] > 640:
                cv2.imshow('Color Image', cv2.resize(color_img, (640, 480)))
        # if depth_img is not None:
        #     cv2.imshow('Depth Image', depth_img)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

__author__ = "Pascal Kern"
__credits__ = "Niklas Kroeger"
__maintainer__ = "Pascal Kern, Philipp Middendorf"
__email__ = "philipp.middendorf@imr.uni-hannover.de"
__status__ = "Development"

import logging
import pyrealsense2 as rs
import cv2
import numpy as np
import time
from threading import Event
import sys
import copy

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

LOGGING_LEVEL = logging.DEBUG


class Controller(ControllerTemplate):
    """ Camera Controller for the Intel Realsense camera based on the python wrapper pyrealsense2.
        This Controller was tested with a D415 camera. Consider building the new wrapper from the source files
        provided by the Intel Realsense github repository: https://github.com/IntelRealSense/librealsense"""

    def __init__(self):
        super(Controller, self).__init__()
        logging.basicConfig()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.logger.info("Starting Intel Realsense Camera Controller")
        self.context = rs.context()
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
        self.logger.info("Searching for Intel Realsense Camera devices")
        self.devices = self.context.query_devices()
        if self.devices.size() == 0:
            self.logger.error("No device connected, please connect a RealSense device")
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
        # TODO Change this for multi realsense setup
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
        self.logger.info("Intel Realsense Camera Controller shutdown")

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
        Capture device representing a realsense camera
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
        #TODO Here check for no connection

        self.device = self.devices[0].query_sensors()[self.device_handle]

        self.pipeline = rs.pipeline(self.context)

        self.config = rs.config()

        self.stream_profile = self.device.get_stream_profiles()[0]

        self.profile = rs.pipeline_profile

        self.img_data = []
        self._expected_images = 0
        self.framerate = 6

        self.aligned_frames = None

        self.openDevice()
        self.TriggerMode = None


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
            #Todo Change here for specifig streamprofiles
            pass

        else:
            stream_type = self.stream_profile.stream_type()
            format = self.stream_profile.format()
            fps = self.stream_profile.fps()

            # resolution = str(self.stream_profile).rsplit(" ", 3)[0].rsplit(" ", 1)[1].rsplit("x")

        #TODO Change this for multicamera setup
        if len(self.context.query_devices()[0].query_sensors()) == 2:
            # self.config.enable_stream(stream_type, int(resolution[0]), int(resolution[1]), format, fps)
            self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 6)
            self.logger.info(f"Start displaying Color profile {rs.stream.color, 1920, 1080, rs.format.bgr8, 6}")


        self.config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 6)

        self.logger.info(f"Start displaying Depth profile {rs.stream.depth, 480, 270, rs.format.z16, 6}")
        if self.pipeline_started is False:
            self.profile = self.pipeline.start(self.config)
            self.pipeline_started = True

    def closeDevice(self):
        """
        Release the device
        """
        self.logger.debug(f'Releasing device {self.device} capture for sensor {self.device_handle}')

        del self.device
        self.device = None

        if self.pipeline_started:
            self.pipeline_started = False
            self.pipeline.stop()

    def get_board_temperature(self):

        """Get current board and depth sensor temperature"""

        # depth_sensor = self.devices[0].query_sensors()[0]
        depth_sensor = self.profile.get_device().first_depth_sensor()

        # Get device temperature
        if depth_sensor.supports(rs.option.asic_temperature):

            depth_asic_temperature = depth_sensor.get_option(rs.option.asic_temperature)
            self.logger.info(f"Vision Processing Board (ASIC) temperature {depth_asic_temperature}°C")

            if 70 > int(depth_asic_temperature) > 60:
                self.logger.warning(f"Vision Processing Board (ASIC) temperature is over 60°C ({depth_asic_temperature})")

            if int(depth_asic_temperature) >= 70:
                self.logger.error(f"Vision Processing Board (ASIC) temperature is over 70°C ({depth_asic_temperature})")
                self.closeDevice()
                raise SystemExit

        if depth_sensor.supports(rs.option.projector_temperature):

            depth_projector_temperature = depth_sensor.get_option(rs.option.projector_temperature)
            self.logger.info(f"Depth Sensor (Projector) temperature {depth_projector_temperature}°C")

            if 110 > int(depth_projector_temperature) > 80:
                self.logger.warning(f"Depth Sensor (Projector) temperature is over 60°C ({depth_projector_temperature})")

            if int(depth_projector_temperature) >= 110:
                self.logger.error(f"Depth Sensor (Projector) temperature is over 110°C ({depth_projector_temperature})")
                self.closeDevice()
                raise SystemExit


    def setTriggerMode(self, mode=None):
        """
        Set the trigger mode of the camera to either "in", "out", or "off", or
        read the current trigger setting by passing None

        """

        # depth_sensor = self.devices[0].query_sensors()[0]
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
        if self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False

        self.openDevice()

        self._expected_images = num + 2

        self.logger.info(f"Prepare recording {num} images")


    def record(self):
        self.logger.info(f"Recording {self._expected_images}")

        if self.pipeline_started is False:
            self.openDevice()

        self.img_data = []
        img_bytes = []
        color_img = None

        for sensor in self.profile.get_device().query_sensors():
            sensor.set_option(rs.option.frames_queue_size, 0)

        for _ in range(self._expected_images):
            frame = self.pipeline.wait_for_frames()
            img_bytes = frame.get_color_frame()
            color_image = np.asanyarray(img_bytes.get_data())
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            self.img_data.append(color_image)

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

        return color_image


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
            self.device = self.profile.get_device().query_sensors()[1]
            print(self.device.get_option(rs.option.exposure))
            self.device.set_option(rs.option.exposure, microns)
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

        # self.device = self.devices[self.device_handle]

        self.stream_profile = self.device.get_stream_profiles()
        # print(self.stream_profile)
        # TODO check if sensors supports this settings
        # if resolution is not None:
        #
        #     print(str(resolution[0])+"x"+str(resolution[1]))
        #
        #     resolution_string = str(resolution[0])+"x"+str(resolution[1])
        #     if resolution_string in str(self.stream_profile):
        #         stream_profile_string_length = str(self.stream_profile).find(",", 0, len(str(self.stream_profile)))
        #         resolution_string_postion = str(self.stream_profile).find(resolution_string, 0, len(str(self.stream_profile)))
        #
        #         index = resolution_string_postion // stream_profile_string_length
        #         self.stream_profile = self.stream_profile[index]

        #TODO change this hardcoded profile to fps and etc
        self.stream_profile = self.stream_profile[12]
        # self.openDevice()

        #TODO return resolution
        return 1920, 1080

    def setFps(self):
        pass

    def set_laser_off(self):

        # depth_sensor = self.devices[0].query_sensors()[0]
        depth_sensor = self.profile.get_device().first_depth_sensor()

        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, 0)
            self.logger.info("Setting Laser off")
        else:
            self.logger.info("Setting Laser off is not possible")

    def get_sensor_options(self, sensor=None):
        self.logger.info("Sensor supports following options:")

        options = list(rs.option.__members__)
        for i in range(len(options)):
            option_type = options[i]
            option = rs.option(i)

            self.logger.info(f"{i} : {option_type}")

            if self.device.supports(option):
                self.logger.info(f"Description : {self.device.get_option_description(option)}")
                self.logger.info(f"Current Value : {self.device.get_option(option)}")

            if sensor is not None and sensor.supports(option):
                self.logger.info(f"Description : {sensor.get_option_description(option)}")
                self.logger.info(f"Current Value : {sensor.get_option(option)}")

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
    cam.setTriggerMode("out")
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
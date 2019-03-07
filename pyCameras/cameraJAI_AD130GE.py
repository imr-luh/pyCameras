import cv2
import numpy as np
from harvesters.core import Harvester, LoadLibraryException
from genicam2.genapi import OutOfRangeException
import logging
import os
import sys
from harvesters.test.helper import get_package_dir
from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

# Adjust me, if installation path differs from README
CTI_FILE = "/opt/mvIMPACT_Acquire/lib/x86_64/mvGenTLProducer.cti"

LOGGING_LEVEL = None


def get_cti_file_path(self):
    name = 'mvGenTLProducer'
    if name in os.environ:
        # Run tests with specified GenTL Producer:
        cti_file_path = os.getenv(name)
    else:
        try:
            import genicam2
        except ImportError:
            # Failed to import genicam2 module; suggest the expected
            # solution to the client:
            raise ImportError(
                'You must specify a target GenTL Producer either using '
                'HARVESTERS_TEST_TARGET or installing genicam2 module.')
        else:
            # Run tests with the default test target, TLSimu:
            dir_name = get_package_dir('genicam2')
            cti_file_path = os.path.join(dir_name, 'TLSimu.cti')
    return cti_file_path

class Controller(ControllerTemplate):
    def __init__(self):
        self.harvester = Harvester()
        self.harvester.add_cti_file(CTI_FILE)

        try:
            self.harvester.update_device_info_list()
        except LoadLibraryException:
            raise LoadLibraryException("Installation path differs from default value. Change constant CTI_FILE to correct file path.")

    def listDevices(self):
        self.harvester.update_device_info_list()
        tmp = self.harvester.device_info_list
        # This has to run, otherwise Problem with the camera class. Not the best solution will be worked
        self.harvester.reset()
        return
    pass

class Camera(CameraTemplate):
    """
    JAI Camaera implemntaion based on Harvester(https://github.com/genicam/harvesters)

    """
    def __init__(self, device_handle, harvester=None):
        if harvester == None:
            self._harverster = Harvester()
        else:
            self._harverster = harvester

        self.device_handle = device_handle

        super(Camera, self).__init__(device_handle)

        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.openDevice()

        self.node_map = self.device.device.node_map

        self.img_list = list()

        self.node_map.Width.value = self.node_map.WidthMax.value

        self.node_map.Height.value = self.node_map.HeightMax.value
        self._expected_triggered_images = 16

    def __del__(self):
        self.closeDevice()
        self._harverster.reset()

    def _liveView(self):
        cv2.startWindowThread()
        cv2.namedWindow("IMG", 2)
        cv2.resizeWindow("IMG", 900, 900)

        self.logger.debug("Starting live view image acquisition.")
        self.device.start_image_acquisition()

        framecount = 0
        droppedframes = []

        while True:
            try:
                with self.device.fetch_buffer() as buffer:
                    imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                         dtype=np.uint16,
                                         shape=((buffer.payload.components[0].height,
                                                 buffer.payload.components[0].width))).copy()
                    imgData = cv2.cvtColor(imgData, cv2.COLOR_BAYER_RG2RGB)
                success = True
            except Exception as e:
                self.logger.debug(e)
                droppedframes.append(framecount)
                success = False

            if success:
                cv2.imshow('IMG', imgData)
            framecount += 1
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                cv2.destroyAllWindows()
                self.logger.info("Frames displayed: %i" % framecount)
                self.logger.info("Frames dropped: %s" % droppedframes)
                break

    def _setMaxTransferRate(self):
        pass

    def _gain_selector(self,selector = None):
        """
        Chose the desired Gain Selector.


        Parameters
        ----------
        selector : str
            Desired Selector Gain as a String. None to read the current Gain Selector.
            The following Selctor are avaialbe.
            0 = {str} 'AnaloglAll'
            1 = {str} 'DigitalGr'
            2 = {str} 'DigitalGb'
            3 = {str} 'DigitalRed'
            4 = {str} 'DigitalBlue'

        Returns
        -------
        microns : int
            The Gain Selector applying the passed value
        """

        if selector is not None:
            self.logger.debug('Setting <Gain Selector> to {selector}'
                              ''.format(selector = selector))
            self.node_map.GainSelector = selector
        return self.node_map.GainSelector.value

    def _getImages(self,num_images_to_acquire = 16, *args, **kwargs):
        """
        Get an image from the camera device

        *args and **kwargs are ignored parameters!


        Returns
        -------
        img : np.ndarray
            Current camera image
        """

        images_list = []

        self.logger.debug('Creating Buffer and starting acquisition')
        self.device.start_image_acquisition()
        # self.device.
        # self.device.num_filled_buffers_to_hold = num_images_to_acquire


        # Read the needed Information from the buffer
        # print(buffer)
        # _1d = buffer.payload.components[0].data
        # height = buffer.payload.components[0].height
        # width = buffer.payload.components[0].width
        # imgData = np.reshape(buffer.payload.components[0].data,(height,width))
        while num_images_to_acquire  > 0:

            with self.device.fetch_buffer() as buffer:
                # imgData = np.reshape(buffer.payload.components[0].data,
                #                      (buffer.payload.components[0].height,buffer.payload.components[0].width)).copy()

                imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                     dtype=np.uint8,
                                     shape=((buffer.payload.components[0].height, buffer.payload.components[0].width))).copy()
                images_list.append(imgData)
            num_images_to_acquire -=1

        return images_list

    def _frame_callback(self):
        with self.device.fetch_buffer() as buffer:
            # imgData = np.reshape(buffer.payload.components[0].data,
            #                      (buffer.payload.components[0].height,buffer.payload.components[0].width)).copy()

            imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                 dtype=np.uint8,
                                 shape=(
                                 (buffer.payload.components[0].height, buffer.payload.components[0].width))).copy()
            self.img_list.append(imgData)

    def listDevices(self):
        """
        List available GenTlproducer cameras

        Returns
        -------
        cams : list
            list of available GenTlproducer devices
        """

        return Controller.listDevices()

    def openDevice(self):
        """
        Opens a camera device with the stored self.device object
        """

        try:
            self.logger.debug('Opening camera device')
            self._harverster.add_cti_file(CTI_FILE)
            self._harverster.update_device_info_list()
            self.device = self._harverster.create_image_acquirer(model=self.device_handle)

        except Exception as e:
            self.logger.exception('Failed to open the camera device: '
                                  '{e}'.format(e=e))

    def closeDevice(self):
        """
        Closes camera device
        """

        if self.device is not None:
            self.device.destroy()

    def isOpen(self):

        """
        Check if the device for this instance is currently open and ready to
        communicate

        Returns
        -------
        bool
            True if the camera connection is open, False if it is not
        """
        # Assuming that if there is a device given in self.device, device is opened.

        if self.device is not None:
            return True
        else:
            return False

    def isClosed(self):

        """
        Check if the device for this instance is currently closed

        Returns
        -------
        bool
            True if the camera connection is closed, False if it is not
        """
        # Assuming that if there is a device given in self.device, device is opened.

        if self.device is  None:
            return True
        else:
            return False

    def getImage(self, *args, **kwargs):
        """
        Get an image from the camera device

        *args and **kwargs are ignored parameters!


        Returns
        -------
        img : np.ndarray
            Current camera image
        """
        self.logger.debug('Creating Buffer and starting acquisition')
        self.device.start_image_acquisition()

        with self.device.fetch_buffer() as buffer:
            # imgData = np.reshape(buffer.payload.components[0].data,
            #                      (buffer.payload.components[0].height,buffer.payload.components[0].width)).copy()

            imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                 dtype=np.uint8,
                                 shape=((buffer.payload.components[0].height, buffer.payload.components[0].width))).copy()



        self.device.stop_image_acquisition()

        self.logger.debug('Image acquisition finished')

        return imgData.copy()

    def prepareRecording(self, num):
        """ Sets the camera to MultiFrame mode and prepares frames. Use with
        "record()"-function.

        Parameters
        ----------
        num : int
            number of frames to be captured during acquisition
        """
        #TODo if hatrvers is on you have to fuind


        # self.device.num_filled_buffers_to_hold = num
        self.device.start_image_acquisition()

        self._expected_triggered_images = num

    def record(self):
        """ Blocking image acquisition, ends acquisition when num frames are
        captured, where num is set by "prepareRecording(num)". Only use with
        "prepareRecording(num)".

        Returns
        -------
        imgData : list
            List of images
        """

        # self.node_map.AcquisitionStart
        self.logger.info('AcquisitionStart')
        while  self._expected_triggered_images > 0:

            with self.device.fetch_buffer() as buffer:
                # imgData = np.reshape(buffer.payload.components[0].data,
                #                      (buffer.payload.components[0].height,buffer.payload.components[0].width)).copy()

                imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                     dtype=np.uint8,
                                     shape=((buffer.payload.components[0].height, buffer.payload.components[0].width))).copy()
                self.img_list.append(imgData)
            self._expected_triggered_images -=1


        # Clean Up
        # self.device.fetch_buffer().queue()
        self.device.stop_image_acquisition()

        self.logger.info('Image acquisition finished')

        return self.img_list

    def grabStart(self, num=None):
        self.num_images = num
        self.device.start_image_acquisition()
        self.img_list = list()
        self.device.on_new_buffer_arrival = self._frame_callback

    def grabStop(self):
        self.device.stop_image_acquisition()

        # while self.num_images != 0:
        #     with self.device.fetch_buffer() as buffer:
        #         imgData = np.ndarray(buffer)
        #         self.img_list.append(imgData)
        return self.img_list

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
            self.logger.debug('Setting <ExposureTime> to {microns}'
                              ''.format(microns=microns))
            self.node_map.ExposureTime.value = microns
        return self.node_map.ExposureTime.value

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
        if resolution is not None:
            self.logger.debug('Setting <Width> to {width}'
                              ''.format(width=resolution[0]))
            self.node_map.Width.value = resolution[0]
            self.logger.debug('Setting <Height> to {height}'
                              ''.format(height=resolution[1]))
            self.node_map.Height.value = resolution[1]
        return self.node_map.Width.value, self.node_map.Height.value

    def setTriggerMode(self,mode=None):
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
        self.logger.debug("Setting trigger mode to: {mode}".format(mode=mode))
        if mode is None:
            if self.node_map.TriggerMode.value == "Off":
                self.setTriggerMode = "off"
            elif self.node_map.TriggerMode.value == "On":
                 self.setTriggerMode = "in"
            return self.setTriggerMode
        elif mode is not None:
            if mode == 'in':
                self.node_map.TriggerMode = 'On'
                # They are 16 possible sources. Look up the Datasheet to see the physical input ranging. Use 'Software' for a software trigger.
                # self.node_map.TriggerSource = 'Software'
                self.node_map.TriggerSelector = 'FrameStart'
                self.node_map.TriggerActivation = "RisingEdge"
                self.triggerModeSetting = 'in'
            elif mode == 'out':
                # TODO: Implement out trigger for AVT cameras
                self.triggerModeSetting = 'out'
                raise NotImplementedError('Sending triggers is not'
                                          'implemented yet!')
            elif mode == 'off':
                self.node_map.TriggerMode = 'Off'
                self.node_map.TriggerSource = 'NotConnected'
                self.node_map.TriggerSelector = 'FrameStart'

                self.triggerModeSetting = 'off'
            else:
                raise ValueError('Unexpected value in setTriggerMode. '
                                 'Expected "in", "out", or "off". Got {mode}'
                                 ''.format(mode=mode))
            return self.triggerModeSetting
        else:
            raise TypeError('Trigger Mode should be None, "in", "out", or '
                            '"off". Got {mode}'.format(mode=mode))

    def _getImages(self,num_images_to_acquire = 16, *args, **kwargs):
        """
        Get an image from the camera device

        *args and **kwargs are ignored parameters!


        Returns
        -------
        img : np.ndarray
            Current camera image
        """

        images_list = []

        self.logger.debug('Creating Buffer and starting acquisition')
        self.device.start_image_acquisition()
        # self.device.
        # self.device.num_filled_buffers_to_hold = num_images_to_acquire


        # Read the needed Information from the buffer
        # print(buffer)
        # _1d = buffer.payload.components[0].data
        # height = buffer.payload.components[0].height
        # width = buffer.payload.components[0].width
        # imgData = np.reshape(buffer.payload.components[0].data,(height,width))
        while num_images_to_acquire  > 0:

            with self.device.fetch_buffer() as buffer:
                # imgData = np.reshape(buffer.payload.components[0].data,
                #                      (buffer.payload.components[0].height,buffer.payload.components[0].width)).copy()

                imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                     dtype=np.uint8,
                                     shape=((buffer.payload.components[0].height, buffer.payload.components[0].width))).copy()
                images_list.append(imgData)
            num_images_to_acquire -=1

        return images_list

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
        if gain is not None:
            self.logger.debug('Setting <GainRaw> to {gain}'
                              ''.format(gain=gain))
            self.node_map.GainRaw.value = gain


        return self.node_map.GainRaw.value



        # cv2.cvtColor(imgData,cv2.COLOR_BAYER_RG2RG)

    def setFormat(self, fmt=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        fmt : str
            String describing the desired image format (e.g. "BayerRG8"), or None
            to read the current image format. Check the following for the avaiable Formats.

            0 = {str} 'BayerRG8'
            1 = {str} 'BayerRG10'
            2 = {str} 'BayerRG10Packed'
            3 = {str} 'BayerRG12'
            4 = {str} 'BayerRG12Packed'
            5 = {str} 'RGB8Packed'
            6 = {str} 'RGB10V1Packed'
            7 = {str} 'RGB10V2Packed'

        !! To convert the Formats in Opencv, see here(https://www.baumer.com/de/de/service-support/know-how/technische-anwendungshinweise-industriekameras/baumer-gapi-und-opencv/a/baumer-gapi-and-opencv)

        Returns
        -------
        fmt : str
            The image format after applying the passed value
        """

        if fmt is not None:
            self.logger.debug('Setting <PixelFormat> to {fmt}'
                              ''.format(fmt=fmt))
            self.node_map.PixelFormat.value = fmt
        return self.node_map.PixelFormat.value

    def setTriggerMode(self, mode=None):


        pass




if __name__ == '__main__':
    import time

    logging.basicConfig(level=logging.DEBUG)
    # print(Controller().listDevices())
    cam_device = Camera("AD-130GE_#1")
    # cam_device.setExposureMicrons()
    # print(cam_device.setGain())
    # cam_device.openDevice()

    # Testing Exposure
    # exposures = [120, 1000, 5000, 10000]
    # img_intensity = []
    # for exposure in exposures:
    #     try:
    #         cam_device.setExposureMicrons(exposure)
    #         img = cam_device.getImage()
    #         # print(img)
    #         # img_intensity.append(np.mean(np.max(img),np.min(img)))
    #         # assert cam_device.setExposureMicrons() == exposure
    #
    #     except OutOfRangeException:
    #         assert exposure < 10 or exposure > 32000


    # Resoultion
    # print(cam_device.setResolution(()))

    # Format
    # print(cam_device.setFormat("BayerRG8"))

    # cam_device._liveView()
    # cam_device.grabStart()
    # time.sleep(1)
    # img_list = cam_device.grabStop()
    #
    # print(len(img_list))
    # for img in img_list:
    #     cv2.imshow('bla', img)
    #     cv2.waitKey(0)
    # print("Done")

    # cam_device._liveView()
    # images = cam_device.getImages(16)
    # images = cam_device.getImages(200)
    # cam_device._gain_selector("DigitalGb")
    # print(cam_device._gain_selector())
    #
    # for img in images:
    #     cv2.imshow("%i number ",img)
    #     cv2.waitKey()
    # cv2.destroyAllWindows()

    # img = Camera.getImage(cam_device)
    # for i in range (100):
    #     img = Camera.getImage(cam_device)
    #     cv2.imshow("testi", img)
    #     cv2.waitKey()
    # cv2.destroyAllWindows()

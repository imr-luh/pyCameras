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
# TODO: Find a way to automatically find the CTI PATH

class Controller(ControllerTemplate):
    """
    JAI Camera Controller implementation based on Harvester(https://github.com/genicam/harvesters)
    """

    def __init__(self):
        """
        Camera controller for JAI camera devices. This implementation uses
        Harvester as backend.
        """
        # Use the same Harvester core if it is already created
        if not Camera.harvester_instance:
            Camera.harvester_instance = Harvester()
        self._harverster = Camera.harvester_instance

        super(Controller, self).__init__()
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.logger.debug('Starting JAI Camera Controller')

        try:
            self._harverster.add_cti_file(CTI_FILE)
        except LoadLibraryException:
            raise LoadLibraryException("Installation path differs from default value. Change constant CTI_FILE to correct file path.")

    def updateDeviceHandles(self):
        """
        Refresh the list of available devices
        """
        self.logger.debug("Start updating device handles")

        if not self._harverster.has_revised_device_info_list:
            self._harverster.add_cti_file(CTI_FILE)
            self._harverster.update_device_info_list()

        self.device_handles = self._harverster.device_info_list

        self.logger.debug("Finished to update device handles and initialize it")

    def getDevice(self, device_handle):
        """
        Return the corresponding camera object for given device handle

        Parameters
        ----------
        device_handle : Harvester gives multiple unique strings related to a Camera. Here the model name is used.


        Returns
        -------
        cam : Camera object
        """

        self.logger.debug('Try to open device {device_handle}'
                          ''.format(device_handle=device_handle))

        for i, device_info in enumerate(self.device_handles):

            if device_info.model == device_handle:
                self.logger.debug("Found the Model name from index {ind} of device list with model name {name}".format(ind=i, name=device_info.model))
                try:
                    return Camera(device_handle)
                except Exception as e:
                        self.logger.exception('Failed to open the camera device: {e}'
                                          .format(e=e))
                        raise
            else:
                self.logger.info("Was not able to open camera with given device handle!!\n' \
                   'Handle must be the correct Model Name >")

    def closeController(self):
        """
        Close the camera controller and do the necessary cleanup
        """
        self._harverster.reset()

    def __repr__(self):
        return "<JAI Camera Controller>"


class Camera(CameraTemplate):
    """
    JAI Camera implementation based on Harvester(https://github.com/genicam/harvesters)
    """

    # Make harvester instance a singleton object
    harvester_instance = None

    def __init__(self, device_handle):

        if Camera.harvester_instance == None:
            Camera.harvester_instance = Harvester()
        self._harverster = Camera.harvester_instance

        self.device_handle = device_handle

        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)

        self.openDevice()
        self.img_list = list()

        self.node_map.TriggerSource.value = self.node_map.TriggerSource.symbolics[0]

        # Not needed, Just in case a previous Resolution was wrongly set.
        self.node_map.Width.value = self.node_map.WidthMax.value
        self.node_map.Height.value = self.node_map.HeightMax.value

        self.img_list = list()

        # Register JAI Camera specific functions.
        # Function to set maximum transfer rate depending has to be yet implemented
        self.registerFeature("gainselector",self._gain_selector)

    def __del__(self):
        self.closeDevice()
        self._harverster.reset()

    def _liveView(self):
        """
        Live image stream an visualization through OpenCV window

        Leave _liveView by pressing "q"
        """
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
                                         dtype=np.uint8,
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
        """
        Sets the transfer rate.
        If passed None, will return actual rate set.

        """

        # TODO Implement setMaxTransferRate
        # self.node_map.PayloadSize.value = self.Transferrate
        # self.logger.debug("Not implendted yet"
        #                   )

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

    def _frame_callback(self):
        """
        Callback function to fill frames with data

        Parameters
        -------
        frame : frame object
            frame created by device.getFrame()
        """

        with self.device.fetch_buffer() as buffer:

            imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                 dtype=np.uint8,
                                 shape=(
                                 (buffer.payload.components[0].height, buffer.payload.components[0].width))).copy()
            self.img_list.append(imgData)

    def listDevices(self):
        """
        List available camera devices

        !! It will give all available devices which uses Genicam. This Class is only for JAI Cameras.
        Returns
        -------
        cams : list
            list of available Camera devices
        """

        return Controller.listDevices()

    def openDevice(self):
        """
        Opens a camera device.
        """
        try:
            self.logger.debug('Opening camera device')
            if not self._harverster.has_revised_device_info_list:
                self._harverster.add_cti_file(CTI_FILE)
                self._harverster.update_device_info_list()

            self.device_list = self._harverster.device_info_list

            # self.device = self._harverster.create_image_acquirer(model=self.device_handle)
            # self.node_map = self.device.device.node_map
            for i, device_info in enumerate(self.device_list):
                if device_info.model == self.device_handle:
                    self.logger.debug("Creating acquirerer from index {ind} of device list with model name {name}".format(ind=i, name=device_info.model))
                    self.device = self._harverster.create_image_acquirer(i)
                    self.node_map = self.device.device.node_map

        except Exception as e:
            self.logger.exception('Failed to open the camera device: '
                                  '{e}'.format(e=e))
            raise

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
        """
        Start grabbing images in a non-blocking
        way and store those images in
        an internal variable

        See Also
        --------
        self.grabStop()
        """
        self.num_images = num
        self.device.start_image_acquisition()
        self.img_list = list()
        self.device.on_new_buffer_arrival = self._frame_callback

    def grabStop(self):
        """
        Stop grabbing images and return the images that have been recorded

        See Also
        --------
        self.grabStart()
        """
        self.device.stop_image_acquisition()
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
            self.node_map.ExposureAbs.value = microns
            self.logger.info("Setted microns fluctuate")
        return self.node_map.ExposureAbs.value

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
                # TODO Implement the TriggerSource.
                # They are 16 possible sources. Look up the Datasheet to see the physical input ranging.
                self.node_map.TriggerSource = self._setTriggerSource()
                self.node_map.TriggerSelector = 'FrameStart'
                self.node_map.TriggerActivation = "RisingEdge"
                self.triggerModeSetting = 'in'
            elif mode == 'out':
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
        For 'BayerRG8' use dataformat uint8 for the rest use uint16 as dataformat.
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

    def _setTriggerSource(self, source_name=None, source_list_display=None):
        """
        Set the desired Trigger source or read the
        current value by passing

        Parameters
        ----------
        source_name : str
            Desired exposure time in microseconds that should be set, or None
            to read the current exposure time
        source_list : Boolen
            If true give out the source list

        Returns
        -------
         : str
            The exposure time in microseconds after applying the passed value

        """
        # TODO: Find the corresponding triggerpins for each source. Documentation is a little confusing.
        trigger_sources = self.node_map.TriggerSource.symbolics
        if source_list_display:
            self.logger.info("(list)".format(list=trigger_sources))
        if not source_name:
            return self.node_map.TriggerSource.value

        if not source_name in trigger_sources:
            raise ValueError("Given source name is not a valid source for input triggering")

        self.node_map.TriggerSource.value = source_name

        return self.node_map.TriggerSource.value

    def __repr__(self):
        return repr(self.device)


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    # ############################## Controller test
    # # # Base
    # controller = Controller()
    # # list devices
    # test1 = controller.listDevices()
    # print("test 1",test1)
    # # get device
    # test2 = controller.getDevice('AD-130GE_#0')
    # print("test2",test2)
    # # # print(Controller.listDevices)
    # # # print(Controller.getDevice("AD-130GE_#0"))
    # #




    ####### Camera Test
    cam_device = Camera("AD-130GE_#1")
    cam_device._setTriggerSource()
    cam_device.setTriggerMode("off")
    # cam_device.getImage()
    cam_device._liveView()
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


    #### Resoultion
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


    #### Get image
    # img = cam_device.getImage()
    # cv2.imshow("df", img)
    # cv2.waitKey(0)

    ## GEt images
    # images = cam_device.getImages(200)
    # print(len(images))
    #
    # for img in images:
    #     cv2.imshow("%i number ",img)
    #     cv2.waitKey()
    # cv2.destroyAllWindows()
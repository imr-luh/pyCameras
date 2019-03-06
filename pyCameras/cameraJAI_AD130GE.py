from pyCameras import cameraTemplate
import cv2
import numpy as np
from harvesters.core import Harvester, LoadLibraryException
import logging

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate
import harvesters_gui

# Adjust me, if installation path differs from README
CTI_FILE = "/opt/mvIMPACT_Acquire/lib/x86_64/mvGenTLProducer.cti"

LOGGING_LEVEL = None

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
    # # CCa
    # "Controller from harvester"
    # h=Harvester()
    #
    # ## needed for finding out which camere is there
    # ## get the location of the cti file via "grep"
    # h.add_cti_file("/opt/mvIMPACT_Acquire/lib/x86_64/mvGenTLProducer.cti")
    # h.update_device_info_list()
    # h.device_info_list
    # print(h.device_info_list)
    # iaa = h.device_info_list
    #
    #
    # ia = h.create_image_acquirer(model='AD-130GE_#0')
    # ## to specifiy which one it is, take the serial number
    # dev = ia.device
    # dev.node_map.ExposureTime.value = 31000
    # # dev.node_map.Width.value = 100
    # ia.start_image_acquisition()
    # print("dfdsf")
    # print(dev.node_map.Width.value)
    #
    # buffer = ia.fetch_buffer()
    # _1d = buffer.payload.components[0].data
    #
    # height = buffer.payload.components[0].height
    # width = buffer.payload.components[0].width
    # print(buffer)
    # img = np.reshape(buffer.payload.components[0].data, (height, width))
    # cv2.imshow('blubb', img)
    # cv2.waitKey()
    #
    # ia.stop_image_acquisition()
    # ia.destroy()d
    # h.reset()
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

        self._harverster.add_cti_file(CTI_FILE)
        self._harverster.update_device_info_list()



        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self.device = self._harverster.create_image_acquirer(serial_number=device_handle)
        self.node_map = self.device.device.node_map

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
            self.node_map.ExposureTime.value = selector
        return self.node_map.ExposureTime.value

    def closeDevice(self):
        self.device.destroy()
    def _sensormode
    def _init_buffer(self):
        pass
    def que_buffer(self):
        self.device.fetch_buffer().queue()
        pass

    def listDevices(self):
        """
        List available GenIcam cameras

        Returns
        -------
        cams : list
            list of available GenIcam devices
        """

        return Controller.listDevices()

    def openDevice(self):
        """
        Opens a camera device with the stored self.device object
        """

        try:
            self.logger.debug('Opening camera device')
        except Exception as e:
            self.logger.exception('Failed to open the camera device: '
                                  '{e}'.format(e=e))

    def setExposureMicrons(self, microns=None):
        """
        Set the exposure time to the given value in microseconds or read the
        current value by passing None

        Parameters
        ----------
        microns : int
            Desired exposure time in microseconds that should be set, or None
            to read the current exposure time
            max

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

    def getImages(self, *args, **kwargs):
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
        # Create a buffer
        # buffer = self.device.fetch_buffer()

        # Read the needed Information from the buffer
        # print(buffer)
        # _1d = buffer.payload.components[0].data
        # height = buffer.payload.components[0].height
        # width = buffer.payload.components[0].width
        # imgData = np.reshape(buffer.payload.components[0].data,(height,width))
        with self.device.fetch_buffer() as buffer:
            # imgData = np.reshape(buffer.payload.components[0].data,
            #                      (buffer.payload.components[0].height,buffer.payload.components[0].width)).copy()

            imgData = np.ndarray(buffer=buffer.payload.components[0].data,
                                 dtype=np.uint8,
                                 shape=((buffer.payload.components[0].height, buffer.payload.components[0].width))).copy()


        # Clean Up
        # self.device.fetch_buffer().queue()
        self.device.stop_image_acquisition()

        self.logger.debug('Image acquisition finished')
        return imgData.copy()
        self.device.

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
        return self.device.GainRaw.GetValue()



        cv2.cvtColor(imgData,cv2.COLOR_BAYER_RG2RG)





if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    # print("liste von devices sind ",Controller().listDevices())
    cam_device = Camera("QV130962")
    cam_device.setExposureMicrons(30000)

    cam_device._liveView()
    # img = Camera.getImage(cam_device)
    # for i in range (100):
    #     img = Camera.getImage(cam_device)
    #     cv2.imshow("testi", img)
    #     cv2.waitKey()
    # cv2.destroyAllWindows()

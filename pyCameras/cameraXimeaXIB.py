#!/usr/bin/env python
"""
Implementation of the template for XIMEA XiB cameras
"""
__author__ = "Jochen Schlobohm"
__credits__ = ["Ruediger Beermann", "Niklas Kroeger"]
__maintainer__ = "not assigned"
__email__ = ""
__status__ = "Development"

import abc
import logging
import numpy as np

from pyXimea import xiapi, xidefs
from pyCameras.cameraTemplate import CameraTemplate

# class CameraControllerTemplate(object):
#     """
#     Template class for spectrometer controllers to inherit from if they are
#     necessary to use the camera. The controller for camera devices should
#     be opened and closed from the Camera.openController and
#     Camera.closeController static methods. If the controller is only used to
#     detect available camera devices, that logic could also be easily
#     implemented in Camera.listDevices().
#     """
# 
#     def __init__(self):
#         self.logger = logging.getLogger(__name__)
#         self.device_handles = []
# 
#     def listDevices(self):
#         """
#         Returns a list of available devices. One of these entries should be
#         used as parameter for self.getDevice to open the corresponding device
# 
#         Returns
#         -------
#         device_handles : list
#             List of available capture devices
#         """
#         self.updateDeviceHandles()
#         return self.device_handles
# 
#     def updateDeviceHandles(self):
#         """
#         Update the list of available device handles
#         """
#         raise NotImplementedError
# 
#     def getDevice(self, device_handle):
#         raise NotImplementedError
# 
#     def closeController(self):
#         raise NotImplementedError
# 
#     def __del__(self):
#         self.logger.debug('Deleting cameracontroller {self}'
#                           ''.format(self=self))
#         self.closeController()
# 
#     def __repr__(self):
#         return "<CameraController Template: OVERLOAD THIS FUNCTION>"


class CameraXimeaXIB(CameraTemplate):
    """
    Class to access XIMEA XiB cameras.
    The device_handle is the serial number STRING
    """
    def __init__(self, device_handle):
        """
        Template class for camera objects. This is only meant to define the
        interface. You probably want a real camera implementation that is
        inherited from this class.

        Parameters
        ----------
        device_handle : object
            Some handle that identifies the camera and is used to open the
            device connection
        """
        super(CameraXimeaXIB, self).__init__(device_handle)

        self.logger = logging.getLogger(__name__)
        self.device_handle = device_handle  # Use this to identify and open the
                                            # device
        self.device = xiapi.Camera()  # Use this variable to store the device itself
        self.triggerMode = "off"

    @staticmethod
    def listDevices():
        """
        List all available camera devices correspponding to the class type
        """
        cam = xiapi.Camera()
        numCameras = cam.get_number_devices()
        result = []
        for i in range(numCameras):
            #print i
            cam = xiapi.Camera(i)
            
            cam.open_device()
            result.append(cam.get_device_sn().decode("utf-8"))
            cam.close_device()
        return result

    def openDevice(self):
        """
        Open the device by using self.device_handle and store the device in
        self.device
        """
        self.device.open_device_by_SN(self.device_handle)

    def closeDevice(self):
        """
        Close the connection to the device and reset self.device to None.
        """
        self.device.close_device()

    def isOpen(self):
        """
        Check if the device for this instance is currently open and ready to
        communicate

        Returns
        -------
        bool
            True if the camera connection is open, False if it is not
        """
        return self.device.CAM_OPEN

    def getImage(self, *args, **kwargs):
        """
        Return a numpy array containing an image

        *args and **kwargs are optional parameters that may be ignored!
        """
        try:
            self.device.start_acquisition()
            img = xiapi.Image()
            self.device.get_image(img)
            nImg = self.getNumpyData(img)
        except:
            raise
        finally:
            self.device.stop_acquisition()
        return nImg

    def getNumpyData(self, img):
        bpp = img.get_bytes_per_pixel()
        if bpp in [1, 3, 4]: 
            npType = np.uint8                
        elif bpp in [2, 6, 8]:
            npType = np.uint16
        raw = img.get_image_data_raw()
        #print(len(raw), img.width, img.height, img.get_bytes_per_pixel(), npType)
        nImg = np.squeeze(np.fromstring(raw, dtype = npType).reshape((img.height, img.width, -1)))
        return nImg 

    def getImages(self, num=None):
        """
        Return a iterable of numpy arrays corresponding to images that were
        recorded previously. If there are no buffered images the iterable may
        be empty.

        Parameters
        ----------
        num : int
            number of images to return. If None return all images currently in
            buffer
        """
 
        result = [xiapi.Image() for i in range(num)]
        resultNp = []
        self.device.start_acquisition()
        t = time()

        try:
            for img in result:
                self.device.get_image(img)
                resultNp.append(self.getNumpyData(img))
        finally:
            self.device.stop_acquisition()

    def grabStart(self):
        """
        Start grabbing images
        """
        self.device.start_acquisition()

    def grabStop(self):
        """
        Stop grabbing images
        """
        self.device.stop_acquisition()

    def listFeatures(self):
        """
        Helper function to return the properties dict
        """
        return xidefs.ASSOC_ENUM.keys()

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

        self.registerFeature('Format', self.setPixelFormat)

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
        self.device.set_exposure(microns)

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
        raise NotImplementedError

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
        self.device.set_gain_selector("XI_GAIN_SELECTOR_ANALOG_ALL");  
        if gain is None:
            return self.device.get_gain()
        self.device.set_gain(gain)

    def setPixelFormat(self, format=None):
        """
        Set the image format to the passed setting or read the current format
        by passing None

        Parameters
        ----------
        format : str
            String describing the desired image format (e.g. "mono8"), or None
            to read the current image format

        Returns
        -------
        format : str
            The image format after applying the passed value
        """
        raise NotImplementedError

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
        if mode is None:
            return self.triggerMode
        elif mode == "off":
            
            self.device.set_gpo_selector("XI_GPO_PORT1")
            self.device.set_gpo_mode("XI_GPO_OFF")
            self.device.set_gpi_selector("XI_GPI_PORT1")
            self.device.set_gpi_mode("XI_GPI_OFF")
            self.device.set_trigger_source("XI_TRG_OFF")
        elif mode == "in":
            self.device.set_gpo_selector("XI_GPO_PORT1")
            self.device.set_gpo_mode("XI_GPO_OFF")
            self.device.set_gpi_selector("XI_GPI_PORT1")
            self.device.set_gpi_mode("XI_GPI_TRIGGER")
            self.device.set_trigger_source("XI_TRG_EDGE_RISING")
        elif mode == "out":
            self.device.set_gpo_selector("XI_GPO_PORT1")
            self.device.set_gpo_mode("XI_GPO_EXPOSURE_ACTIVE")
            self.device.set_gpi_selector("XI_GPI_PORT1")
            self.device.set_gpi_mode("XI_GPI_OFF")
            self.device.set_trigger_source("XI_TRG_OFF")
        self.triggerMode = mode

    def __del__(self):
        if self.device is not None:
            self.closeDevice()


if __name__ == "__main__":
    from time import time
    t = time()
    print ("start", time()-t)
    
    devices =  CameraXimeaXIB.listDevices()
    print ("listed", time()-t)
    print ("Devices: ", devices)
    cam = CameraXimeaXIB(devices[0])
    print ("constructed", time()-t)
    cam.openDevice()
    print (cam.setTriggerMode())
    cam.setTriggerMode("in")
    cam.setTriggerMode("out")
    cam.setTriggerMode("off")
    print ("opened", time()-t)
    print( "Gain: ", cam.setGain())
    cam.setGain(-9)
    print( "Gain: ", cam.setGain())
    cam.setExposureMicrons(10000)
    print ("set", time()-t)
    from matplotlib import pyplot as plt
    img = cam.getImage()
    print ("got image", time()-t)
    
    print ("Image 1", img, img.shape, img.dtype)
    plt.imshow(img, cmap="gray")
    plt.show()
    cam.setGain(12)
    img = cam.getImage()
    print ("Image 2", img, img.shape, img.dtype)
    plt.imshow(img, cmap="gray")
    plt.show()
    
    def testMultipleImages(num):
        print ("getting %d images ..."%(num))
        from time import time
        t = time()
        cam.getImages(num)
        res = time()-t
        print ("done in %f seconds with %f fps"%(res, float(num)/res))
        return res
    testMultipleImages(10)
    testMultipleImages(20)
    testMultipleImages(200)
    #testMultipleImages(90)
    
    

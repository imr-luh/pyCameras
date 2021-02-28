import unittest
import pytest
from imrpy.utils.imports import dynamic_import_class
from os import path
import yaml
import logging
from pyCameras.cameraDummy import Camera

LOGGING_LEVEL = logging.DEBUG

class CameraTest(unittest.TestCase):
    """ Camera Test for all cameras implemented in pyCameras."""

    fname = 'cam_config.yaml'
    file_ext = path.splitext(fname)[1]
    if file_ext == '.yaml' and path.exists(fname):
        with open(fname) as f:
            camDict = yaml.load(f, Loader=yaml.SafeLoader)

    logging.basicConfig()
    logger = logging.getLogger(__name__)
    if LOGGING_LEVEL is not None:
        logger.setLevel(LOGGING_LEVEL)

    logger.info("Starting Camera Test")

    # camClass = dynamic_import_class(camDict['class'])
    # cam = camClass(camDict['id'])
    cam = Camera()

    expectedImages = 0  # used by self.record
    resolution = (500, 500)

    # Variables to mimic behaviour of real cameras
    _trigger = None
    _format = None
    _gain = None
    _exposure = None
    _isOpen = False
    _pixelFormat = None

    # self.cam = self.test_connection()
    # self.validFeatures = self.test_features()

    def test_setResolution(self):
        for resolution in self.camDict['test_settings']['resolution']:
            self.logger.info(f"Testing Camera Resolution {resolution}")
            res = self.cam.setResolution(resolution=resolution)
            self.resolution = res
            self.assertEqual(resolution, res)

    def test_setPixelFormat(self):
        for pixFormat in self.camDict['test_settings']['pixelFormat']:
            fmt = self.cam.setPixelFormat(pixFormat)
            self.assertEqual(pixFormat, fmt)

    def test_listDevices(self):
        devices = self.cam.listDevices()
        self.logger.info(f"Testing list Devices, devices:{devices}")
        self.assertEqual(isinstance(devices, list), True)

    def test_listFeatures(self):
        features = self.cam.listFeatures()
        self.assertEqual(isinstance(features, list), True)

    def test_registerFeatures(self):
        self.cam.registerSharedFeatures()
        self.assertEqual(isinstance(self.cam.features, dict), True)

    def test_openDevice(self):
        self.cam.openDevice()
        self.assertEqual(self.cam._isOpen, True)

    def test_closeDevice(self):
        self.cam.closeDevice()
        self.assertEqual(self.cam._isOpen, False)

    def test_setTriggerMode(self):
        for triggerMode in self.camDict['test_settings']['triggerMode']:
            currentTrigger = self.cam.setTriggerMode(triggerMode)
            self.logger.info(f'testing triggerMode {triggerMode}')
            self.assertEqual(triggerMode, currentTrigger)

    def test_prepareRecording(self):
        self.cam.prepareRecording(self.expectedImages)
        self.assertEqual(self.expectedImages, self.cam._expectedImages)

    def test_record(self):
        imgs = self.cam.record()
        self.assertEqual(self.expectedImages, len(imgs))

    def test_getImages(self):
        for i in range(10):
            self.test_getImage()

    def test_getImage(self):
        img = self.cam.getImage()
        if img.size == 3:
            w, h, depth = img.size
        elif img.size == 2:
            w, h = img.size
        else:
            # raise exception missing
            return
        self.assertTrue(w == img[0])
        self.assertTrue(h == img[1])
    #
    def test_setExposureMicrons(self):
        for expo in self.camDict['test_settings']['exposure']:
            self.logger.info(f"Testing Camera Exposure {expo}")
            res = self.cam.setExposureMicrons(expo)
            self.assertEqual(expo, res)


if __name__ == '__main__':
    unittest.main()

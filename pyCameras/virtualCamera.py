#!/usr/bin/env python3
__author__ = 'Niklas Kroeger'
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import logging
import os
import sys
import errno
from natsort import natsorted
import cv2
import numpy as np

from pyCameras.cameraTemplate import CameraTemplate

LOGGING_LEVEL = None


class Camera(CameraTemplate):
    """
    Dummy camera for use with code that expects a pyCameras compatible camera
    implementation. This camera can be used during development to simulate a
    camera that will be used later.
    """
    def __init__(self, device_handle: object):
        """
        Dummy camera to be used instead of an actual hardware camera

        If a image_folder parameter is given the dummy camera loads all image
        files inside that directory and returns one of them for every call to
        getImage(). Images are returned in sorted order and in an infinite
        loop.

        Parameters
        ----------
        imageDir : str
            Path to a folder containing sample images that should be returned
            from the dummy camera. Images are returned in a infinite loop and
            in sorted order.

        cvLoadFlags : cv2 imread flag
            Additional imread flags. Defaults to cv2.IMREAD_GRAYSCALE
        """
        super(Camera, self).__init__(device_handle)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self._images = []

        self._expectedImages = 0     # used by self.record
        self._curIndex = 0
        self._imageDirs = list()
        self._resolution = (500, 500)
        self.curDir = None

        # Variables to mimic behaviour of real cameras
        self._trigger = None
        self._format = None
        self._gain = None
        self._exposure = None
        self._isOpen = True
        self.multipleMeasurements = False

    def registerFeatures(self) -> None:
        """
        Registration of shared features that should be the same for all camera
        implementations. E.g. ExposureMicrons, Resolution, Gain, Format and
        TriggerMode
        """
        self.logger.debug('Registering camera features')
        self.registerFeature('multiple_measurements', self.setMultipleMeasurements)
        self.registerFeature('path', self.setPath)

    def setMultipleMeasurements(self, multipleMeasurements=None) -> bool:
        if multipleMeasurements is None:
            return self.multipleMeasurements
        else:
            self.multipleMeasurements = multipleMeasurements
            return self.multipleMeasurements

    def setPath(self, pathList=None):
        if pathList is None:
            return self._imageDirs
        else:
            self._imageDirs = pathList
            return self._imageDirs

    @staticmethod
    def listDevices():
        return [Camera]

    def openDevice(self):
        self._isOpen = True

    def closeDevice(self):
        self._isOpen = False

    def isOpen(self):
        return self._isOpen

    def getImage(self, *args, **kwargs):
        try:
            cvLoadFlags = cv2.IMREAD_GRAYSCALE
            self._loadImages(self._imageDirs[self._curIndex], cvLoadFlags=cvLoadFlags)
            img = self._images[self._curIndex] #todo: edit this case
        except Exception as e:
            raise NotImplementedError(f"hier nichts live {e}")
        # self._curIndex = (self._curIndex+1) % len(self._images)
        return img

    def prepareRecording(self, num):
        self._expectedImages = num
        if self.multipleMeasurements:
            self.curDir = self._imageDirs[self._curIndex] if os.path.exists(self._imageDirs[self._curIndex]) else None
        else:
            self.curDir = self._imageDirs[0] if os.path.exists(self._imageDirs[0]) else None

    def record(self):
        try:
            cvLoadFlags = cv2.IMREAD_GRAYSCALE
            self._loadImages(self.curDir, cvLoadFlags=cvLoadFlags)
            imgs = self._images
            self._curIndex += 1
            if len(imgs) != self._expectedImages:
                raise ValueError(f"expected {self._expectedImages} but got {len(imgs)}")
        except IndexError:
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), self.curDir)
        return imgs

    def setExposureMicrons(self, microns=None):
        if microns is None:
            return self._exposure
        self._exposure = microns
        return self._exposure

    def setResolution(self, resolution=None):
        if resolution is not None:
            self._resolution = resolution
        return self._resolution

    def setGain(self, gain=None):
        if gain is None:
            return self._gain
        self._gain = gain

        return self._gain

    def setFormat(self, fmt=None):
        if fmt is None:
            return self._format
        self._format = fmt

        return self._format

    def setTriggerMode(self, mode=None):
        if mode is None:
            return self._trigger
        self._trigger = mode

        return self._trigger

    @staticmethod
    def openController():
        return 0

    @staticmethod
    def closeController():
        return 0

    def _loadImages(self, imageDir, cvLoadFlags=cv2.IMREAD_GRAYSCALE):
        """
        Load all image inside the given image_dir as grayscale and store them
        in self._images in sorted order.

        Parameters
        ----------
        imageDir : str
            Path to a directory containing image files that should be loaded

        cvLoadFlags : cv2 imread flag
            Additional imread flags such as cv2.IMREAD_GRAYSCALE
        """
        if isinstance(imageDir, str):
            if imageDir.startswith("~") and 'linux' in sys.platform.lower():
                imageDir = os.path.expanduser(imageDir)
            if os.path.exists(imageDir):
                self.logger.debug('Loading images in {image_dir}'
                                  ''.format(image_dir=imageDir))

                extensions = ['.png',
                              '.jpg',
                              '.jpeg',
                              '.bmp']

                images = natsorted([image for image in os.listdir(imageDir) if
                                 os.path.splitext(image)[
                                     -1].lower() in extensions])
                if len(images) > 0:
                    self.logger.debug('found the following images: {images}'
                                      ''.format(images=images))
                    for image in images:
                        self._images.append(cv2.imread(os.path.join(imageDir, image),
                                                       flags=cvLoadFlags))
                else:
                    self.logger.info(
                        "No valid image path was specified. Using random image instead (Noisy Image).")
                    self._images = list()
                    for i in range(2):
                        noisy_img = np.random.random(self.resolution)
                        noisy_img = cv2.putText(noisy_img,
                                                "No images found in path",
                                                (10, self.resolution[1] // 2),
                                                cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                                (255, 255, 255),
                                                lineType=cv2.LINE_AA,
                                                thickness=2)
                        self._images.append(noisy_img)
            else:
                self.logger.info("No valid image path was specified. Using random image instead (Noisy Image).")
                self._images = list()
                for i in range(2):
                    noisy_img = np.random.random(self.resolution)
                    noisy_img = cv2.putText(noisy_img, "Invalid path",
                                            (10, self.resolution[1] // 2),
                                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                            (255, 255, 255),
                                            lineType=cv2.LINE_AA,
                                            thickness=2)
                    self._images.append(noisy_img)
        else:
            self.logger.info(
                "No valid image path was specified. Using random image instead (Noisy Image).")
            self._images = [np.random.random(self.resolution),
                            np.random.random(self.resolution)]

        # Set resolution to image resolution (all images should have the same resolution)
        self.setResolution(self._images[0].shape[0:2])

    def __repr__(self):
        return '<Dummy Camera: {image_dir}>'.format(image_dir=self._imageDirs)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    devices = Camera.listDevices()
    print("Available Devices: ", devices)

    cam = Camera(devices)
    cam.setPath(['/home/middendorf/Schreibtisch/test_images/'])
    cam.setMultipleMeasurements(False)
    cam.prepareRecording(16)
    imgs = cam.record()

    img = cam.getImage()
    # cv2.imshow("cameraDummy image", img)
    # cv2.waitKey(0)

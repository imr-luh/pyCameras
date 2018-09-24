#!/usr/bin/env python3
__author__ = 'Niklas Kroeger'
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import logging
import os

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
    def __init__(self, imageDir=None, cvLoadFlags=cv2.IMREAD_GRAYSCALE):
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
        super(Camera, self).__init__(imageDir)
        self.logger = logging.getLogger(__name__)
        if LOGGING_LEVEL is not None:
            self.logger.setLevel(LOGGING_LEVEL)
        self._images = []

        self._expectedImages = 0     # used by self.record
        self._curIndex = 0
        self._imageDir = imageDir
        self._resolution = (100, 100)
        if isinstance(self._imageDir, str):
            self._loadImages(self._imageDir, cvLoadFlags=cvLoadFlags)

        # Variables to mimic behaviour of real cameras
        self._trigger = None
        self._format = None
        self._gain = None
        self._exposure = None

    @staticmethod
    def listDevices():
        return [Camera]

    def openDevice(self):
        return 0

    def closeDevice(self):
        return 0

    def getImage(self, *args, **kwargs):
        try:
            img = self._images[self._curIndex]
        except IndexError:
            return np.zeros(self._resolution, dtype=np.uint8)
        self._curIndex = (self._curIndex+1) % len(self._images)
        return img

    def prepareRecording(self, num):
        self._expectedImages = num

    def record(self):
        N = self._expectedImages
        self._expectedImages = 0
        return [self.getImage() for _ in range(N)]

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
        self.logger.debug('Loading images in {image_dir}'
                          ''.format(image_dir=imageDir))

        extensions=['.png',
                    '.jpg',
                    '.jpeg',
                    '.bmp']

        images = sorted([image for image in os.listdir(imageDir) if
                         os.path.splitext(image)[-1].lower() in extensions])
        self.logger.debug('found the following images: {images}'
                          ''.format(images=images))
        for image in images:
            self._images.append(cv2.imread(os.path.join(imageDir, image),
                                           flags=cvLoadFlags))
        # Set resolution to image resolution (all images should have the same resolution)
        self.setResolution(self._images[0].shape[0:2])

    def __repr__(self):
        return '<Dummy Camera: {image_dir}>'.format(image_dir=self._imageDir)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    devices = Camera.listDevices()
    print(devices)
    cam = Camera('/home/kroeger/kroeger/Fotos/Ellipsometer/2017-07-31')
    print([img.sum() for img in cam.getImages(15)])
    print(cam)
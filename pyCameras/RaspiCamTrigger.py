#!/usr/bin/python
import sys
import time

sys.stdout = sys.stderr
from time import time as t
import atexit
import threading
import numpy as np
from io import BytesIO
# import Image
import os
new_lib = 'tmp/pycharm_project_207/pyCameras/pyRaspiCam/_raspiCam'
if not new_lib in os.environ['LD_LIBRARY_PATH']:
    os.environ['LD_LIBRARY_PATH'] += ':'+new_lib
    try:
        os.execv(sys.argv[0], sys.argv)
    except Exception as e:
        sys.exit('EXCEPTION: Failed to Execute under modified environment, '+e)
from pyRaspiCam import raspiCam


# camera = picamera.PiCamera()	#start at first

class Root(object):

    def __init__(self):
        self.img = np.zeros((1944, 2592, 1), dtype=np.uint8)
        self.exposure = 50000

    def start(self):
        self.camera = raspiCam
        raspiCam.initialize(raspiCam.FORMAT_YUV_Y, raspiCam.FLASH_TORCH)
        print("starting")

    def stop(self):
        raspiCam.finalize()
        print("stopping")

    def binning(self, ratio):
        if ratio == 0:
            return self.img
        if ratio != 2:
            raise Exception("ratio %d not supported yet" % ratio)

        t = time.time()

        result = self.img
        result = result / 4
        result = result[::2, ::2] + result[1::2, 0::2] + result[0::2, 1::2] + result[1::2, 1::2]

        print(f"t = {time.time() - t} ")

        return result

    def gray(self, *args, **kwargs):
        print(kwargs)
        self.handleVars(self, *args, **kwargs)
        self.camera.getImage(self.img, self.exposure)

        image = self.img.reshape((self.img.shape[0], self.img.shape[1]))

        if "binning" in kwargs.keys():
            image = self.binning(int(kwargs["binning"]))

        data = BytesIO()

        im = np.frombuffer(data.getvalue(), dtype=np.uint8)
        im.save(data, format='bmp')
        rawdata = data.getvalue()
        return rawdata

    def jpeg(self, *args, **kwargs):

        self.handleVars(self, *args, **kwargs)
        self.camera.getImage(self.img, self.exposure)

        image = self.img

        if "binning" in kwargs.keys():
            image = self.binning(int(kwargs["binning"]))

        data = BytesIO()
        im = np.frombuffer(data.getvalue(), dtype=np.uint8)
        im.save(data, format='jpeg')
        rawdata = data.getvalue()
        if kwargs.has_key('bytes'):
            return rawdata
        return rawdata

    def handleVars(self, *args, **kwargs):
        print(kwargs.keys())
        if "exposure" in kwargs.keys():
            self.exposure = int(kwargs['exposure'])


# daemonize()

root = Root()
root.start()
root.gray()
root.stop()
# cherrypy.engine.start()


import logging
from cython_ov2740 import *
import time
if __name__ == '__main__':
    logger = logging.getLogger(__name__)
    available_devices = Camera.listDevices()
    logger.debug(f"Available Devices {available_devices}")
    cam = Camera(available_devices[1])

    # Code for the image acquisition of x Frames
    cam.setTriggerMode("Out")
    cam.setFramerate(framerate=10)
    expectedImages = 19
    cam.setExposureMicrons(10000)
    start = time.time()

    for i in range(0,1):
        cam.prepareRecording(expectedImages)
        Images = cam.record()

    Images = cam.postProcessImages(Images, colour=True, bit=8, blacklevelcorrection=False)
    end = time.time()

    print(end-start)
    plt.imshow(Images[3])
    plt.show()
    del cam
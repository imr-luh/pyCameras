

import raspiCam
import numpy as np
import time
from scipy.misc import imsave


format = raspiCam.FORMAT_YUV_Y

dims = 1
if format == raspiCam.FORMAT_RGB:
   dims = 3

arr = np.zeros((1944, 2592, dims), dtype = np.uint8)


#raspiCam.initialize(format, raspiCam.FLASH_OFF)
#print "Initialized"

exposure = 1000
#print "getting Image no Flash"
#raspiCam.getImage(arr, 10000)
#print "done"
#raspiCam.finalize()
#print "finalized"
#print "getting Image flash torch"
#raspiCam.initialize(format, raspiCam.FLASH_TORCH)
#print "Initialized"
#raspiCam.getImage(arr, 10000)
#print "done"
#raspiCam.finalize()
#print "finalized"
print "getting Image flash on"
raspiCam.initialize(format, 17, 60, 2)
print "Initialized"
raspiCam.getImage(arr, 10000)
print "done"
time.sleep(3)
for ex in range(10000, 1000000, 100000):
    
    retCode = raspiCam.getImage(arr, ex)
    print "got image with mean: %03.3f and exposure %7d"%(np.mean(arr), ex)



print "trying to get burst of images with exposure %d"%exposure
raspiCam.getImage(arr, exposure)

n = 20
t = time.time()
for i in range(n):
    img = raspiCam.getImage(arr, exposure)
    


t = time.time() -t
print "took %f seconds for %d images"%(t, n)
print float(n) / t , "fps"

img = raspiCam.getImage(arr, 0)
img = raspiCam.getImage(arr, 0)

imsave("test.jpg", arr)
raspiCam.finalize()
time.sleep(3)

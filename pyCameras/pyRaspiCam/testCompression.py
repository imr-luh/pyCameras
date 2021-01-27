

import raspiCam
import numpy as np
import time
import lz4f
from scipy.misc import imsave
from os import path
import StringIO
import Image

arr = np.zeros((1944, 2592), dtype = np.ubyte)


raspiCam.initialize()
print "initialized"

exposure = 10000


retCode = raspiCam.getImage(arr, exposure)
raspiCam.finalize()

print "got image"

for f in ["bmp", "png", "jpeg"]:
    
    t = time.time()
    data = StringIO.StringIO()
    im = Image.fromarray(arr)
    im.save(data, format=f)
    
    print "Saving %s took %f seconds"%(f, time.time()-t)
    #print "fileSize: %d"%(path.getsize(f))
    print "fileSize: %d"%(len(data.getvalue()))
    

t = time.time()
f = 'test.lz4'
data = StringIO.StringIO()
data.write(lz4f.compressFrame(np.getbuffer(arr)))

print "Saving %s took %f seconds"%(f, time.time()-t)
print "fileSize: %d"%(len(data.getvalue()))
    
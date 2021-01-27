

import numpy as np
import urllib2
from matplotlib import pyplot as plt
import threading
from time import time

binning = 2
exposure = 32000
url= "http://130.75.27.3:5900/"
fullurl = url + \
                '%s?bytes&exposure=%i&binning=%d'% \
                                                ("gray", exposure, binning)

w, h = (2592, 1944)
if binning > 0:
    w = w//binning
    h = h//binning
imgNum = 8



numThreads = imgNum

def getImg():
    req = urllib2.Request(fullurl)
    raw = urllib2.urlopen(req, timeout=1e4).read()
    return np.fromstring(raw, dtype = np.uint8).reshape((h,w))

img = getImg()

plt.imshow(img)
plt.show()
startTime = time()
imgs = []
for i in range(imgNum):
    imgs.append(getImg())

print "done in %f seconds. %f fps (single threaded)"%(time()-startTime, numThreads/(time()-startTime))

imgs = []
threadList = []

startTime = time()

for i in range(numThreads):
    t = threading.Thread(target=lambda: imgs.append(getImg()))
    t.start()
    threadList.append(t)
    
for t in threadList:
    t.join()

print "done in %f seconds. %f fps (multithreading)"%(time()-startTime, numThreads/(time()-startTime))

plt.hold(1)
for i in range(numThreads):
    pltnum = 4*100+10+10*(numThreads//4)+i+1
    #print pltnum, 100+(numThreads//5)*100, 10+10*(numThreads/(1+numThreads//5)), i+1, (numThreads/(numThreads//5))
    ax = plt.subplot(pltnum)
    ax.imshow(imgs[i])
    
plt.show()
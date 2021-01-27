#!/usr/bin/python
import sys
import time
sys.stdout = sys.stderr
from time import time as t
import atexit
import cherrypy
import threading
import numpy as np
from cStringIO import StringIO
import Image
import os
import raspiCam
#camera = picamera.PiCamera()	#start at first
import Queue

#imgShape = (1944, 2592)
#imgShape = (972, 1296)
imgShape = (992, 1312)
#imgShape = (1296,972)
imgSize = (imgShape[1], imgShape[0])

if cherrypy.__version__.startswith('3.0') and cherrypy.engine.state == 0:
    cherrypy.engine.start(blocking=False)
    atexit.register(cherrypy.engine.stop)

class FiFoLock(object):
    def __init__(self):
        self.queue = Queue.Queue()
        self.lock = threading.Lock()
        self.busy = False
        
    def acquire(self):
        e = None
        with self.lock:
            if self.busy or not self.queue.empty():
                e = threading.Event()
                self.queue.put(threading.Event())
                e.wait()
            self.busy = True
            

            
        
    def release(self):
        with self.lock:
            if not self.queue.empty():
                self.queue.get().set()
            self.busy = False

class Root(object):
    
    def __init__(self):
        self.img = np.zeros((imgShape[0], imgShape[1], 1), dtype = np.uint8)
        self.exposure = 50000
#        self.camSemaphore = FiFoLock()
        self.camSemaphore = threading.Semaphore()
        self.lastExposure = 50000
        self.flash = 17
    
    def start(self):
        self.camera = raspiCam
        raspiCam.initialize(raspiCam.FORMAT_YUV_Y, self.flash, 10, 2)
        print "starting"

    def stop(self):
        raspiCam.finalize()
        print "stopping"
    
    def getImage(self, exposure, delay = 0):
        try:
            self.camSemaphore.acquire()
            if self.lastExposure != exposure:
                self.camera.setFlash(self.camera.FLASH_OFF)
                
                self.camera.getImage(self.img, exposure, 0)
                self.lastExposure = exposure
                self.camera.setFlash(self.flash)
            self.camera.getImage(self.img, exposure, delay)
        finally:
            self.camSemaphore.release()
        
    @cherrypy.expose
    def index(self, *args, **kwargs):
        html = 'index page: Use "jpeg" or "thumb" subpage \n'
        html += 'Args: %s\n'%str(args)
        html += "kwargs: %s\n"%str(kwargs)
        print "index"
        return html

    @cherrypy.expose
    def hellocamera(self, *args, **kwargs):
        html = 'raspberrypi camera'
        cherrypy.response.headers['Content-Length'] = len(html)
#        cherrypy.response.headers['Content-Type'] = "html"
        return html



    @cherrypy.expose
    def help(self, *args, **kwargs):
        html = '<html><h1>raspberrypi camera help</h1>'
        html += '<p>Available parameters (HTTP GET):</p>'
        html += '<ul>'
        html += '<li>autorefresh (if existant, have the auto-refresh line in the response header)</li>'
        html += '<li>exposure={int milliseconds}</li>'
        html += '<li>binning=2 (binning of 4 neighboring pixels</li>'
       

        html += '</ul></html>'
        cherrypy.response.headers['Content-Length'] = len(html)
        return html


 
    def binning(self, ratio):
        if ratio == 0:
            return self.img
        if ratio != 2:
            raise Exception("ratio %d not supported yet"%ratio)
        
        
        t = time.time()
        
        result = self.img
        result = result/4
        result = result[::2,::2]+result[1::2,0::2]+result[0::2,1::2]+result[1::2,1::2]
         
        print "t ", time.time()-t
        
        return result 
    
    
    @cherrypy.expose
    def gray(self, *args, **kwargs):
        
        print kwargs
        self.handleVars(self, *args, **kwargs)
        delay = 0
        if "delay" in kwargs.keys():
            delay = kwargs["delay"]

        self.getImage(self.exposure, int(delay))

        image = self.img.reshape((self.img.shape[0], self.img.shape[1]))
        
        if "binning" in kwargs.keys():
            image = self.binning(int(kwargs["binning"]))

        if kwargs.has_key('autorefresh'):
            cherrypy.response.headers['Refresh'] = 0
        if kwargs.has_key('bytes'):
            return image.tostring(order='C')

        data = StringIO()
        
        im = Image.fromarray(image)
        im.save(data, format='bmp')
        rawdata = data.getvalue()
        cherrypy.response.headers['Content-Type'] = "image/bmp"
        cherrypy.response.headers['Content-Length'] = len(rawdata)
        return rawdata
    
    @cherrypy.expose
    def jpeg(self, *args, **kwargs):
        
        
        self.handleVars(self, *args, **kwargs)
        self.getImage(self.exposure)

        image = self.img
        
        if "binning" in kwargs.keys():
            image = self.binning(int(kwargs["binning"]))
        

        if kwargs.has_key('autorefresh'):
            cherrypy.response.headers['Refresh'] = 0

        data = StringIO()
        
        im = Image.fromarray(image)
        im.save(data, format='jpeg')
        rawdata = data.getvalue()
        if kwargs.has_key('bytes'):
            return rawdata
        cherrypy.response.headers['Content-Type'] = "image/jpeg"
        cherrypy.response.headers['Content-Length'] = len(rawdata)
        return rawdata
    
    
    def handleVars(self, *args, **kwargs):
        print kwargs.keys()
        if "exposure" in kwargs.keys():
            self.exposure = int(kwargs['exposure'])
        
        



#daemonize()

#config muse somehow be doubled for port number to work
config = {
    'global': {     'server.socket_port' : 5900,
                    'server.thread_pool' : 20,
                    'tools.sessions.on' : True,
                    'tools.staticdir.root' : ".",
                    'request.show_tracebacks': True
              }
}
cherrypy.config.update({'server.socket_host': '130.75.27.3', 
                         'server.socket_port': 5900,
                         'request.show_tracebacks': True
                        })

root = Root()
root.start()
cherrypy.tree.mount(root, script_name='', config=config)
cherrypy.quickstart(root)
root.stop()
#cherrypy.engine.start()


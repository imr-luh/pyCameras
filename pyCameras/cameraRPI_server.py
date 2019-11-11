
import socket
import time
import picamera
import zmq
import socket
import time
import subprocess
import sys
import copy
#sys.path.insert(0, './imagezmq')

import imagezmq as imagezmq
from picamera.array import PiRGBArray
from picamera.array import PiYUVArray
from picamera import PiCamera
from io import BytesIO

HOST = "localhost"
PORT = 4223
UID1 = "G33"
UID2 = "G2Z"
UID3 = "EMP"
FREQ = 5000


class server_ctrl():
    def __init__(self):
        print("try init camera")
        self.cam = PiCamera()
        print("init server_ctrl")
        self.rawCap = PiRGBArray(self.cam)
        self.cam.raw_format = "bgr"
        self.cam.image_denoise = True
        self.cam.flash_mode ='off'
        print(self.cam.FLASH_MODES)
        
        
        self.cam.framerate = 30
        self.cam.awb_mode = 'auto'
        self.cam.exposure_mode ='auto'
        
        time.sleep(1)
        #self.cam.start_preview()
        time.sleep(3)


        #self.cam.exposure_mode = 'off'
        self.cam.awb_mode = 'off'
        self.cam.awb_gains = (1.7, 1.8)
        self.cam.iso = 100
        #self.cam.shutter_speed = 30000
        
        
        
        
    def start_img_server(self):
        self.sender = imagezmq.ImageSender(connect_to='tcp://130.75.27.44:5555')
        self.rpi = socket.gethostname()  # send RPi hostname with each image
        print("img server ready")
        print(self.rpi)  
    
    def start_ctrl_server(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind("tcp://*:5554")
        print("ctrl server ready")
        self.waitforcmd()
        time.sleep(3)
        #self.socket.send(b"Server ready.")
     
    def stream_live(self):
        #camera = picamera.PiCamera()
        #camera.resolution =(1920,1080)
        #camera.framerate = 15

        live_socket = socket.socket()
        #live_socket = self.socket
        live_socket.bind(('0.0.0.0', 5555))
        live_socket.listen(0)
        
        # Accept a single connection and make a file-like object out of it
        connection = live_socket.accept()[0].makefile('wb')
        
        try:
            self.cam.flash_mode ='on'
            self.cam.start_recording(connection, format='h264')
            self.cam.wait_recording(30)
            self.cam.stop_recording()
        finally:
            connection.close()
            live_socket.close()
            
            
    def sendImage(self):
          print("inside sendImge")
          self.rawCap.truncate(0)
          for i in range(20):
            img = self.cam.capture(self.rawCap, format="raw")
            img = self.rawCap.array
            self.rawCap.truncate(0)
          self.sender.send_image(self.rpi, img)
          print("Single Image sent.")
          

    def filenames(self,frames):
        frame = 0
        while frame < frames:
            yield 'image%02d.jpg' % frame
            frame += 1
          
    def captureSequence(self,numstr):
        print("Capture Sequence of ",numstr, "Images")
        rawCaplist = []
        img= []
        for i in range(0,int(numstr)):
         rawCaplist.append(PiRGBArray(self.cam))
         
#        self.cam.start_preview()
 #       time.sleep(1)
        start_time = time.time()
        #self.cam.capture_continuous(rawCaplist)
        #time.sleep(10)
        try:
            self.cam.capture_sequence(rawCaplist,format="raw",use_video_port = True)
        except Excetion:
            print("Capture Sequence not succesfull")
            self.cam.close()
        capture_time = time.time()-start_time
        print("capture time is",capture_time)
        print("fps is",int(numstr)/capture_time)
        self.cam.close()
        
        print("Images captured.Camera Closed. Now sending...")
        for i in range(0,int(numstr)):
            img.append(rawCaplist[i].array)
            rawCaplist[i].truncate(0)
            self.sender.send_image(self.rpi, img[i])
            
            
        print("images send")
        return
        
    def setResolution(self,numstr):
        print(len(numstr))
        if len(numstr)>6:
            width = int(numstr[0:4])
            height = int(numstr[4:8])
            print("Setting Resolution to ", width, "x", height)
            self.cam.resolution = (width, height)
        else:
            width = int(numstr[0:3])
            height = int(numstr[3:len(numstr)])
            print("Setting Resolution to ", width, "x", height)
            self.cam.resolution = (width, height)
        return
    
    def setExposure(self,numstr):
        print("Setting Exposure to",numstr)
        self.cam.shutter_speed = int(numstr)
        return
    def captureStream(self):
        numstr = 5
        
        start_time = time.time()
        self.cam.start_recording('my_video.h264')
        self.cam.wait_recording(5)
        self.cam.stop_recording()
        capture_time = time.time()-start_time
        print("capture time is",capture_time)
        print("fps is",int(numstr)/capture_time)
        
        
    
    def waitforcmd(self):
        cmd = self.socket.recv()
        print("rcv")
        self.socket.send(b"OK")
        
        cmdstr = str(cmd)
        splits = cmdstr.split(" ")
        topic = splits[0]
        numstr = splits[-1][0:-1]
        
        print("Received Command: ", cmdstr)
        if topic == "b'getImage'":
            print("get image detected")
            self.sendImage()
        elif topic == "b'sequence":
            self.captureSequence(numstr)
        elif topic == "b'Resolution":
            self.setResolution(numstr)
        elif topic == "b'Exposure":
            self.setExposure(numstr)
        elif topic == "b'Framerate":
            self.cam.framerate = int(numstr)
            print("Setting Framerate to ", numstr)
        elif topic == "b'live":
            self.stream_live()
        else:
            print(cmdstr)
        return 1
            
  
    
if __name__ == "__main__":
    
    server = server_ctrl()
    server.start_ctrl_server()
    server.start_img_server()
    
    
    
    run = 1
    while run:
        run = server.waitforcmd()
        print(" run ist now " ,run)
    print("Server closed!")


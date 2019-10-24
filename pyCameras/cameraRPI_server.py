import socket
import time
import picamera
import zmq
import socket
import time
import subprocess
import sys
import copy
sys.path.insert(0, './imagezmq')

import imagezmq as imagezmq
from picamera.array import PiRGBArray
from picamera.array import PiBayerArray
from picamera import PiCamera


HOST = "localhost"
PORT = 4223
UID1 = "G33"
UID2 = "G2Z"
UID3 = "EMP"
FREQ = 5000




class server_ctrl():
    def __init__(self):
        self.cam = PiCamera()
        self.rawCap = PiRGBArray(self.cam)
        self.cam.raw_format = "bgr"
        
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
        time.sleep(1)
        self.socket.send(b"Server ready.")
     
    def stream_live(self):
        camera = picamera.PiCamera()
        camera.resolution =(1920,1080)
        camera.framerate = 15

        live_socket = socket.socket()
        #live_socket = self.socket
        live_socket.bind(('0.0.0.0', 8000))
        live_socket.listen(0)
        
        # Accept a single connection and make a file-like object out of it
        connection = live_socket.accept()[0].makefile('wb')
        
        try:
            camera.start_recording(connection, format='h264')
            camera.wait_recording(30)
            camera.stop_recording()
        finally:
            connection.close()
            live_socket.close()
    def sendImage(self):
          img = self.cam.capture(self.rawCap, format="raw")
          img = self.rawCap.array
          self.rawCap.truncate(0)
          self.sender.send_image(self.rpi, img)
          print("Single Image sent.")
          
    def captureSequence(self,numstr):
        print("Capture Sequence of ",numstr, "Images")
        rawCaplist = []
        img= []
        for i in range(0,int(numstr)):
         rawCaplist.append(PiRGBArray(self.cam))
         
        self.cam.capture_sequence(rawCaplist,format="raw")
        
        for i in range(0,int(numstr)):
            img.append(rawCaplist[i].array)
            rawCaplist[i].truncate(0)
            self.sender.send_image(self.rpi, img[i])

            
    def waitforcmd(self):
        cmd = self.socket.recv()
        cmdstr = str(cmd)
        splits = cmdstr.split(" ")
        topic = splits[0]
        numstr = splits[-1][0:-1]
        
        print("Received Command: ", cmdstr)
        if topic == "b'getImage":
            self.sendImage()
        elif topic == "b'sequence":
            self.captureSequence(numstr)
        return
            

          


    
    
if __name__ == "__main__":
    
    server = server_ctrl()
    server.start_ctrl_server()
    server.start_img_server()
    #server.stream_live()
    
    run = 1
    while run:
        run = server.waitforcmd()
    print("Server closed!")


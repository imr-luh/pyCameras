

import socket

import time

import picamera

import zmq

import subprocess

import sys

import copy

# i2c stuff for arduino communication
import smbus


#sys.path.insert(0, './imagezmq')



import imagezmq as imagezmq

from picamera.array import PiRGBArray

from picamera.array import PiYUVArray

from picamera import PiCamera

from io import BytesIO


# i2c stuff
bus = smbus.SMBus(1)
address = 0x05
Register = 42


class server_ctrl():

    def __init__(self):
        print("server class init")
        self.cam = None
        self.context = None
        self.framerate = 30
        
    def start_camera(self):
        self.cam = PiCamera()

        self.rawCap = PiRGBArray(self.cam)

        self.cam.raw_format = "bgr"

        self.cam.image_denoise = True

        self.cam.flash_mode ='off'        

        self.cam.framerate = self.framerate

        self.cam.awb_mode = 'auto'

        self.cam.exposure_mode ='auto'

        self.cam.awb_mode = 'off'

        self.cam.awb_gains = (1.7, 1.8)

        self.cam.iso = 100

        

    def start_img_server(self):

        self.sender = imagezmq.ImageSender(connect_to='tcp://130.75.27.44:5555')

        self.rpi = socket.gethostname()  # send RPi hostname with each image

        print("img server ready")
  


    def start_ctrl_server(self):
        try:

            self.context = zmq.Context()

            self.socket = self.context.socket(zmq.REP)

            self.socket.bind("tcp://*:5554")

            print("ctrl server ready")

            self.waitforcmd()

            time.sleep(3)
        except Exception as e:
            self.close_everything()
            print("start ctrl server did not work")
            print(e)
            

        #self.socket.send(b"Server ready.")

     



    def sendImage(self):
          if self.cam == None:
             self.start_camera()

          print("inside sendImge")

          self.rawCap.truncate(0)

          img = self.cam.capture(self.rawCap, format="raw",use_video_port=True)

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

        try:
          print("Capture Sequence of ",numstr, "Images")
          self.start_camera()

          img= []
            
          rawCaplist = [PiRGBArray(self.cam) for i in range(int(numstr))]
          
          self.cam.capture_sequence(rawCaplist,format="raw",use_video_port = True)

        except Exception:

            print("Capture Sequence not succesfull")

            self.cam.close()

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
    
    def setTriggermode(self,numstr):
        try:
            #self.cam.close()
            Dataset = bus.write_byte(address,int(numstr))
            print("Raspberry Schickt folgende Zahlen: ", numstr)
            time.sleep(0.1)
            received = bus.read_byte(address)
            print("Arduino received Frametime of: ", received)
            
        except Exception:
            print("Setting Triggermode not succesfull")
            if self.cam is not None:
               self.cam.close()
               
    def close_everything(self):

        if self.cam is not None:
            self.cam.close()
        if self.socket is not None:
            self.socket.close()
        #if self.context is not None:
           # self.context.term()
        print("everything closed")


    

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

            self.framerate = int(numstr)

            print("Setting Framerate to ", numstr)
            
        elif topic == "b'triggermode":
            self.setTriggermode(numstr)

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





# vlc tcp/h264://130.75.27.134:8000/ --h264-fps=50


import sys
import cv2
import zmq
import numpy as np
import time
import logging
from pyCameras.cameraTemplate import CameraTemplate
import pyCameras.imagezmq as imagezmq




# import imagezmq.imagezmq.imagezmq

LOGGING_LEVEL = None

class Controller():
  def __init__(self):
    context = zmq.Context()
    print("Init Controller…")
    self.socket = context.socket(zmq.REQ)
    self.image_hub = imagezmq.ImageHub()

      # imagezmq.imagezmq.imagezmq.ImageHub()



  def connect_ctrls(self):
    #  Socket to talk to server
    print("Connecting to crtl server…")
    self.socket.connect("tcp://130.75.27.143:5554")
    print("Connected to crtl server…")
    self.socket.send(b"Client ready.")
    #  Get the reply.
    message = self.socket.recv()
    print("Server: ", message)
    # message = self.socket.recv()
    # print("Server: ", message)



  def send_command(self, cmd, val):
    if val is not None:
      cmd = bytes(cmd + " " + str(val), 'ascii')
    else:
      cmd = bytes(cmd, 'ascii')
    self.socket.send(cmd)

    print("Command send:",cmd)
    msg = self.socket.recv()
    return

#################################################################################%%%%%%%%%%
class Camera(CameraTemplate):
  def __init__(self,device_handle):
    self.logger = logging.getLogger(__name__)
    if LOGGING_LEVEL is not None:
      self.logger.setLevel(LOGGING_LEVEL)

    self.device_handle = device_handle
    self._expected_triggered_images = None
    self.device_handle.connect_ctrls()
    self.device = None


  def recv_live(self,number):
    self.device_handle.send_command("live", number)
    time.sleep(4)
    cap = cv2.VideoCapture('tcp://130.75.27.143:5555')


    # Check if camera opened successfully
    if (cap.isOpened()== False):
      print("Error opening video  file")

    # Read until video is completed
    while(cap.isOpened()):

      # Capture frame-by-frame
      ret, frame = cap.read()
      if ret == True:

        # Display the resulting frame
        cv2.namedWindow("Frame", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Frame", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow('Frame', frame)

        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
          break

      # Break the loop
      else:
        break

    # When everything done, release
    # the video capture object
    cap.release()

    # Closes all the frames
    cv2.destroyAllWindows()

  def getImage(self):
    self.device_handle.send_command("getImage",None)
    rpi_name, img = self.device_handle.image_hub.recv_image()
    self.device_handle.image_hub.send_reply(b'OK')
    print("Single image recieved")
    return img

    return img


  def getImages(self):
    self.prepareRecording()
    return self.record()

  def prepareRecording(self,num):
    print("prepare Reocrding")
    self._expected_triggered_images = num
    return

  def record(self):
    print("record sequence of",self._expected_triggered_images, "Images")
    self.device_handle.send_command("sequence", self._expected_triggered_images)
    print("record command send")
    for i in range(self._expected_triggered_images):

      print(i,"Image recieved")
      rpi_name, img = self.device_handle.image_hub.recv_image()
      print("img")
      self.device_handle.image_hub.send_reply(b'OK')
      cv2.imwrite("image2_%i.jpg" %i, img)

    print("Sequence recieved")
    return

  def setExposureMicrons(self, microns=None):
    self.device_handle.send_command("Exposure", microns)
    return microns

  def setFramerate(self, Framerate=None):
    self.device_handle.send_command("Framerate", Framerate)
    return Framerate

  def captureStream(self):
    self.device_handle.send_command("stream",5)
    return 5


  def setResolution(self, resolution=None):

    if resolution[1]<1000 and resolution[0]>999:
      resolution = 1000*resolution[0]+resolution[1]
    elif resolution[1] > 999 and resolution[0]>999:
      resolution = 10000 * resolution[0] + resolution[1]
    else:
       resolution = 1000 * resolution[0] + resolution[1]


    print(resolution)
    if resolution is not None:
      self.device_handle.send_command("Resolution", resolution)

    return resolution

  def setTriggerMode(self, mode=None):
    print("setting mode to:",mode)

    if mode == "out" or mode =="OUT":
      print("inside out")
      mode_num = 1
    else:
      print("inside zero")
      mode_num = 0

    if mode is not None:
      self.device_handle.send_command("triggermode", mode_num)




if __name__ == "__main__":
  device_handle = Controller()
  cam = Camera(device_handle)
  cam.setTriggerMode('out')
  cam.setFramerate(1)
  # cam.recv_live(30)
  cam.prepareRecording(15)
  time.sleep(3)
  cam.record()
  time.sleep(3)

  # cam.setResolution([1280,720])
  # cam.setResolution([3280, 2464]) not

  # cam.setResolution([2560,1920])
  # cam.setResolution([640, 480])
  # cam.setResolution([1920, 1080])

  # cam.setExposureMicrons(40000)
  # cam.setTriggerMode('off')



  # cam.recv_live()
  # cam.getImage()
  # cam.captureStream()

  # cam.setResolution([3280, 2464]) not
  # cam.setResolution([3000, 2000]) #working
  # cam.setResolution([3240, 2160]) not
  # cam.setResolution([3200, 2048]) #working
  #
  # for i in range(500):
  #     img = cam.getImage()
  #     print(i)
  #
  # cv2.imwrite("image4k.jpg",img)
  # cv2.namedWindow("Received Image", cv2.WND_PROP_FULLSCREEN)
  # cv2.setWindowProperty("Received Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
  # cv2.imshow("Received Image", img)
  # cv2.waitKey(1)




  # client.recv_live()
  # cmd = bytes("camclose", 'ascii')
  # client.send_command(cmd)


  # cam.setFramerate(1)
  # set 3 results in 0,4 fps
  # set 25 results in 1,3 fps
  # set 45 results in 1,56 fps
  # set 50 results in 1,65
  # set 55 results in 1,75 fps
  # set 64 results in 1,75 fps
  # set 65 results in 7,75 fps
  # set 95 results in 1.48 fps
# set 115 results in 1,57 fps

#
  #   set 115 results in 1,45 fps
#   set 117 results in 1,44 fps
  # set 119 results in 1,78
  # set 120 results in 1.93 fps
  # set 130 results in invalid framerate
  # set 140 results in invalid framerate
  # set 160 results in invalid framerad
  # set 180 results in invalid framerat
  # set 200 results in inavild framrat

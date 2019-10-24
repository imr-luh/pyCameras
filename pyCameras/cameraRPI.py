# vlc tcp/h264://130.75.27.134:8000/ --h264-fps=50


import sys
import cv2
import zmq
import numpy as np
import time
import logging
from pyCameras.cameraTemplate import CameraTemplate
# sys.path.insert(0, './imagezmq/imagezmq/imagezmq')  # imagezmq.py is in ../imagezmq


import imagezmq
# import imagezmq.imagezmq.imagezmq

LOGGING_LEVEL = None

class Controller():
  def __init__(self):
    context = zmq.Context()
    print("Connecting to crtl server…")
    self.socket = context.socket(zmq.REQ)
    self.image_hub = imagezmq.ImageHub()

      # imagezmq.imagezmq.imagezmq.ImageHub()



  def connect_ctrls(self):
    #  Socket to talk to server
    print("Connecting to crtl server…")
    self.socket.connect("tcp://130.75.27.134:5554")
    self.socket.send(b"Client ready.")
    #  Get the reply.
    message = self.socket.recv()
    print("Server: ", message)



  def send_command(self, cmd, val):
    if val is not None:
      cmd = bytes(cmd + " " + str(val), 'ascii')
    else:
      cmd = bytes(cmd, 'ascii')
    self.socket.send(cmd)

    print("Command send:",cmd)

#################################################################################%%%%%%%%%%
class Camera():
  def __init__(self,device_handle):
    self.logger = logging.getLogger(__name__)
    if LOGGING_LEVEL is not None:
      self.logger.setLevel(LOGGING_LEVEL)

    self.device_handle = device_handle
    self._expected_triggered_images = None
    self.device_handle.connect_ctrls()


  def recv_live(self):

    time.sleep(4)
    cap = cv2.VideoCapture('tcp://130.75.27.134:8000')


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
    self.device_handle.send_command("getImage")
    rpi_name, img = self.device_handle.image_hub.recv_image()
    self.device_handle.image_hub.send_reply(b'OK')
    print("Single image recieved")
    return img

    return img


  def getImages(self):
    self.prepareRecording()
    return self.record()

  def prepareRecording(self,num):
    self._expected_triggered_images = num
    return

  def record(self):
    print("record sequence of",self._expected_triggered_images, "Images")
    self.device_handle.send_command("sequence", self._expected_triggered_images)


    for i in range(self._expected_triggered_images):
      rpi_name, img = self.device_handle.image_hub.recv_image()
      self.device_handle.image_hub.send_reply(b'OK')
      cv2.imwrite("image%i.jpg" %i, img)

    print("Sequence recieved")
    return

  def setResolution(self, resolution=None):
    """

    """
    if resolution is not None:
      self.device_handle.send_command("Resolution", resolution)



if __name__ == "__main__":
  device_handle = Controller()
  cam = Camera(device_handle)
  cam.prepareRecording(4)
  cam.record()
  #img = cam.getImage()

  # cv2.imwrite("image.jpg",img)
  # cv2.namedWindow("Received Image", cv2.WND_PROP_FULLSCREEN)
  # cv2.setWindowProperty("Received Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
  # cv2.imshow("Received Image", img)
  # cv2.waitKey(1)
  time.sleep(3)



  # client.recv_live()
  # cmd = bytes("camclose", 'ascii')
  # client.send_command(cmd)




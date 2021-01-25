from threading import Thread
import time
from picamera.array import PiRGBArray
from picamera import PiCamera


class CaptureThread(object):
    # def __init__(self, continous_capture, camera, rawCap, format, **kwargs):
    def __init__(self, cam, rawCap, continous_capture, frame_goal, fmt, **kwargs):
        # initialize the camera
        if cam is None:
            self.RaspiCam = PiCamera()
            self.ImageFormat = "bgr"
            self.rawCapture = PiRGBArray(self.RaspiCam)
        else:
            self.RaspiCam = cam
            self.rawCapture = rawCap

        self.format = fmt
        self.continous_capture = continous_capture
        self.frame_goal = frame_goal
        # set optional camera parameters (refer to PiCamera docs)
        for (arg, value) in kwargs.items():
            setattr(self.RaspiCam, arg, value)

        if fmt.lower() == "bgr" or fmt.lower() =="rgb":
            self.ImageFormat = "bgr"
            self.rawCapture = PiRGBArray(self.RaspiCam)

        if fmt.lower() == "yuv":
            self.ImageFormat = "yuv"
            self.rawCapture = PiRGBArray(self.RaspiCam)

        if self.continous_capture:
            self.stream = self.RaspiCam.capture_continuous(self.rawCapture, format = self.format, use_video_port = True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False
        self.frame_counter = 0

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target = self.update, args = ())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # while self.frame_counter < self.frame_goal:
        if self.continous_capture:
            for f in self.stream:
                # self.rawCapture.truncate(0)
                # grab the frame from the stream and clear the stream in
                # preparation for the next frame
                self.frame = f.array
                # time.sleep(0.01)
                self.rawCapture.truncate(0)
                # self.rawCapture.flush()
                # f.clear()

        else:
            self.RaspiCam.capture(self.rawCapture, format = self.format, use_video_port = True)
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = self.rawCapture.array
            # self.rawCapture.seek(0)
            self.rawCapture.truncate(0)

            # self.frame_counter += 1

        # if the thread indicator variable is set, stop the thread
        # and resource camera resources
        if self.continous_capture:
            self.stream.close()
        self.rawCapture.close()
        self.RaspiCam.close()
        return

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

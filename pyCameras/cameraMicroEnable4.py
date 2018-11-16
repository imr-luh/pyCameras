#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Nils Melchert"
__copyright__ = "Copyright 2017, LUH: IMR"
# __license__ = ""
__version__ = "0.1"
__maintainer__ = "Nils Melchert"
__email__ = "nils.melchert@imr.uni-hannover.de"
__status__ = "alpha"
__package_name__ = "MicroEnable4"
__developer__ = __author__

'''
Based on the SDK-Wrapper from the framegrabber card microEnableIV from Silicon Software
'''

try:
	import SiSoPyInterface as s
except ImportError:
	raise ImportError('SiSo module not loaded successfully')

import coloredlogs, logging
import sys
import struct
import numpy as np
import cv2
import time
import copy

from pyCameras.cameraTemplate import ControllerTemplate, CameraTemplate

MAX_ITER = 10000000

LOGGING_LEVEL = logging.DEBUG

class Controller(ControllerTemplate):

	def __init__(self, maxNrOfboards=10):
		super(Controller, self).__init__()
		self.logger = logging.getLogger(__name__)
		coloredlogs.install(level=logging.DEBUG, logger=self.logger)

		if LOGGING_LEVEL is not None:
			self.logger.setLevel(LOGGING_LEVEL)

		self.device_handles = []
		self.maxNrOfboards = maxNrOfboards
		self.logger.debug('Starting microEnable4 camera controller')

	def __repr__(self):
		return "<MicroEnable Frame Grabber Controller>"


	# returns count of available boards
	def _getNrOfBoards(self):
		nrOfBoards = 0
		(err, buffer, buflen) = s.Fg_getSystemInformation(None, s.INFO_NR_OF_BOARDS, s.PROP_ID_VALUE, 0)
		if (err == s.FG_OK):
			nrOfBoards = int(buffer)
		return nrOfBoards

	def updateDeviceHandles(self):
		"""
		Refresh the list of available devices
		"""
		self.logger.debug('Searching for frame grabber devices')

		nrOfBoardsFound = 0
		nrOfBoardsPresent = self._getNrOfBoards()

		for i in range(self.maxNrOfboards):
			if s.Fg_getBoardType(i) == s.PN_MICROENABLE4VD4CL:
				self.device_handles.append(i)
				nrOfBoardsFound += 1

			if nrOfBoardsFound >= nrOfBoardsPresent:
				break

		self.logger.debug('Found {num} frame grabber device(s): {devices}'.format(num=len(self.device_handles),
									devices=self.device_handles))

	def closeController(self):
		self.logger.info("ME4-VD4 Controller shutdown")


class Camera(CameraTemplate):
	"""
	microEnable frame grabber implementation based on the Silicon Software SDK
	to work with cameras with camera link standard in FULL configuration.
	"""
	def __init__(self, device_handle, applet = 'Acq_FullAreaGray8'):
		"""
		Implementation of the microEnable4-VD4 framegrabber

		Parameters
		----------
		device_handle : int
			Framegrabber device handle to identify the frame grabber
		"""
		super(Camera, self).__init__(device_handle)
		self.logger = logging.getLogger(__name__)
		if LOGGING_LEVEL is not None:
			self.logger.setLevel(LOGGING_LEVEL)

		if s.Fg_getBoardType(device_handle) != s.PN_MICROENABLE4VD4CL:
			self.logger.error('Board {0} is not supported by this package!'.format(s.Fg_getBoardNameByType(s.Fg_getBoardType(device_handle), s.Fg_getBoardType(device_handle))))
			exit(1)

		self.triggerModeSetting = 'off'

		self._applet = applet
		self.logger.debug('Initializing Framegrabber...')

		self.device = s.Fg_InitEx(self._applet, device_handle, 0)

		# error handling
		err = s.Fg_getLastErrorNumber(self.device)
		if err < 0:
			mes = s.Fg_getErrorDescription(err)
			self.logger.error("Error", err, ":", mes)
			sys.exit()
		else:
			self.logger.debug("Grabber initialized successfully!")

		self.clser_ref = s.clSerialInit(0)
		# Setting Camera link factory profile
		# Resolution: 1280x1024, Image freq.: 430, Mode: 8x8, CL-Conf.: FULL
		factory_profile = ':f7'
		self._clSerialWrite(command=factory_profile)

		s.Fg_setParameterWithInt(self.device, s.FG_BITALIGNMENT, s.FG_LEFT_ALIGNED, self.device_handle)
		s.Fg_setParameterWithInt(self.device, s.FG_GEN_ENABLE, s.FG_CAMPORT, self.device_handle)
		s.Fg_setParameterWithInt(self.device, s.FG_TRIGGER_LEGACY_MODE, s.FG_ON, self.device_handle)
		self.__width = s.Fg_getParameterWithInt(self.device, s.FG_WIDTH, self.device_handle)[1]
		self.__height = s.Fg_getParameterWithInt(self.device, s.FG_HEIGHT, self.device_handle)[1]

		self.free_run_buffer = self._prepareLiveBuffer()

		self._pics_to_be_recorded = None
		self.buffer_handle = None
		self.img_list = list()
		# time.sleep(1)

	def __del__(self):
		s.Fg_FreeMemEx(self.device, self.free_run_buffer)
		s.clSerialClose(self.clser_ref[1])
		self.closeDevice()

	def _prepareLiveBuffer(self):
		"""
		Prepares a small buffer (2 images), which will be used to store images from:
			- getImage()
			- _liveView()
		For other images a new buffer will be allocated.

		Returns
		-------
			free_run_buffer: memory_handle
				Memory handle which adresses the allocated buffer memory
		"""
		# Calculate buffer size
		samplePerPixel = 1
		bytePerSample = 1
		# Buffer size of two images
		nbBuffers = 2
		totalBufferSize = self.__width * self.__height * samplePerPixel * bytePerSample * nbBuffers
		free_run_buffer = s.Fg_AllocMemEx(self.device, totalBufferSize, nbBuffers)
		# Give time to allocate buffer (tests without sleep/too short sleep failed)
		time.sleep(.5)

		return free_run_buffer

	def _freeLiveBuffer(self):
		"""
		Frees allocated memory of self.free_run_buffer.
		"""
		# TODO: Use this funtion to free and change buffer when changing the resolution!
		s.Fg_FreeMemEx(self.device, self.free_run_buffer)

	def _prepareImageBuffer(self, numImages=1):
		"""
		Prepares a bigger buffer for e.g. more than 1 triggered image.

		Notes: This function uses a sleep function to guarantee allocated memory.

		Parameters
		----------
		numImages : int
			Number of expected images.

		Returns
		-------
			buffer: memory_handle
				Memory handle which adresses the allocated buffer memory
		"""
		# Calculate buffer size
		samplePerPixel = 1
		bytePerSample = 1
		nbBuffers = numImages
		totalBufferSize = self.__width * self.__height * samplePerPixel * bytePerSample * nbBuffers
		buffer = s.Fg_AllocMemEx(self.device, totalBufferSize, nbBuffers)
		# Give time to allocate buffer (tests without sleep/too short sleep failed)
		time.sleep(.5)

		return buffer

	def _clSerialWrite(self, command, serial_port = 0):
		import time
		# print('Sending command:', command)
		# self.clser_ref = s.clSerialInit(serial_port)
		self.logger.debug('Setting command <{0}> on camera via camera link serial interface'.format(command))
		s.clSerialWrite(self.clser_ref[1], command, sys.getsizeof(command), 100)

		# TODO: Implement the serial read functionality. (Seems like Silicon Software did some mistakes wrapping the function)
		# time.sleep(0.5)
		# print("Bytes in Buffer: ", s.clGetNumBytesAvail(self.clser_ref[1]))
		# bytes_avail = 0
		#
		# time.sleep(0.5)
		# while bytes_avail < 1:
		# 	bytes_avail = s.clGetNumBytesAvail(self.clser_ref[1])[1]
		# 	print('Waiting for data...')
		# 	time.sleep(0.1)
		#
		# # buff = bytes('00000000', encoding='ASCII')
		# a = s.clSerialRead(self.clser_ref[1], command, 1, 1000)


		# s.clSerialClose(self.clser_ref[1])
		# return command



	@staticmethod
	def listDevices():
		"""
		List available ME4-VD4 frame grabbers
		Returns
		-------
		cams : list
			List of available grabber devices
		"""
		return Controller().listDevices()

	def openDevice(self):
		"""
		Opens a camera device
		"""
		if not self.isOpen():
			self.logger.debug('Creating grabber object')
			self.device = s.Fg_InitEx(self._applet, self.device_handle, 0)
		else:
			self.logger.debug('Grabber object already exists')

	def closeDevice(self):
		"""
		Closes camera device
		"""
		self.logger.debug('Freeing the framegrabber device...')

		if s.Fg_FreeGrabber(self.device) == s.FG_OK:
			self.logger.debug('Freed successfully!')
		else:
			self.logger.error('Failed to free the grabber!')
			exit(1)

	def isOpen(self):
		"""
		Check if the device for this instance is currently open and ready to
		communicate

		Returns
		-------
		bool
			True if the camera connection is open, False if it is not
		"""
		# ME4-VD4 frame grabbers do not have any isOpen-function by itself.
		# Assuming that if there is a device given in self.device, device is opened.
		if self.device is not None:

			return True
		else:
			return False

	def setExposureMicrons(self, microns=None):
		"""
		Set the exposure time to the given value in microseconds or read the
		current value by passing None

		Parameters
		----------
		microns : int
			Desired exposure time in microseconds that should be set, or None
			to read the current exposure time

		Returns
		-------
		microns : int
			The exposure time in microseconds after applying the passed value
		"""

		if microns is not None:
			self.logger.debug('Setting exposure time to {microns} micro seconds'.format(microns=microns))
			# Make sure the exposure time is less than (frame_rate)^-1

			s.Fg_setParameterWithInt(self.device, s.FG_EXPOSURE, microns, self.device_handle)
		if self.setTriggerMode() == 'in' or self.setTriggerMode() == 'off':
			return s.Fg_getParameterWithInt(self.device, s.FG_EXPOSURE, self.device_handle)[1]
		else:
			raise NotImplementedError('Requesting exposure time for a trigger mode that is not implemented')

	def _setParamWithInt(self, parameter, value):
		retval = [-99, -99]
		iterations = 0
		s.Fg_setParameterWithInt(self.device, parameter, value, self.device_handle)
		while  retval[1] != value:
			retval = s.Fg_getParameterWithInt(self.device, parameter, self.device_handle)
			if iterations > MAX_ITER:
				raise TimeoutError("Max iterations reached while waiting to set parameter!")
			iterations += 1

	def setTriggerMode(self, mode=None):
		"""
		Set the trigger mode of the camera to either "in", "out" or "off", or
		read the current trigger setting ba passing None

		Parameters
		----------
		mode : str
			The desired trigger mode. "in" means the camera receives a trigger
			signal, "out" means the camera sends a trigger signal, "off"" means
			the camera does not react to triggers. To read the current trigger
			setting pass None

		Returns
		-------
		mode : str
		The trigger mode after applying the passed value
		"""
		self.logger.debug("Setting trigger mode to: {mode}".format(mode=mode))
		if mode is None:
			return self.triggerModeSetting
		elif isinstance(mode, str):
			if mode.lower() == 'in':
				# Setting camera to external trigger mode
				self._clSerialWrite(':h1')
				self._setParamWithInt(s.FG_TRIGGERMODE, s.ASYNC_TRIGGER)
				self._setParamWithInt(s.FG_EXSYNCON, s.FG_ON)
				self._setParamWithInt(s.FG_EXSYNCPOLARITY, s.FG_HIGH)
				self._setParamWithInt(s.FG_TRIGGERIN_SRC, s.TRGINSRC_0)
				self.triggerModeSetting = 'in'

				time.sleep(.2)

			elif mode.lower() == 'out':
				# TODO: Implement out trigger
				self.triggerModeSetting = 'out'
				raise NotImplementedError('Sending triggers is not implemented yet!')
			elif mode.lower() == 'off':
				# Setting camera to freerun mode
				self._clSerialWrite(':h1')
				self._setParamWithInt(s.FG_TRIGGERMODE, s.GRABBER_CONTROLLED)
				self._setParamWithInt(s.FG_EXSYNCON, s.FG_ON)
				self._setParamWithInt(s.FG_EXSYNCPOLARITY, s.FG_HIGH)
				self._setParamWithInt(s.FG_TRIGGERIN_SRC, s.TRGINSRC_0)
				self.triggerModeSetting = 'off'

				time.sleep(.2)

			else:
				raise ValueError('Unexpected value in setTriggerMode. Expected "in", "out", or "off". Got {mode}'.format(mode=mode))
			return self.triggerModeSetting
		else:
			raise TypeError('Trigger Mode should be None, "in", "out", or "off". Got {mode}'.format(mode=mode))

	def setResolution(self, resolution=None):
		"""
		Set the resolution of the camera to the given values in pixels or read
		the current resolution by passing None
		This is actually not the real camera resolution but only the amount of pixels
		considered by the frame grabber.

		Parameters
		----------
		resolution : tuple
			Desired camera resolution in the form (width, height), or None to
			read the current resolution

		Returns
		-------
		resolution : tuple
			The set camera resolution after applying the passed value
		"""
		if resolution is not None:
			self.logger.debug('Setting <Width> to {width}'.format(width=resolution[0]))
			s.Fg_setParameterWithInt(self.device, s.FG_WIDTH, resolution[0], self.device_handle)
			self.logger.debug('Setting <Height> to {height}'.format(height=resolution[1]))
			s.Fg_setParameterWithInt(self.device, s.FG_HEIGHT, resolution[1], self.device_handle)

		self.__width = s.Fg_getParameterWithInt(self.device, s.FG_WIDTH, self.device_handle)[1]
		self.__height = s.Fg_getParameterWithInt(self.device, s.FG_HEIGHT, self.device_handle)[1]
		return self.__width, self.__height

	def _setRoiOffset(self, offset=None):
		"""
		Sets the offset of the current region of interes (ROI) of the camera

		Parameters
		----------
		offset : touple
			X and Y offsets of the acquisition ROI
		Returns
		-------
		offset : tuple
			The set ROI-offset after applying the passed value
		"""
		if offset is not None:
			self.logger.debug('Setting <X-ROI-Offset> to {0}'.format(offset[0]))
			s.Fg_setParameterWithInt(self.device, s.FG_XOFFSET, offset[0], self.device_handle)
			self.logger.debug('Setting <Y-ROI-Offset> to {0}'.format(offset[1]))
			s.Fg_setParameterWithInt(self.device, s.FG_YOFFSET, offset[1], self.device_handle)

		x_offset = s.Fg_getParameterWithInt(self.device, s.FG_XOFFSET, self.device_handle)[1]
		y_offset = s.Fg_getParameterWithInt(self.device, s.FG_YOFFSET, self.device_handle)[1]
		return x_offset, y_offset

	def getImage(self, *args, **kwargs):
		"""
		Get an image from the camera device

		*args and **kwargs are ignored parameters!

		This function uses self.free_run_buffer to write images to.

		Returns
		-------
		img : np.ndarray
			Current camera image
		"""
		# while s.Fg_getStatusEx(self.device, s.BUFFER_STATUS, 0, self.device_handle, self.free_run_buffer) == 1:
		# 	print (s.Fg_getStatusEx(self.device, s.BUFFER_STATUS, 0, self.device_handle, self.free_run_buffer))
		# time.sleep(0.1)

		s.Fg_AcquireEx(self.device, self.device_handle, s.GRAB_INFINITE, s.ACQ_STANDARD, self.free_run_buffer)

		cur_img_no = 0
		iterations = 0
		# while s.Fg_getStatusEx(self.device, s.NUMBER_OF_GRABBED_IMAGES, 0, self.device_handle, self.free_run_buffer) != 1:
		# 	pass
		while cur_img_no == 0:
			cur_img_no = s.Fg_getLastPicNumberEx(self.device, self.device_handle, self.free_run_buffer)
			# print(cur_img_no)
			if iterations > MAX_ITER:
				raise TimeoutError("Max iterations reached while waiting for image! Missing a trigger signal?")
			iterations += 1

		img = s.Fg_getImagePtrEx(self.device, cur_img_no, self.device_handle, self.free_run_buffer)

		np_img = s.getArrayFrom(img, self.__width, self.__height)

		s.Fg_stopAcquireEx(self.device, self.device_handle, self.free_run_buffer, s.STOP_ASYNC)

		return np_img.copy()
		# return np_img

		# return copy.copy(np_img)

	def prepareRecording(self, num):
		"""
		Prepares num images to be grabbed. This function is not blocking.
		Calling "grabStop()" will end acquisition.

		Parameters
		----------
		num : int
			Number of images that should be recorded
		"""
		self._pics_to_be_recorded = num
		self.img_list = []

		self.buffer_handle = self._prepareImageBuffer(num)


	def record(self):
		"""
		Stop grabbing images and return camera to continuous mode.
		"""
		s.Fg_AcquireEx(self.device, self.device_handle, self._pics_to_be_recorded, s.ACQ_STANDARD, self.buffer_handle)

		while s.Fg_getStatusEx(self.device, s.NUMBER_OF_GRABBED_IMAGES, 0, self.device_handle,
		                       self.buffer_handle) != self._pics_to_be_recorded:
			pass

		s.Fg_stopAcquireEx(self.device, self.device_handle, self.buffer_handle, s.STOP_ASYNC)

		for img_no in range(self._pics_to_be_recorded):
			# print('current img_no:', img_no)
			# img_buff_no = s.Fg_getImageEx(self.device, s.SEL_NUMBER, img_no+1, self.device_handle, 10000, self.buffer_handle)
			# print('img num from getImage:', img_buff_no)
			img = s.Fg_getImagePtrEx(self.device, img_no + 1, self.device_handle, self.buffer_handle)
			# print('Image pointer:', img)
			self.img_list.append(s.getArrayFrom(img, self.__width, self.__height).copy())
		# print('Got image number:', img_no+1)

		# s.Fg_stopAcquireEx(self.device, self.device_handle, self.buffer_handle, s.STOP_ASYNC)
		s.Fg_FreeMemEx(self.device, self.buffer_handle)
		self._pics_to_be_recorded = None

		return self.img_list

	def _liveView(self):
		"""
		Live image stream an visualization through OpenCV window.

		Leave _liveView by pressing "q"
		"""
		cv2.startWindowThread()
		cv2.namedWindow("IMG", 2)
		cv2.resizeWindow("IMG", 900, 900)

		s.Fg_AcquireEx(self.device, self.device_handle, s.GRAB_INFINITE, s.ACQ_STANDARD, self.free_run_buffer)

		last_img = -1
		while True:
			cur_img_no = -1
			iterations = 0
			while last_img == cur_img_no or cur_img_no <= 0:
				cur_img_no = s.Fg_getLastPicNumberEx(self.device, self.device_handle, self.free_run_buffer)
				if iterations > MAX_ITER:
					raise TimeoutError("Max iterations reached while waiting for image! Missing a trigger signal?")
				iterations += 1
			last_img = cur_img_no
			# self.logger.debug("Actual image number is: {cur_img_no}".format(cur_img_no=cur_img_no))

			img_data = s.Fg_getImagePtrEx(self.device, cur_img_no, self.device_handle, self.free_run_buffer)
			# Convert to numpy array
			live_img =  s.getArrayFrom(img_data, self.__width, self.__height)

			cv2.imshow("IMG", live_img)
			key = cv2.waitKey(1) & 0xFF
			if key == ord("q"):
				cv2.destroyAllWindows()
				break

		# Cleanup
		s.Fg_stopAcquireEx(self.device, self.device_handle, self.free_run_buffer, s.STOP_ASYNC)
		return

	def _cleanUp(self, memHandle):
		# TODO: Not needed I guess... leaving it here for the information for now
		s.Fg_stopAcquireEx(self.device, self.device_handle, memHandle, s.STOP_SYNC)
		err = s.Fg_FreeMemEx(self.device, memHandle)
		return err




if __name__ == '__main__':
	import ctypes
	logging.basicConfig(level=logging.DEBUG)
	Camera.listDevices()
	MicroEnable4 = Camera(0)

	MicroEnable4.setExposureMicrons(10000)
	MicroEnable4.setTriggerMode('in')
	# MicroEnable4.setTriggerMode('off')

	MicroEnable4.prepareRecording(5)

	imgs = MicroEnable4.record()

	for img in imgs:
		cv2.imshow('img_test', img)
		cv2.waitKey(0)

	# MicroEnable4._liveView()

	# img = MicroEnable4.getImage()
	# cv2.imshow("sd", img)
	# cv2.waitKey(0)

	# MicroEnable4._prepareBuffer(0)
	# MicroEnable4.setExposureMicrons(5000)
	# time.sleep(4)
	# MicroEnable4.setExposureMicrons(50000)



	# from matplotlib import pyplot as plt
	# plt.imshow(img)
	# plt.show()
	#
	# img = MicroEnable4.getImage()
	# plt.imshow(img)
	# plt.show()

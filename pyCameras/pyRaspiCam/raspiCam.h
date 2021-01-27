


#ifndef RASPICAM_H
#define RASPICAM_H

#include "./bcm2835/bcm2835-1.50/src/bcm2835.h"
#include  "interface/mmal/mmal.h"
#include "Python.h"
#define FLASH_OFF   -1


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

int initialize(int format, int flash, int waitFlash, int binning);

int getImage(unsigned char* seq, int n1, int n2, int n3, unsigned int shutter, int delay);

PyObject* getImageBuffer(unsigned char* seq, int len, unsigned int shutter);

int finalize(void);

void setFlash(int flash);

/// Frame advance method
#define FRAME_NEXT_SINGLE        0
#define FRAME_NEXT_TIMELAPSE     1
#define FRAME_NEXT_KEYPRESS      2
#define FRAME_NEXT_FOREVER       3
#define FRAME_NEXT_GPIO          4
#define FRAME_NEXT_SIGNAL        5
#define FRAME_NEXT_IMMEDIATELY   6

#define FORMAT_RAW			0
#define FORMAT_RGB			1
#define FORMAT_YUV_Y			2

#endif // RASPICAM_H

# pyCameras

This repository contains a selection of Python Camera implementations for different cameras used at the IMR.
The idea is that all cameras follow the same interface and are therefore easily interchangeable.
The interface is defined in `pyCameras/cameraTemplate.py`.
All new camera implementations should inherit from the `CameraTemplate` base class and implement it's abstract functions (those that raise `NotImplementedError`).

## Description of the interface

The camera interface allows the following function calls:

| function | description |
| -------- | ----------- |
| listDevices | List devices that can be opened with the implemented camera object |
| openDevice | Actually open the device connection to the camera |
| closeDevice | Close the connection to the camera |
| getImage | Get a single image from the camera |
| getImages | Get a number of images from the camera |
| prepareRecording | Prepare the camera to record a number of images (call record afterwards) |
| record | Record the previously announced number of images (see prepareRecording) |
| grabStart | Start recording images (non-blocking) and store the images in an internal variable (see grabStop) |
| grabStop | Stop recording images (see grabStart) and return the images that have been recorded |
| getFeature | Get the value for the passed feature keyword |
| setFeature | Set the value for the passed feature keyword |
| listFeatures | Return a list of all registered feature keywords |
| getFeatures | Return a dictionary of feature registrations (keywords and their corresponding callback function) |
| registerFeature | Register a new feature keyword to a callback function |
| setExposureMicrons | Set the exposure time to the passed value in microns. This function is available under the feature keyword 'Exposure', 'ExposureTime', or 'ExposureMicrons' |
| setGain | Set the gain of the camera to the passed value. This function is availbale under the feature keyword 'Gain' |
| setFormat | Set the image format of the camera to the passed value (e.g. 'Mono8' or 'Mono10'. This function is available under the feature keyword 'Format' |
| setTriggerMode | Set the trigger mode to either 'in', 'out', or 'off depending on passed value. This function is available under the feature keyword 'TriggerMode' |

## Implemented camera interfaces

The following camera interfaces are available in this repository.
Since this project only defines a common interface for the cameras, the actual interaction with the camera ususally relies on other python packages and APIs.
These are listed along with the corresponding interface.

### AVT Cameras

The AVT camera interface uses the [pymba](https://github.com/morefigs/pymba.git) backend.

**Installation**
Clone or download pymba from the given link. Install the [Vimba API](https://www.alliedvision.com/de/produkte/software.html)
from AVT (the installation guide can be found inside the archive under `/[Vimba_Version]/Documenation/ReleaseNotes.txt`)

Install pymba by moving into the pymba folder and run
```bash
pip install . [--user]
```
Add `--user` to apply the installation only to the current user. 

### Basler Cameras

The Basler camera interface uses a forked version of the [PyPylon](https://gitlab.imr.uni-hannover.de/kroeger/PyPylon) backend.

### USB Cameras

The generic USB camera interface uses [openCV](https://opencv.org/) as backend.
Due to the wide variety of USB cameras some functionality might not work as expected with all devices.
If possible new interfaces for specific manufacturers should be implemented to guarantee better functionality.

### JAI Camera

Bla this is my manual
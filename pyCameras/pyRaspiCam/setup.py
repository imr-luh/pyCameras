'''
Created on 14.01.2015

@author: schlobohm
'''

from distutils.core import setup, Extension
import subprocess

import numpy as np

if subprocess.call(("rm -rf build/"), shell=True) != 0:
    print("No Old files Found")

if subprocess.call(("mkdir build"), shell=True) != 0:
    print("Compilation Error!")

if subprocess.call(("cd build/"), shell=True) != 0:
    print("Compilation Error!")

if subprocess.call(("cmake ./interface/mmal/"), shell=True) != 0:
    print("Compilation Error!")

if subprocess.call(("make ."), shell=True) != 0:
    print("Compilation Error!")

if subprocess.call(["swig", "-python", "raspiCam.i"]) != 0:
    raise Exception("Error running swig")

try:
    numpy_include = np.get_include()
except AttributeError:
    numpy_include = np.get_numpy_include()

sources = ["raspiCam_wrap.c", "raspiCam.c", "./raspicam/RaspiCamControl.c", "RaspiPreview.c", "RaspiCLI.c"]
lib_dirs = ["/opt/vc/lib"]
# libraries = ["mmal_components", "mmal_core", "mmal_omx", "mmal_omxutil", "mmal", "mmal_util", "mmal_util", 
#              "mmal_vc_client","vcos","vcsm","vcilcs", "vchostif", "vchiq_arm", "vmcs_rpc_client", "vcfiled_check", "vcilcs",
#              "bcm_host", "brcmjpeg","containers","debug_sym_static","EGL_static","GLESv2_static","khrn_client","khrn_static","openmaxil","OpenVG","WFC"]
libraries = ["mmal_components", "mmal_core", "mmal", "mmal_util", "mmal_util",
             "mmal_vc_client", "vcos", "vcsm", "vcilcs", "vchostif", "vchiq_arm", "vmcs_rpc_client", "vcfiled_check",
             "vcilcs",
             "bcm_host", "containers", "debug_sym_static", "EGL_static", "GLESv2_static", "khrn_client", "khrn_static",
             "openmaxil", "OpenVG", "WFC", "bcm2835"]

reader_writerlibs = ["reader_asf", "reader_avi", "reader_binary", "reader_flv", "reader_metatdata_id3", "reader_mkv",
                     "reader_mp4", "reader_mpga", "reader_ps", "reader_qsynth", "reader_raw_video", "reader_rcv",
                     "reader_rtp", "reader_rtsp", "reader_rv9", "reader_simple", "reader_wav", "writer_asf",
                     "writer_avi", "writer_binary", "writer_dummy", "writer_mp4", "writer_raw_video", "writer_simple"]
include_dirs = ["./", "/opt/vc/include/", "./interface/vcos/pthreads/", "./interface/vmcs_host/linux/", "./raspicam/", "./bcm2835/", "./vcinclude/", "./interface/", "./interface/mmal/"]
# include_dirs = []


module1 = Extension('_raspiCam',
                    sources=sources,
                    library_dirs=lib_dirs,
                    libraries=libraries,
                    include_dirs=include_dirs,
                    extra_compile_args=["-fopenmp"],
                    extra_link_args=["-fopenmp"])

setup(script_args=['build'],
      name='_raspiCam',
      version='1.0',
      description='wrapper for RaspiCam grayscale',
      ext_modules=[module1],
      include_dirs=include_dirs)

if subprocess.call(["cp", "build/lib.linux-armv7l-2.7/_raspiCam.so", "./"]) != 0:
    raise Exception("Error compiling")

print("SUCCESS")

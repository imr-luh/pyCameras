#!/usr/bin/env python3
import numpy as np
import v4l2
import fcntl
import mmap
import select
import time
import cv2
import matplotlib.pyplot as plt
import colour_demosaicing
import skimage.util
import pyunraw
# import colorcorrect.algorithm as cca
from colorcorrect.util import from_pil, to_pil
from OV2740_UVC_Camera.video_helper_fct import stretch

def getFrames(Frames, exposure):
    exp = True
    vd = open('/dev/video0', 'rb+', buffering=0)

    print(">> get device capabilities")
    cp = v4l2.v4l2_capability()
    fcntl.ioctl(vd, v4l2.VIDIOC_QUERYCAP, cp)

    print("Driver:", "".join((chr(c) for c in cp.driver)))
    print("Name:", "".join((chr(c) for c in cp.card)))
    print("Is a video capture device?", bool(cp.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE))
    print("Supports read() call?", bool(cp.capabilities & v4l2.V4L2_CAP_READWRITE))
    print("Supports streaming?", bool(cp.capabilities & v4l2.V4L2_CAP_STREAMING))

    print(">> device setup")
    fmt = v4l2.v4l2_format()
    fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    fcntl.ioctl(vd, v4l2.VIDIOC_G_FMT, fmt)  # get current settings
    print("width:", fmt.fmt.pix.width, "height", fmt.fmt.pix.height)
    print("pxfmt:",
          "V4L2_PIX_FMT_YUYV" if fmt.fmt.pix.pixelformat == v4l2.V4L2_PIX_FMT_YUYV else fmt.fmt.pix.pixelformat)
    print("bytesperline:", fmt.fmt.pix.bytesperline)
    print("sizeimage:", fmt.fmt.pix.sizeimage)
    fcntl.ioctl(vd, v4l2.VIDIOC_S_FMT, fmt)

    # set exposure time here ...
    if exp:
        # query control
        qc = v4l2.v4l2_queryctrl()
        qc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
        fcntl.ioctl(vd, v4l2.VIDIOC_QUERYCTRL, qc)
        print("exposure defalut: ", qc.default)

        # get control value
        gc = v4l2.v4l2_control()
        gc.id = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
        fcntl.ioctl(vd, v4l2.VIDIOC_G_CTRL, gc)
        print("exposure_curr", gc.value)

        # set control value
        gc.value = exposure
        fcntl.ioctl(vd, v4l2.VIDIOC_S_CTRL, gc)
        print("exposure set to: ", gc.value)

        # get control value
        fcntl.ioctl(vd, v4l2.VIDIOC_G_CTRL, gc)
        print("exposure is", gc.value)

    print(">>> streamparam")  ## somewhere in here you can set the camera framerate
    parm = v4l2.v4l2_streamparm()
    parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    parm.parm.capture.capability = v4l2.V4L2_CAP_TIMEPERFRAME
    fcntl.ioctl(vd, v4l2.VIDIOC_G_PARM, parm)
    fcntl.ioctl(vd, v4l2.VIDIOC_S_PARM, parm)  # just got with the defaults

    print(">> init mmap capture")
    req = v4l2.v4l2_requestbuffers()
    req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    req.memory = v4l2.V4L2_MEMORY_MMAP
    req.count = 1  # nr of buffer frames
    fcntl.ioctl(vd, v4l2.VIDIOC_REQBUFS, req)  # tell the driver that we want some buffers
    print("req.count", req.count)

    buffers = []

    print(">>> VIDIOC_QUERYBUF, mmap, VIDIOC_QBUF")
    for ind in range(req.count):
        # setup a buffer
        buf = v4l2.v4l2_buffer()
        buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = v4l2.V4L2_MEMORY_MMAP
        buf.index = ind
        fcntl.ioctl(vd, v4l2.VIDIOC_QUERYBUF, buf)

        mm = mmap.mmap(vd.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
        buffers.append(mm)

        # queue the buffer for capture
        fcntl.ioctl(vd, v4l2.VIDIOC_QBUF, buf)

    print(">> Start streaming")
    buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
    fcntl.ioctl(vd, v4l2.VIDIOC_STREAMON, buf_type)

    print(">> Capture image")
    t0 = time.time()
    max_t = 1
    ready_to_read, ready_to_write, in_error = ([], [], [])
    print(">>> select")
    while len(ready_to_read) == 0 and time.time() - t0 < max_t:
        ready_to_read, ready_to_write, in_error = select.select([vd], [], [], max_t)

    print(">>> download buffers")
    index = 1
    buf = v4l2.v4l2_buffer()
    buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = v4l2.V4L2_MEMORY_MMAP

    start = time.time()

    for i in range(Frames):  # capture 50 frames
        if index == 1:
            print("empyt frame passed")
            fcntl.ioctl(vd, v4l2.VIDIOC_DQBUF, buf)  # get image from the driver queue
            fcntl.ioctl(vd, v4l2.VIDIOC_QBUF, buf)  # request new image
            print(f"Frame : {index}")
        else:
            print(f"Frame : {index}")

            fcntl.ioctl(vd, v4l2.VIDIOC_DQBUF, buf)  # get image from the driver queue
            # print("buf.index", buf.index)

            mm = buffers[buf.index]

            image_bytestream = mm.read()
            mm.seek(0)
            fcntl.ioctl(vd, v4l2.VIDIOC_QBUF, buf) # request new image

            image_bytearray = bytearray(image_bytestream)

            # image_struct = struct.unpack('>'+'H'*(1928*1088), image_bytes)

            image_array = np.ndarray(shape=(1088, 1928), dtype='>u2', buffer=image_bytearray).astype(np.uint16)
            image = np.right_shift(image_array, 6).astype(np.uint16)




            # LibRaw Test:
            # proc = libraw.LibRaw()  # create RAW processor
            # proc.imgdata = image_array_right_shift
            # proc.open_file(image_array_right_shift)  # open file
            # proc.unpack()  # extract mosaic from file
            # mosaic = proc.imgdata.rawdata.raw_image

            # AndoridCamera Test:

            # cap = {"width": 1928,
            #        "height": 1088,
            #        "format": "raw",
            #        }
            # props = {}
            # r, gr, gb, b = convert_capture_to_planes(cap, image_array_right_shift, props)
            # # image = convert_raw_to_rgb_image(r, gr, gb, b, props, cap_res=None)
            # h = r.shape[0]
            # w = r.shape[1]
            # image = np.dstack([b, (gr + gb) / 2.0, r])
            #
            # norm_img = np.float64(image / 1024.0)

            # image = np.int16(image)
            # image = np.ndarray([r[:,:,0], (gb[:,:,0] + gr[:,:,0]) / 2, b[:,:,0]])
            # img = (((img.reshape(h, w, 3) - black_levels) * scale) * gains).clip(0.0, 1.0)
            # img = numpy.dot(img.reshape(w * h, 3), ccm.T).reshape(h, w, 3).clip(0.0, 1.0)

            # image = image_array_right_shift.reshape(1088, 1928)

            # with rawpy.imread(img) as raw:
            #     rgb = raw.postprocess()
            # imageio.imsave('default.tiff', rgb)


        index += 1
    stop= time.time()
    print(">> Stop streaming t:", stop-start)
    fcntl.ioctl(vd, v4l2.VIDIOC_STREAMOFF, buf_type)
    # vid.close()
    vd.close()
    return image


if __name__ == "__main__":
    Frames = 86
    img = getFrames(Frames, exposure=800)
    print(img.shape)
    print(img.dtype)
    img[0::2, 0::2] = np.multiply(img[0::2, 0::2], 1.8)
    img[1::2, 1::2] = np.multiply(img[1::2, 1::2], 1.7)
    # img[0::2, 0::2] *= 1.969 # Blue
    # img[1::2, 1::2] *= 1.707 # Red
    # img = img.astype(np.uint16)

    # cv2.imwrite("OV2740_UVC_Camera/RawBilder/bayer_rightshifted.tiff", img)
    # np.save('OV2740_UVC_Camera/NumpyRaw/frame_testpattern2', img)
    plt.imshow(img, cmap="gray", vmin=0, vmax=1024)
    plt.show()



    # result = cv2.cvtColor(img_norm, cv2.COLOR_BGR2LAB)
    # avg_a = np.average(result[:, :, 1])
    # avg_b = np.average(result[:, :, 2])
    # result[:, :, 1] = result[:, :, 1] - ((avg_a - 0.5) * (result[:, :, 0] / 1.0) * 1.1)
    # result[:, :, 2] = result[:, :, 2] - ((avg_b - 0.5) * (result[:, :, 0] / 1.0) * 1.1)
    # img_whitebalanced = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)


    # demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_bilinear(img, "BGGR")
    demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_DDFAPD(img, "BGGR")
    # demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_Malvar2004(img, "BGGR")
    # demosaic_img = colour_demosaicing.demosaicing_CFA_Bayer_Menon2007(img, "BGGR")
    # demosaic_norm = np.dstack([demosaic_img[:,:,0] / np.max(demosaic_img[:,:,0]), demosaic_img[:,:,1] / np.max(demosaic_img[:,:,1]),demosaic_img[:,:,2] / np.max(demosaic_img[:,:,2])])

    demosaic_norm = demosaic_img.copy() / np.max(demosaic_img)
    img = demosaic_img.copy().astype(np.uint16)
    img2 = demosaic_norm.copy()

    # img_cc = stretch(img)

    # images = np.concatenate((demosaic_norm, demosaic_img), axis=1)
    plt.imshow(demosaic_norm)
    plt.show()

    # img_cc_norm = img_cc.copy() / np.max(img_cc)

    # plt.imshow(img_cc_norm)
    # plt.show()



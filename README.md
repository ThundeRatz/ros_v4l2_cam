# ROS v4l2_cam

**Under development**

High performance video capture using Video4Linux2.

## About

This package is aimed at video capture with fewer latency and CPU consumption than other ROS packages.

The [nodelet API](http://wiki.ros.org/nodelet) is supported and its usage is recommended for zero copy image transport. This package does not support frame decoding; instead, it publishes images at the capture's native colorspace. The main reason for this choice is that it supports the nodelet API and publishing messages incurs almost no cost. If colorspace conversion is desired, a nodelet can be added to convert the output of this package. On the other hand, you might save processing if the capture is already in a desired format (eg. MPEG capture and you need a lower bandwidth, compressed transmission).

mmap access is the supported IO method; it can be used on USB cameras and is the fastest available. YUYV to RGB conversion and frame resizing are supported.

## Usage

TODO

## Parameters

TODO
Should be supported:
* device
* brightness
* contrast
* saturation
* sharpness
* width
* height
* framerate
* pixel_format
* autofocus
* focus
* autoexposure
* exposure
* gain
* auto_white_balance
* white_balance

## Alternatives

For similar packages with frame decoding and some more features, see:

* [bosch-ros-pkg/usb_cam](https://github.com/bosch-ros-pkg/usb_cam)
* [ktossell/camera_umd](https://github.com/ktossell/camera_umd) (deprecated, but it is one of the few with nodelet support)
* [ktossell/libuvc_ros](https://github.com/ktossell/libuvc_ros)

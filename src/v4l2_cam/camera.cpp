/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 ThundeRatz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

#include "v4l2_cam/camera.h"
#include "v4l2_cam/safe_ioctl.h"


namespace
{
// Conversions taken from:
// https://github.com/philips/libv4l/blob/master/libv4lconvert/rgbyuv.c#L84
inline uint8_t crop(int x)
{
  return x > 255 ? 255 : x < 0 ? 0 : x;
}

inline uint8_t yCr2r(int y, int cr)
{
  return crop(y + (((cr - 128) * 1436) >> 10));
}

inline uint8_t yCbCr2g(int y, int u, int v)
{
  return crop(y - (((u - 128) * 352 + (v - 128) * 731) >> 10));
}

inline uint8_t yCb2b(int y, int cb)
{
  return crop(y + (((cb - 128) * 1814) >> 10));
}

void perror_quit(const std::string& msg)
{
  perror(msg.c_str());
  exit(-1);
}
}  // namespace

namespace v4l2_cam
{
const std::string ctrl_type[] =
{
  "INVALID", "INTEGER", "BOOLEAN", "MENU", "INTEGER_MENU", "BITMASK", "BUTTON", "INTEGER64", "STRING", "CTRL_CLASS",
  "U8", "U16", "U32"
};

Camera::Camera(const std::string& device)
{
  fd_ = open(device.c_str(), O_RDWR);
  if (-1 == fd_)
    perror_quit("open");

  struct v4l2_capability cap;
  safe_ioctl(fd_, VIDIOC_QUERYCAP, &cap);
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    ROS_ERROR("Device does not support video capture");
    exit(-1);
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING))
  {
    ROS_ERROR("Device does not support streaming I/O");
    exit(-1);
  }

  v4l2_priority priority = V4L2_PRIORITY_RECORD;
  safe_ioctl(fd_, VIDIOC_S_PRIORITY, &priority);

  list_controls();
  list_formats();
  set_format("YUYV", 640, 480);
  set_framerate(1, 10);
  setup_transfer();
}

Camera::~Camera()
{
  for (auto &x : buffers)
    munmap(x.start, x.length);
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  safe_ioctl(fd_, VIDIOC_STREAMOFF, &type);
}

void Camera::set_format(const std::string& fourcc, unsigned width, unsigned height)
{
  ROS_ASSERT(fourcc.size() == 4);

  struct v4l2_format format = {};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = width;
  format.fmt.pix.height = height;
  format.fmt.pix.pixelformat = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
  format.fmt.pix.field = V4L2_FIELD_INTERLACED;

  safe_ioctl(fd_, VIDIOC_S_FMT, &format);
  if (format.fmt.pix.width != width || format.fmt.pix.height != height)
    ROS_WARN_STREAM("Driver rejected " << width << "x" << height << " and used "
                    << format.fmt.pix.width << "x" << format.fmt.pix.height << " instead");
  this->width = format.fmt.pix.width;
  this->height = format.fmt.pix.height;
  bytesperline = format.fmt.pix.bytesperline;
}

void Camera::set_framerate(unsigned numerator, unsigned denominator)
{
  struct v4l2_streamparm streamparm = {};
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  safe_ioctl(fd_, VIDIOC_G_PARM, &streamparm);
  if (!(streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME))
  {
    ROS_WARN("V4L2_CAP_TIMEPERFRAME not supported by driver, leaving frame rate unchanged");
    return;
  }
  streamparm.parm.capture.timeperframe.numerator = numerator;
  streamparm.parm.capture.timeperframe.denominator = denominator;
  safe_ioctl(fd_, VIDIOC_S_PARM, &streamparm);
  if (streamparm.parm.capture.timeperframe.numerator != numerator ||
      streamparm.parm.capture.timeperframe.denominator != denominator)
  {
    ROS_WARN_STREAM("Driver rejected " <<numerator << "/" << denominator << "s per frame and used "
                    << streamparm.parm.capture.timeperframe.numerator << "/"
                    << streamparm.parm.capture.timeperframe.denominator << "s instead");
  }
}

void Camera::setup_transfer()
{
  struct v4l2_requestbuffers reqbuf = {};
  reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count = 2;

  safe_ioctl(fd_, VIDIOC_REQBUFS, &reqbuf);
  ROS_ASSERT(reqbuf.count >= 2);

  buffers.resize(reqbuf.count);

  for (unsigned i = 0; i < reqbuf.count; i++)
  {
    struct v4l2_buffer buffer = {};

    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;

    safe_ioctl(fd_, VIDIOC_QUERYBUF, &buffer);

    buffers[i].length = buffer.length;
    buffers[i].start = static_cast<uint8_t *>(mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
                                                   MAP_SHARED, fd_, buffer.m.offset));

    if (MAP_FAILED == buffers[i].start)
      perror_quit("mmap");
  }

  for (unsigned i = 0; i < reqbuf.count; i++)
    enqueue(i);

  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  safe_ioctl(fd_, VIDIOC_STREAMON, &type);
}

const boost::shared_ptr<Camera::V4L2Image> Camera::capture()
{
  struct v4l2_buffer buffer = {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  safe_ioctl(fd_, VIDIOC_DQBUF, &buffer);
  ROS_ASSERT(buffer.flags & V4L2_BUF_FLAG_MAPPED);
  ROS_ASSERT(!(buffer.flags & V4L2_BUF_FLAG_DONE));
  if (buffer.flags & V4L2_BUF_FLAG_ERROR)
  {
    ROS_WARN("Camera driver notified buffer error, ignoring frame");
    return boost::shared_ptr<V4L2Image>();
  }
  boost::shared_ptr<V4L2Image> image = empty_image(buffer.index);
  uint8_t *frame = buffers[buffer.index].start;

  // Linear interpolation
  int row_pos = 0;
  float x_scale = static_cast<float>(width - 1) / (448 - 1);
  float y_scale = static_cast<float>(height - 1) / (448 - 1);
  ROS_ASSERT(width <= 2 * 448);
  ROS_ASSERT(width > 448);
  ROS_ASSERT(!(448 & 1));
  for (unsigned r = 0; r != height; r++)
  {
    for (unsigned c = 0;; c++)
    {
      if (c == 448 - 1)
      {
        int y = frame[row_pos + 2 * 659], cr = frame[row_pos + 2 * 659 - 1], cb = frame[row_pos + 2 * 659 + 1];
        image->data.emplace_back(yCr2r(y, cr));
        image->data.emplace_back(yCbCr2g(y, cb, cr));
        image->data.emplace_back(yCb2b(y, cb));
        break;
      }

      float source_x = c * x_scale;
      int x = static_cast<int>(source_x);
      float dx = source_x - x;
      int r1, g1, b1, r2, g2, b2;
      if (x & 1)
      {
        int y1 = frame[row_pos + 2 * x], cb1 = frame[row_pos + 2 * x - 1], cr1 = frame[row_pos + 2 * x + 1];
        int y2 = frame[row_pos + 2 * x + 2], cb2 = frame[row_pos + 2 * x + 3], cr2 = frame[row_pos + 2 * x + 5];
        r1 = yCr2r(y1, cr1), g1 = yCbCr2g(y1, cb1, cr1), b1 = yCb2b(y1, cb1);
        r2 = yCr2r(y2, cr2), g2 = yCbCr2g(y2, cb2, cr2), b2 = yCb2b(y2, cb2);
      }
      else
      {
        int y1 = frame[row_pos + 2 * x], cb = frame[row_pos + 2 * x + 1];
        int y2 = frame[row_pos + 2 * x + 2], cr = frame[row_pos + 2 * x + 3];
        r1 = yCr2r(y1, cr), g1 = yCbCr2g(y1, cb, cr), b1 = yCb2b(y1, cb);
        r2 = yCr2r(y2, cr), g2 = yCbCr2g(y2, cb, cr), b2 = yCb2b(y2, cb);
      }
      image->data.emplace_back((1 - dx) * r1 + dx * r2);
      image->data.emplace_back((1 - dx) * g1 + dx * g2);
      image->data.emplace_back((1 - dx) * b1 + dx * b2);
    }

    row_pos += bytesperline;
  }
  row_pos = 0;
  for (unsigned r = 0; r != 448; r++)
  {
    if (r == 448 - 1)
      for (unsigned c = 0; c != 448; c++)
      {
        image->data[row_pos + 3 * c] = image->data[3 * (480 - 1) * 448 + 3 * c];
        image->data[row_pos + 3 * c + 1] = image->data[3 * (480 - 1) * 448 + 3 * c + 1];
        image->data[row_pos + 3 * c + 2] = image->data[3 * (480 - 1) * 448 + 3 * c + 2];
      }
    else
    {
      float source_y = r * y_scale;
      int y = static_cast<int>(source_y);
      float dy = source_y - y;
      for (int c = 0; c != 448; c++)
      {
        image->data[row_pos + 3 * c] = (1 - dy) * image->data[3 * y * 448 + 3 * c] +
                                       dy * image->data[3 * (y + 1) * 448 + 3 * c];
        image->data[row_pos + 3 * c + 1] = (1 - dy) * image->data[3 * y * 448 + 3 * c + 1] +
                                           dy * image->data[3 * (y + 1) * 448 + 3 * c + 1];
        image->data[row_pos + 3 * c + 2] = (1 - dy) * image->data[3 * y * 448 + 3 * c + 2] +
                                           dy * image->data[3 * (y + 1) * 448 + 3 * c + 2];
      }
    }
    row_pos += 3 * 448;
  }

  image->data.resize(3 * 448 * 448);
  return image;
}

boost::shared_ptr<Camera::V4L2Image> Camera::empty_image(int buffer_index)
{
  boost::shared_ptr<V4L2Image> image(new V4L2Image(*this, buffer_index));
  image->width = 448;
  image->height = 448;
  image->encoding = sensor_msgs::image_encodings::RGB8;
  image->step = 3 * 448;
  image->data.reserve(3 * height * 448);  // height * output_width pixels will be used during interpolation
  return image;
}

void Camera::enqueue(int index)
{
  struct v4l2_buffer buffer = {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.index = index;
  safe_ioctl(fd_, VIDIOC_QBUF, &buffer);
}

void Camera::list_controls()
{
  struct v4l2_query_ext_ctrl query_ext_ctrl = {};

  ROS_INFO("Camera controls:");
  query_ext_ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  while (!print_control(&query_ext_ctrl))
    query_ext_ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
}

int Camera::print_control(struct v4l2_query_ext_ctrl *query_ext_ctrl)
{
  if (ioctl(fd_, VIDIOC_QUERY_EXT_CTRL, query_ext_ctrl))
  {
    if (errno == EINVAL)
      return EINVAL;
    perror_quit("VIDIOC_QUERY_EXT_CTRL");
  }
  if (query_ext_ctrl->flags & V4L2_CTRL_FLAG_DISABLED)
    return 0;

  if (query_ext_ctrl->type == V4L2_CTRL_TYPE_BOOLEAN || query_ext_ctrl->type == V4L2_CTRL_TYPE_BUTTON)
    ROS_INFO_STREAM("" << query_ext_ctrl->name << " [" << ctrl_type[query_ext_ctrl->type]
                    << "] - Power-on default " << query_ext_ctrl->default_value);
  else if (query_ext_ctrl->step > 1)
    ROS_INFO_STREAM("" << query_ext_ctrl->name << " [" << ctrl_type[query_ext_ctrl->type] << " - "
                    << query_ext_ctrl->minimum << ".." << query_ext_ctrl->maximum << " with " << query_ext_ctrl->step
                    << " step] - Power-on default " << query_ext_ctrl->default_value);
  else
    ROS_INFO_STREAM("" << query_ext_ctrl->name << " [" << ctrl_type[query_ext_ctrl->type] << " - "
                    << query_ext_ctrl->minimum << ".." << query_ext_ctrl->maximum << "] - Power-on default "
                    << query_ext_ctrl->default_value);

  if (query_ext_ctrl->type == V4L2_CTRL_TYPE_MENU)
    enumerate_menu(query_ext_ctrl);
  else if (query_ext_ctrl->type == V4L2_CTRL_TYPE_INTEGER_MENU)
    enumerate_integer_menu(query_ext_ctrl);

  return 0;
}

void Camera::enumerate_menu(const struct v4l2_query_ext_ctrl *query_ext_ctrl)
{
  struct v4l2_querymenu querymenu = {};
  querymenu.id = query_ext_ctrl->id;
  for (querymenu.index = query_ext_ctrl->minimum; querymenu.index <= query_ext_ctrl->maximum; querymenu.index++)
    if (!query_menu(&querymenu))
      ROS_INFO_STREAM("\t" << querymenu.index << " - " << querymenu.name);
}

void Camera::enumerate_integer_menu(const struct v4l2_query_ext_ctrl *query_ext_ctrl)
{
  struct v4l2_querymenu querymenu = {};
  querymenu.id = query_ext_ctrl->id;
  for (querymenu.index = query_ext_ctrl->minimum; querymenu.index <= query_ext_ctrl->maximum; querymenu.index++)
    if (!query_menu(&querymenu))
      ROS_INFO_STREAM("\t" << querymenu.index << " - " << querymenu.value);
}

int Camera::query_menu(struct v4l2_querymenu *querymenu)
{
  if (ioctl(fd_, VIDIOC_QUERYMENU, querymenu))
  {
    if (errno != EINVAL)
      perror_quit("VIDIOC_QUERYMENU");
    return EINVAL;
  }
  return 0;
}

void Camera::list_formats()
{
  struct v4l2_fmtdesc fmtdesc = {};

  ROS_INFO("Camera formats:");
  fmtdesc.index = 0;
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (!print_format(&fmtdesc))
    fmtdesc.index++;
}

int Camera::print_format(struct v4l2_fmtdesc *fmtdesc)
{
  if (ioctl(fd_, VIDIOC_ENUM_FMT, fmtdesc))
  {
    if (errno != EINVAL)
      perror_quit("VIDIOC_ENUM_FMT");
    return EINVAL;
  }
  ROS_INFO("%c%c%c%c (%s)", fmtdesc->pixelformat, fmtdesc->pixelformat >> 8, fmtdesc->pixelformat >> 16,
           fmtdesc->pixelformat >> 24, fmtdesc->description);
  if (fmtdesc->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    ROS_INFO("\tCapture is multi-plane and not supported");
  if (fmtdesc->flags & V4L2_FMT_FLAG_COMPRESSED)
    ROS_INFO("\tCompressed");
  if (fmtdesc->flags & V4L2_FMT_FLAG_EMULATED)
    ROS_INFO("\tEmulated compression");

  struct v4l2_format format;
  ioctl(fd_, VIDIOC_G_FMT, &format);

  return 0;
}

}  // namespace v4l2_cam

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

#ifndef V4L2_CAM_SAFE_IOCTL_H
#define V4L2_CAM_SAFE_IOCTL_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#ifndef TEMP_FAILURE_RETRY
// Taken from Android source code
#define TEMP_FAILURE_RETRY(exp) ({       \
  typeof(exp) _rc;                       \
  do                                     \
  {                                      \
    _rc = (exp);                         \
  }                                      \
  while (_rc == -1 && errno == EINTR);   \
  _rc;                                   \
})
#endif

#define safe_ioctl(fd, request, ...) do {                         \
  if (TEMP_FAILURE_RETRY(ioctl(fd, request, __VA_ARGS__)) == -1)  \
  {                                                               \
    perror(#request); close(fd); exit(-1);                        \
  }                                                               \
} while (0)

#endif  // V4L2_CAM_SAFE_IOCTL_H

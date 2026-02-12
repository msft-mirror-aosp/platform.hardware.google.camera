/*
 * Copyright (C) 2026 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "VideoFrameSource"
#define ATRACE_TAG ATRACE_TAG_CAMERA

#include "VideoFrameSource.h"

#include <android/hardware/graphics/common/1.2/types.h>
#include <fcntl.h>
#include <libyuv.h>
#include <log/log.h>
#include <media/NdkMediaFormat.h>
#include <media/stagefright/MediaCodecConstants.h>
#include <sys/types.h>
#include <unistd.h>
#include <utils/Trace.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "utils/HWLUtils.h"

namespace android {
namespace framesource {

using namespace std::chrono_literals;

struct MediaFormatDeleter {
  void operator()(AMediaFormat* f) {
    AMediaFormat_delete(f);
  }
};
using MediaFormatPtr = std::unique_ptr<AMediaFormat, MediaFormatDeleter>;

VideoFrameSource::VideoFrameSource(const LogicalCharacteristics& chars,
                                   uint32_t /*camera_id*/, std::string file_path)
    : chars_(std::make_unique<LogicalCharacteristics>(chars)),
      file_path_(file_path) {
}

VideoFrameSource::~VideoFrameSource() {
  if (codec_) {
    AMediaCodec_stop(codec_);
    AMediaCodec_delete(codec_);
  }
  if (extractor_) {
    AMediaExtractor_delete(extractor_);
  }
  if (video_fd_ >= 0) {
    close(video_fd_);
    video_fd_ = -1;
  }
}

status_t VideoFrameSource::Initialize() {
  video_fd_ = open(file_path_.c_str(), O_RDONLY);
  if (video_fd_ < 0) {
    ALOGE("%s: Failed to open video file %s", __FUNCTION__, file_path_.c_str());
    return BAD_VALUE;
  }

  extractor_ = AMediaExtractor_new();
  off64_t fileSize = lseek64(video_fd_, 0, SEEK_END);
  lseek64(video_fd_, 0, SEEK_SET);
  media_status_t status =
      AMediaExtractor_setDataSourceFd(extractor_, video_fd_, 0, fileSize);

  if (status != AMEDIA_OK) {
    ALOGE("%s: Failed to set data source", __FUNCTION__);
    return BAD_VALUE;
  }

  if (!InitializeMediaCodec()) {
    return BAD_VALUE;
  }

  if (AMediaCodec_start(codec_) != AMEDIA_OK) {
    ALOGI(
        "%s: Received error in starting decoder. "
        "Trying again after resetting this emulated device.",
        __FUNCTION__);

    if (!InitializeMediaCodec()) {
      ALOGE("%s: Failed to re-configure the media codec.", __FUNCTION__);
      return BAD_VALUE;
    }

    AMediaExtractor_seekTo(extractor_, 0, AMEDIAEXTRACTOR_SEEK_CLOSEST_SYNC);

    if (auto status = AMediaCodec_start(codec_); status != AMEDIA_OK) {
      ALOGE("%s: Received error again in starting decoder. Error code: %d",
            __FUNCTION__, status);
      return BAD_VALUE;
    }
  }

  return OK;
}

bool VideoFrameSource::InitializeMediaCodec() {
  if (codec_) {
    AMediaCodec_stop(codec_);
    AMediaCodec_delete(codec_);
    codec_ = nullptr;
  }

  int numTracks = AMediaExtractor_getTrackCount(extractor_);
  int selectedTrack = -1;
  const char* mime = nullptr;
  MediaFormatPtr format;

  for (int i = 0; i < numTracks; ++i) {
    format.reset(AMediaExtractor_getTrackFormat(extractor_, i));
    AMediaFormat_getString(format.get(), AMEDIAFORMAT_KEY_MIME, &mime);
    if (strncmp(mime, "video/", 6) == 0) {
      selectedTrack = i;
      break;
    }
  }

  if (selectedTrack < 0 || !format) {
    ALOGE("%s: No video track found", __FUNCTION__);
    return false;
  }

  media_status_t status = AMediaExtractor_selectTrack(extractor_, selectedTrack);
  if (status != AMEDIA_OK) {
    ALOGE("%s: Failed to select video track: %d", __FUNCTION__, status);
    return false;
  }

  codec_ = AMediaCodec_createDecoderByType(mime);
  if (!codec_) {
    ALOGE("%s: Failed to create decoder for %s", __FUNCTION__, mime);
    return false;
  }

  AMediaFormat_setInt32(format.get(), AMEDIAFORMAT_KEY_COLOR_FORMAT,
                        COLOR_FormatYUV420Planar);

  status = AMediaCodec_configure(codec_, format.get(), nullptr, nullptr, 0);
  if (status != AMEDIA_OK) {
    ALOGE("%s: Failed to configure codec", __FUNCTION__);
    return false;
  }

  MediaFormatPtr outFormat(AMediaCodec_getOutputFormat(codec_));
  AMediaFormat_getInt32(outFormat.get(), AMEDIAFORMAT_KEY_WIDTH, &video_width_);
  AMediaFormat_getInt32(outFormat.get(), AMEDIAFORMAT_KEY_HEIGHT,
                        &video_height_);
  AMediaFormat_getInt32(outFormat.get(), AMEDIAFORMAT_KEY_COLOR_FORMAT,
                        &video_color_format_);
  AMediaFormat_getInt32(outFormat.get(), AMEDIAFORMAT_KEY_STRIDE,
                        &video_stride_);
  AMediaFormat_getInt32(outFormat.get(), AMEDIAFORMAT_KEY_SLICE_HEIGHT,
                        &video_slice_height_);

  return true;
}

int VideoFrameSource::GetNextDecodedFrame(AMediaCodecBufferInfo* out_info) {
  int retries = 0;
  const int kMaxRetries = 600;  // Try for ~600ms total
  const int kTimeoutUs = 1000;  // 1ms

  while (retries < kMaxRetries) {
    // Feed input until full (match EVS behavior)
    while (true) {
      ssize_t bufIdx = AMediaCodec_dequeueInputBuffer(codec_, 0);
      if (bufIdx < 0) {
        break;
      }

      size_t bufSize;
      uint8_t* buf = AMediaCodec_getInputBuffer(codec_, bufIdx, &bufSize);
      ssize_t sampleSize = AMediaExtractor_getSampleSize(extractor_);
      if (sampleSize > (ssize_t)bufSize) {
        ALOGE("%s: Input buffer too small for sample size %zd > %zu",
              __FUNCTION__, sampleSize, bufSize);
        return -1;
      }
      sampleSize = AMediaExtractor_readSampleData(extractor_, buf, bufSize);
      int64_t timeUs = AMediaExtractor_getSampleTime(extractor_);

      if (sampleSize < 0) {
        // EOF, queue EOS flag
        AMediaCodec_queueInputBuffer(codec_, bufIdx, 0, 0, timeUs,
                                     AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM);
        ALOGI("%s: End of stream reached, queued EOS", __FUNCTION__);
      } else {
        AMediaCodec_queueInputBuffer(codec_, bufIdx, 0, sampleSize, timeUs, 0);
        AMediaExtractor_advance(extractor_);
      }
    }

    // Check output
    ssize_t outBufIdx =
        AMediaCodec_dequeueOutputBuffer(codec_, out_info, kTimeoutUs);
    if (outBufIdx >= 0) {
      return outBufIdx;
    } else if (outBufIdx == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
      MediaFormatPtr newFormat(AMediaCodec_getOutputFormat(codec_));
      AMediaFormat_getInt32(newFormat.get(), AMEDIAFORMAT_KEY_WIDTH,
                            &video_width_);
      AMediaFormat_getInt32(newFormat.get(), AMEDIAFORMAT_KEY_HEIGHT,
                            &video_height_);
      AMediaFormat_getInt32(newFormat.get(), AMEDIAFORMAT_KEY_STRIDE,
                            &video_stride_);
      AMediaFormat_getInt32(newFormat.get(), AMEDIAFORMAT_KEY_SLICE_HEIGHT,
                            &video_slice_height_);
      AMediaFormat_getInt32(newFormat.get(), AMEDIAFORMAT_KEY_COLOR_FORMAT,
                            &video_color_format_);
      ALOGI("%s: Format changed: %dx%d, stride %d, slice %d, color %d",
            __FUNCTION__, video_width_, video_height_, video_stride_,
            video_slice_height_, video_color_format_);
      // EVS logs/returns on format change, effectively retrying. We retry loop.
    } else if (outBufIdx == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
      retries++;
    } else {
      ALOGE("%s: AMediaCodec_dequeueOutputBuffer failed: %zd", __FUNCTION__,
            outBufIdx);
      return -1;
    }
  }
  ALOGE("%s: Timed out waiting for decoded frame", __FUNCTION__);
  return -1;
}

status_t VideoFrameSource::GetFrameForTimestamp(nsecs_t timestamp,
                                                uint8_t** buffer,
                                                size_t* buffer_size,
                                                AMediaCodecBufferInfo* out_info) {
  // Check if we can reuse the cached frame
  if (timestamp == last_request_timestamp_ && !cached_frame_.empty()) {
    *buffer = cached_frame_.data();
    *buffer_size = cached_frame_.size();
    *out_info = cached_info_;
    return OK;
  }

  // Decode new frame
  size_t outSize = 0;
  AMediaCodecBufferInfo info = {};
  int bufIdx = GetNextDecodedFrame(&info);

  if (bufIdx < 0) {
    ALOGE("%s: Failed to decode frame", __FUNCTION__);
    return UNKNOWN_ERROR;
  }

  bool loop_reset = ((info.flags & AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM) != 0);

  if (info.size > 0) {
    uint8_t* codec_out = AMediaCodec_getOutputBuffer(codec_, bufIdx, &outSize);
    codec_out += info.offset;

    // Update cache
    if (cached_frame_.size() < static_cast<size_t>(info.size)) {
      cached_frame_.resize(info.size);
    }
    memcpy(cached_frame_.data(), codec_out, info.size);
    cached_info_ = info;
    last_request_timestamp_ = timestamp;

    *buffer = cached_frame_.data();
    *buffer_size = cached_frame_.size();
    *out_info = cached_info_;
  } else {
    // Empty frame (likely EOS without data).
    *buffer = nullptr;
    *buffer_size = 0;
    *out_info = info;
  }

  AMediaCodec_releaseOutputBuffer(codec_, bufIdx, false);

  if (loop_reset) {
    Reset();
    // If the EOS frame had no data, reuse the last cached frame to avoid
    // returning a null buffer (which causes black/green frames).
    if (*buffer == nullptr && !cached_frame_.empty()) {
      *buffer = cached_frame_.data();
      *buffer_size = cached_frame_.size();
      *out_info = cached_info_;
    }
  }

  return OK;
}

void VideoFrameSource::Reset() {
  ALOGI("Start video playback from the beginning.");
  AMediaExtractor_seekTo(extractor_, 0, AMEDIAEXTRACTOR_SEEK_CLOSEST_SYNC);
  AMediaCodec_flush(codec_);
}

status_t VideoFrameSource::ProduceFrame(uint32_t /*camera_id*/,
                                        nsecs_t timestamp,
                                        const SensorSettings& /*settings*/,
                                        SensorBuffer* buffer,
                                        const SensorBuffer* /*input_buffer*/) {
  ATRACE_CALL();
  if (buffer == nullptr) return BAD_VALUE;

  uint8_t* src_ptr = nullptr;
  size_t src_size = 0;
  AMediaCodecBufferInfo info = {};

  status_t res = GetFrameForTimestamp(timestamp, &src_ptr, &src_size, &info);
  if (res != OK) {
    return res;
  }

  if (src_ptr != nullptr && info.size > 0) {
    CopyFrame(src_ptr, info.size, info, buffer);
  }

  return OK;
}

uint8_t* VideoFrameSource::GetBufferFromFrame(uint8_t* src, int src_width,
                                              int src_height, int dst_width,
                                              int dst_height, int* dst_stride_y,
                                              int* dst_stride_u,
                                              int* dst_stride_v) {
  // Assume default stride if not set
  int stride = (video_stride_ > 0) ? video_stride_ : src_width;
  int slice_height =
      (video_slice_height_ > 0) ? video_slice_height_ : src_height;

  size_t y_size = stride * slice_height;

  uint8_t* src_y = src;
  uint8_t* src_u = nullptr;
  uint8_t* src_v = nullptr;
  int src_stride_y = stride;
  int src_stride_u = 0;
  int src_stride_v = 0;

  // Identify format (approximate parity with EVS which uses ConfigManager)
  switch (video_color_format_) {
    case COLOR_FormatYUV420SemiPlanar:  // NV12
      src_u = src + y_size;
      src_v = nullptr;  // Not used for NV12ToABGR
      src_stride_u = stride;
      src_stride_v = stride;
      break;
    // Note: EVS doesn't strictly check COLOR_Format values in the loop,
    // it relies on ConfigManager. We assume standard mappings here.
    case COLOR_FormatYUV420Planar:  // I420 or YV12
    default:
      // Assume I420/YV12 as default for software decoders
      src_u = src + y_size;
      src_v = src + y_size + (stride / 2 * slice_height / 2);
      src_stride_u = stride / 2;
      src_stride_v = stride / 2;
      break;
  }

  // If dimensions match, return directly
  if (src_width == dst_width && src_height == dst_height) {
    *dst_stride_y = src_stride_y;
    *dst_stride_u = src_stride_u;
    *dst_stride_v = src_stride_v;
    return src;
  }

  // Scaling logic if dimensions mismatch
  size_t required_size = dst_width * dst_height * 3 / 2;
  if (scaled_buffer_.size() < required_size) {
    scaled_buffer_.resize(required_size);
  }

  // Calculate Aspect Ratio and Crop Region
  int crop_x = 0;
  int crop_y = 0;
  int crop_w = src_width;
  int crop_h = src_height;

  float video_ar = (float)src_width / src_height;
  float dst_ar = (float)dst_width / dst_height;

  // Allow a small epsilon for float comparison
  if (std::abs(video_ar - dst_ar) > 0.001f) {
    if (video_ar > dst_ar) {
      // Video is wider than Dest: Crop Left/Right
      crop_w = (int)(src_height * dst_ar);
      crop_x = (src_width - crop_w) / 2;
    } else {
      // Video is taller than Dest: Crop Top/Bottom
      crop_h = (int)(src_width / dst_ar);
      crop_y = (src_height - crop_h) / 2;
    }
  }

  // Ensure crop dimensions are even to maintain color plane alignment
  crop_x &= ~1;
  crop_y &= ~1;
  crop_w &= ~1;
  crop_h &= ~1;

  uint8_t* dst_y = scaled_buffer_.data();
  uint8_t* dst_u = nullptr;
  uint8_t* dst_v = nullptr;
  *dst_stride_y = dst_width;
  *dst_stride_u = 0;
  *dst_stride_v = 0;

  if (video_color_format_ == COLOR_FormatYUV420SemiPlanar) {
    // NV12 Scale
    // Apply crop offset to source pointers
    const uint8_t* src_y_crop = src_y + (crop_y * src_stride_y) + crop_x;
    const uint8_t* src_u_crop = src_u + ((crop_y / 2) * src_stride_u) + crop_x;

    *dst_stride_u = dst_width;
    dst_u = dst_y + dst_width * dst_height;

    libyuv::NV12Scale(src_y_crop, src_stride_y, src_u_crop, src_stride_u,
                      crop_w, crop_h, dst_y, *dst_stride_y, dst_u,
                      *dst_stride_u, dst_width, dst_height, libyuv::kFilterBox);
  } else {
    // I420 Scale
    // Apply crop offset to source pointers
    const uint8_t* src_y_crop = src_y + (crop_y * src_stride_y) + crop_x;
    const uint8_t* src_u_crop =
        src_u + ((crop_y / 2) * src_stride_u) + (crop_x / 2);
    const uint8_t* src_v_crop =
        src_v + ((crop_y / 2) * src_stride_v) + (crop_x / 2);

    *dst_stride_u = dst_width / 2;
    *dst_stride_v = dst_width / 2;
    dst_u = dst_y + dst_width * dst_height;
    dst_v = dst_u + (dst_width / 2) * (dst_height / 2);

    libyuv::I420Scale(src_y_crop, src_stride_y, src_u_crop, src_stride_u,
                      src_v_crop, src_stride_v, crop_w, crop_h, dst_y,
                      *dst_stride_y, dst_u, *dst_stride_u, dst_v, *dst_stride_v,
                      dst_width, dst_height, libyuv::kFilterBox);
  }

  return scaled_buffer_.data();
}

void VideoFrameSource::CopyFrame(uint8_t* src, size_t /*src_size*/,
                                 AMediaCodecBufferInfo /*info*/,
                                 SensorBuffer* buffer) {
  int src_stride_y, src_stride_u, src_stride_v;
  uint8_t* src_buffer = GetBufferFromFrame(
      src, video_width_, video_height_, buffer->width, buffer->height,
      &src_stride_y, &src_stride_u, &src_stride_v);

  // Set up plane pointers for the (possibly scaled) source buffer
  uint8_t* src_y = src_buffer;
  uint8_t* src_u = nullptr;
  uint8_t* src_v = nullptr;

  if (video_color_format_ == COLOR_FormatYUV420SemiPlanar) {
    // Check if we returned the original src or scaled buffer
    if (src_buffer == src) {
      // Original buffer logic
      int stride = (video_stride_ > 0) ? video_stride_ : video_width_;
      int slice_height =
          (video_slice_height_ > 0) ? video_slice_height_ : video_height_;
      size_t y_size = stride * slice_height;

      src_u = src_y + y_size;
      // src_v not used for NV12
    } else {
      // Scaled buffer logic
      // Scaled buffer is packed.
      size_t y_size = buffer->width * buffer->height;
      if (video_color_format_ == COLOR_FormatYUV420SemiPlanar) {
        src_u = src_y + y_size;
      } else {
        src_u = src_y + y_size;
        src_v = src_u + (buffer->width / 2) * (buffer->height / 2);
      }
    }
  } else {
    // I420
    if (src_buffer == src) {
      int stride = (video_stride_ > 0) ? video_stride_ : video_width_;
      int slice_height =
          (video_slice_height_ > 0) ? video_slice_height_ : video_height_;
      size_t y_size = stride * slice_height;
      src_u = src_y + y_size;
      src_v = src_y + y_size + (stride / 2 * slice_height / 2);
    } else {
      size_t y_size = buffer->width * buffer->height;
      src_u = src_y + y_size;
      src_v = src_u + (buffer->width / 2) * (buffer->height / 2);
    }
  }

  // NOTE: EVS does NOT scale. We assume buffer dimensions match video.
  // If they don't, libyuv will handle the region of interest defined by width/height.

  if (buffer->format == PixelFormat::RGBA_8888) {
    if (video_color_format_ == COLOR_FormatYUV420SemiPlanar) {
      // NV12
      libyuv::NV12ToABGR(
          src_y, src_stride_y, src_u, src_stride_u, buffer->plane.img.img,
          buffer->plane.img.stride_in_bytes, buffer->width, buffer->height);
    } else {
      // I420
      libyuv::I420ToABGR(src_y, src_stride_y, src_u, src_stride_u, src_v,
                         src_stride_v, buffer->plane.img.img,
                         buffer->plane.img.stride_in_bytes, buffer->width,
                         buffer->height);
    }
  } else if (buffer->format == PixelFormat::RGB_888) {
    if (video_color_format_ == COLOR_FormatYUV420SemiPlanar) {
      // NV12
      libyuv::NV12ToRGB24(
          src_y, src_stride_y, src_u, src_stride_u, buffer->plane.img.img,
          buffer->plane.img.stride_in_bytes, buffer->width, buffer->height);
    } else {
      // I420
      libyuv::I420ToRGB24(src_y, src_stride_y, src_u, src_stride_u, src_v,
                          src_stride_v, buffer->plane.img.img,
                          buffer->plane.img.stride_in_bytes, buffer->width,
                          buffer->height);
    }
  } else if (buffer->format == PixelFormat::YCBCR_420_888 ||
             buffer->format == PixelFormat::YCRCB_420_SP) {
    // Copy/Convert to I420 or NV12/NV21 destination
    auto& planes = buffer->plane.img_y_crcb;
    bool dst_is_nv21 = (planes.cbcr_step == 2 && planes.img_cr < planes.img_cb);
    bool dst_is_nv12 = (planes.cbcr_step == 2 && planes.img_cb < planes.img_cr);

    if (video_color_format_ == COLOR_FormatYUV420SemiPlanar) {
      // Source NV12
      if (dst_is_nv12) {
        // Dest NV12 - Copy
        libyuv::CopyPlane(src_y, src_stride_y, planes.img_y, planes.y_stride,
                          buffer->width, buffer->height);
        libyuv::CopyPlane(src_u, src_stride_u, planes.img_cb,
                          planes.cbcr_stride, buffer->width, buffer->height / 2);
      } else if (dst_is_nv21) {
        // Dest NV21 - Swap UV (NV12 -> NV21)
        libyuv::CopyPlane(src_y, src_stride_y, planes.img_y, planes.y_stride,
                          buffer->width, buffer->height);
        libyuv::SwapUVPlane(src_u, src_stride_u, planes.img_cr,
                            planes.cbcr_stride, buffer->width,
                            buffer->height / 2);
      } else {
        // Dest I420 - Split
        libyuv::NV12ToI420(src_y, src_stride_y, src_u, src_stride_u,
                           planes.img_y, planes.y_stride, planes.img_cb,
                           planes.cbcr_stride, planes.img_cr,
                           planes.cbcr_stride, buffer->width, buffer->height);
      }
    } else {
      // Source I420
      if (dst_is_nv12) {
        // Dest NV12 - Merge
        libyuv::I420ToNV12(src_y, src_stride_y, src_u, src_stride_u, src_v,
                           src_stride_v, planes.img_y, planes.y_stride,
                           planes.img_cb, planes.cbcr_stride, buffer->width,
                           buffer->height);
      } else if (dst_is_nv21) {
        // Dest NV21 - Merge (Swap U/V args)
        libyuv::I420ToNV21(src_y, src_stride_y, src_u, src_stride_u, src_v,
                           src_stride_v, planes.img_y, planes.y_stride,
                           planes.img_cr, planes.cbcr_stride, buffer->width,
                           buffer->height);
      } else {
        // Dest I420 - Copy
        libyuv::I420Copy(src_y, src_stride_y, src_u, src_stride_u, src_v,
                         src_stride_v, planes.img_y, planes.y_stride,
                         planes.img_cb, planes.cbcr_stride, planes.img_cr,
                         planes.cbcr_stride, buffer->width, buffer->height);
      }
    }
  }
}

status_t VideoFrameSource::RenderYUV420(uint32_t /*camera_id*/,
                                        nsecs_t timestamp,
                                        const SensorSettings& /*settings*/,
                                        const YUV420Frame& output_frame,
                                        const YUV420Frame* /*input_frame*/) {
  ATRACE_CALL();

  uint8_t* src_ptr = nullptr;
  size_t src_size = 0;
  AMediaCodecBufferInfo info = {};

  status_t res = GetFrameForTimestamp(timestamp, &src_ptr, &src_size, &info);
  if (res != OK) {
    return res;
  }

  if (src_ptr != nullptr && info.size > 0) {
    int src_stride_y, src_stride_u, src_stride_v;
    uint8_t* src_buffer = GetBufferFromFrame(
        src_ptr, video_width_, video_height_, output_frame.width,
        output_frame.height, &src_stride_y, &src_stride_u, &src_stride_v);

    // Set up plane pointers for the (possibly scaled) source buffer
    uint8_t* src_y = src_buffer;
    uint8_t* src_u = nullptr;
    uint8_t* src_v = nullptr;

    if (src_buffer == src_ptr) {
      int stride = (video_stride_ > 0) ? video_stride_ : video_width_;
      int slice_height =
          (video_slice_height_ > 0) ? video_slice_height_ : video_height_;
      size_t y_size = stride * slice_height;

      src_u = src_y + y_size;
      if (video_color_format_ == COLOR_FormatYUV420Planar) {
        src_v = src_y + y_size + (stride / 2 * slice_height / 2);
      }
    } else {
      size_t y_size = output_frame.width * output_frame.height;
      src_u = src_y + y_size;
      if (video_color_format_ == COLOR_FormatYUV420Planar) {
        src_v = src_u + (output_frame.width / 2) * (output_frame.height / 2);
      }
    }

    YCbCrPlanes dst_planes = output_frame.planes;
    bool dst_is_nv21 =
        (dst_planes.cbcr_step == 2 && dst_planes.img_cr < dst_planes.img_cb);
    bool dst_is_nv12 =
        (dst_planes.cbcr_step == 2 && dst_planes.img_cb < dst_planes.img_cr);

    // Direct copy/convert (no scaling)
    if (video_color_format_ == COLOR_FormatYUV420SemiPlanar) {
      if (dst_is_nv12) {
        libyuv::CopyPlane(src_y, src_stride_y, dst_planes.img_y,
                          dst_planes.y_stride, output_frame.width,
                          output_frame.height);
        libyuv::CopyPlane(src_u, src_stride_u, dst_planes.img_cb,
                          dst_planes.cbcr_stride, output_frame.width,
                          output_frame.height / 2);
      } else if (dst_is_nv21) {
        libyuv::CopyPlane(src_y, src_stride_y, dst_planes.img_y,
                          dst_planes.y_stride, output_frame.width,
                          output_frame.height);
        libyuv::SwapUVPlane(src_u, src_stride_u, dst_planes.img_cr,
                            dst_planes.cbcr_stride, output_frame.width,
                            output_frame.height / 2);
      } else {
        libyuv::NV12ToI420(src_y, src_stride_y, src_u, src_stride_u,
                           dst_planes.img_y, dst_planes.y_stride,
                           dst_planes.img_cb, dst_planes.cbcr_stride,
                           dst_planes.img_cr, dst_planes.cbcr_stride,
                           output_frame.width, output_frame.height);
      }
    } else {
      if (dst_is_nv12) {
        libyuv::I420ToNV12(src_y, src_stride_y, src_u, src_stride_u, src_v,
                           src_stride_v, dst_planes.img_y, dst_planes.y_stride,
                           dst_planes.img_cb, dst_planes.cbcr_stride,
                           output_frame.width, output_frame.height);
      } else if (dst_is_nv21) {
        libyuv::I420ToNV21(src_y, src_stride_y, src_u, src_stride_u, src_v,
                           src_stride_v, dst_planes.img_y, dst_planes.y_stride,
                           dst_planes.img_cr, dst_planes.cbcr_stride,
                           output_frame.width, output_frame.height);
      } else {
        libyuv::I420Copy(src_y, src_stride_y, src_u, src_stride_u, src_v,
                         src_stride_v, dst_planes.img_y, dst_planes.y_stride,
                         dst_planes.img_cb, dst_planes.cbcr_stride,
                         dst_planes.img_cr, dst_planes.cbcr_stride,
                         output_frame.width, output_frame.height);
      }
    }
  }

  return OK;
}

void VideoFrameSource::CalculateAndAppendNoiseProfile(
    float /*gain*/, float /*base_gain_factor*/, HalCameraMetadata* result) {
  // Zero noise for video
  double noise_profile[8] = {0.0};
  result->Set(ANDROID_SENSOR_NOISE_PROFILE, noise_profile, 8);
}

float VideoFrameSource::GetBaseGainFactor(float /*max_raw_value*/) const {
  return 1.0f;
}

bool VideoFrameSource::HasBinningInfo(uint32_t /*camera_id*/) const {
  return false;
}

BinningState VideoFrameSource::GetBinningState(uint32_t /*camera_id*/) const {
  return BinningState();
}

void VideoFrameSource::ResetSensorBinningInfo() {
}

}  // namespace framesource
}  // namespace android

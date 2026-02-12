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

#ifndef HW_EMULATOR_CAMERA2_VIDEO_FRAME_SOURCE_H
#define HW_EMULATOR_CAMERA2_VIDEO_FRAME_SOURCE_H

#include <android/hardware/graphics/common/1.2/types.h>
#include <media/NdkMediaCodec.h>
#include <media/NdkMediaExtractor.h>
#include <system/camera_metadata.h>
#include <utils/Errors.h>

#include <memory>
#include <string>
#include <vector>

#include "Base.h"
#include "IFrameSource.h"
#include "SensorCharacteristics.h"

namespace android {
namespace framesource {

using google_camera_hal::HalCameraMetadata;

class VideoFrameSource : public IFrameSource {
 public:
  VideoFrameSource(const LogicalCharacteristics& chars, uint32_t camera_id,
                   const std::string& file_path);
  virtual ~VideoFrameSource();

  status_t Initialize();

  // IFrameSource Implementation
  status_t ProduceFrame(uint32_t camera_id, nsecs_t timestamp,
                        const SensorSettings& settings, SensorBuffer* buffer,
                        const SensorBuffer* input_buffer) override;

  void CalculateAndAppendNoiseProfile(float gain, float max_raw_value,
                                      HalCameraMetadata* result) override;

 private:
  // Decodes one frame from the video stream. Returns the buffer index or error code.
  int GetNextDecodedFrame(AMediaCodecBufferInfo* out_info);

  // Helper to fill the destination buffer from decoder output
  // Uses GetBufferFromFrame internally to handle scaling/cropping if needed.
  void CopyFrame(uint8_t* src, size_t src_size, AMediaCodecBufferInfo info,
                 SensorBuffer* buffer);

  // Helper to get a buffer of the desired dimensions from the source frame.
  // Performs scaling/cropping if src and dst dimensions do not match.
  // Returns a pointer to the buffer (either src or scaled_buffer_).
  // Updates dst_stride_y/u/v with the strides of the returned buffer.
  uint8_t* GetBufferFromFrame(uint8_t* src, int src_width, int src_height,
                              int dst_width, int dst_height, int* dst_stride_y,
                              int* dst_stride_u, int* dst_stride_v);

  // Retrieves a decoded frame for the given timestamp.
  // Handles caching: returns cached frame if timestamp matches last request.
  // Otherwise decodes a new frame.
  // Returns OK and populates *buffer and *info on success.
  status_t GetFrameForTimestamp(nsecs_t timestamp, uint8_t** buffer,
                                size_t* buffer_size,
                                AMediaCodecBufferInfo* info);

  bool InitializeMediaCodec();
  void Reset();

  std::unique_ptr<LogicalCharacteristics> chars_;
  std::string file_path_;
  int video_fd_ = -1;

  AMediaExtractor* extractor_ = nullptr;
  AMediaCodec* codec_ = nullptr;

  int video_width_ = 0;
  int video_height_ = 0;
  int video_stride_ = 0;
  int video_slice_height_ = 0;
  int32_t video_color_format_ = 0;

  std::vector<uint8_t> scaled_buffer_;

  // Caching for multi-stream requests
  nsecs_t last_request_timestamp_ = -1;
  std::vector<uint8_t> cached_frame_;
  AMediaCodecBufferInfo cached_info_ = {};
};

}  // namespace framesource
}  // namespace android

#endif  // HW_EMULATOR_CAMERA2_VIDEO_FRAME_SOURCE_H

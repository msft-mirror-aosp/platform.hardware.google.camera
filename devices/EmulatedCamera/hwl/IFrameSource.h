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

#ifndef HW_EMULATOR_CAMERA2_IFRAME_SOURCE_H
#define HW_EMULATOR_CAMERA2_IFRAME_SOURCE_H

#include <system/camera_metadata.h>
#include <utils/Errors.h>
#include <utils/Timers.h>

#include "Base.h"
#include "SensorCharacteristics.h"

namespace android {
namespace framesource {

using google_camera_hal::HalCameraMetadata;

struct BinningState {
  bool raw_binning_factor_used = false;
  bool raw_in_sensor_zoom_applied = false;
  bool has_cropped_raw_stream = false;
};

// Struct used for YUV processing (shared with JpegCompressor usage)
struct YUV420Frame {
  uint32_t width = 0;
  uint32_t height = 0;
  YCbCrPlanes planes;
  const uint8_t* output_buffer = nullptr;
  size_t output_buffer_size = 0;
  const uint8_t* app1_buffer = nullptr;
  size_t app1_buffer_size = 0;
  int32_t color_space = 0;
};

class IFrameSource {
 public:
  virtual ~IFrameSource() = default;

  // Main entry point for producing a frame into a sensor buffer
  virtual status_t ProduceFrame(uint32_t camera_id, nsecs_t timestamp,
                                const SensorSettings& settings,
                                SensorBuffer* buffer,
                                const SensorBuffer* input_buffer) = 0;

  // Helper for JPEG compression (renders directly to memory)
  virtual status_t RenderYUV420(uint32_t camera_id, nsecs_t timestamp,
                                const SensorSettings& settings,
                                const YUV420Frame& output_frame,
                                const YUV420Frame* input_frame) = 0;

  virtual void CalculateAndAppendNoiseProfile(
      float gain /*in ISO*/, float base_gain_factor,
      HalCameraMetadata* result /*out*/) = 0;

  virtual float GetBaseGainFactor(float max_raw_value) const = 0;

  virtual bool HasBinningInfo(uint32_t camera_id) const = 0;
  virtual BinningState GetBinningState(uint32_t camera_id) const = 0;
  virtual void ResetSensorBinningInfo() = 0;
};

}  // namespace framesource
}  // namespace android

#endif  // HW_EMULATOR_CAMERA2_IFRAME_SOURCE_H

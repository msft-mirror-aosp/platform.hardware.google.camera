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

#ifndef HW_EMULATOR_CAMERA2_COLOR_BAR_FRAME_SOURCE_H
#define HW_EMULATOR_CAMERA2_COLOR_BAR_FRAME_SOURCE_H

#include <android/hardware/graphics/common/1.2/types.h>
#include <system/camera_metadata.h>
#include <utils/Errors.h>

#include <array>

#include "Base.h"
#include "IFrameSource.h"

namespace android {
namespace framesource {

using google_camera_hal::HalCameraMetadata;

class ColorBarFrameSource : public IFrameSource {
 public:
  ColorBarFrameSource();
  ~ColorBarFrameSource() override = default;

  // IFrameSource Implementation
  status_t ProduceFrame(uint32_t camera_id, nsecs_t timestamp,
                        const SensorSettings& settings, SensorBuffer* buffer,
                        const SensorBuffer* input_buffer) override;

  void CalculateAndAppendNoiseProfile(float gain, float max_raw_value,
                                      HalCameraMetadata* result) override;

 private:
  struct BarColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t y;
    uint8_t cb;
    uint8_t cr;
  };

  static const size_t kNumBars = 8;

  void InitializeColors();

  // Drawing helpers
  void DrawYUV420(const YUV420Frame& frame);
  void DrawRGB(uint8_t* img, uint32_t width, uint32_t height, uint32_t stride,
               int bytes_per_pixel, bool is_rgba);

  std::array<BarColor, kNumBars> colors_;
};

}  // namespace framesource
}  // namespace android

#endif  // HW_EMULATOR_CAMERA2_COLOR_BAR_FRAME_SOURCE_H

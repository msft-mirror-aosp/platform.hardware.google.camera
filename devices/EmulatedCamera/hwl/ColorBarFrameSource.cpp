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

#define LOG_TAG "ColorBarFrameSource"
#define ATRACE_TAG ATRACE_TAG_CAMERA

#include "ColorBarFrameSource.h"

#include <android/hardware/graphics/common/1.2/types.h>
#include <log/log.h>
#include <utils/Trace.h>

#include <algorithm>
#include <cmath>

#include "utils/HWLUtils.h"

namespace android {
namespace framesource {

ColorBarFrameSource::ColorBarFrameSource() {
  InitializeColors();
}

void ColorBarFrameSource::InitializeColors() {
  // Standard 100% Color Bars
  // Order: White, Yellow, Cyan, Green, Magenta, Red, Blue, Black
  struct {
    uint8_t r, g, b;
  } rgb_colors[kNumBars] = {
      {255, 255, 255},  // White
      {255, 255, 0},    // Yellow
      {0, 255, 255},    // Cyan
      {0, 255, 0},      // Green
      {255, 0, 255},    // Magenta
      {255, 0, 0},      // Red
      {0, 0, 255},      // Blue
      {0, 0, 0}         // Black
  };

  for (size_t i = 0; i < kNumBars; ++i) {
    colors_[i].r = rgb_colors[i].r;
    colors_[i].g = rgb_colors[i].g;
    colors_[i].b = rgb_colors[i].b;

    colors_[i].y = static_cast<uint8_t>(
        (0.299 * colors_[i].r + 0.587 * colors_[i].g + 0.114 * colors_[i].b));
    colors_[i].cb = static_cast<uint8_t>(
        128 +
        (-0.1687 * colors_[i].r - 0.3313 * colors_[i].g + 0.5 * colors_[i].b));
    colors_[i].cr =
        static_cast<uint8_t>(128 + (0.5 * colors_[i].r - 0.4187 * colors_[i].g -
                                    0.0813 * colors_[i].b));
  }
}

status_t ColorBarFrameSource::ProduceFrame(uint32_t /*camera_id*/,
                                           nsecs_t /*timestamp*/,
                                           const SensorSettings& /*settings*/,
                                           SensorBuffer* buffer,
                                           const SensorBuffer* /*input_buffer*/) {
  ATRACE_CALL();
  if (buffer == nullptr) {
    return BAD_VALUE;
  }

  switch (buffer->format) {
    case PixelFormat::RGB_888:
      DrawRGB(buffer->plane.img.img, buffer->width, buffer->height,
              buffer->plane.img.stride_in_bytes, 3, false);
      break;
    case PixelFormat::RGBA_8888:
      DrawRGB(buffer->plane.img.img, buffer->width, buffer->height,
              buffer->plane.img.stride_in_bytes, 4, true);
      break;
    case PixelFormat::YCRCB_420_SP:
    case PixelFormat::YCBCR_420_888: {
      YUV420Frame output_frame;
      output_frame.width = buffer->width;
      output_frame.height = buffer->height;
      output_frame.planes = buffer->plane.img_y_crcb;
      DrawYUV420(output_frame);
      break;
    }
    default:
      ALOGE("%s: Unknown format %x", __FUNCTION__, buffer->format);
      return BAD_VALUE;
  }

  return OK;
}

void ColorBarFrameSource::DrawYUV420(const YUV420Frame& frame) {
  uint32_t width = frame.width;
  uint32_t height = frame.height;
  auto& planes = frame.planes;

  for (uint32_t y = 0; y < height; ++y) {
    uint8_t* row_y = planes.img_y + y * planes.y_stride;
    uint8_t* row_cb = planes.img_cb + (y / 2) * planes.cbcr_stride;
    uint8_t* row_cr = planes.img_cr + (y / 2) * planes.cbcr_stride;

    for (uint32_t b = 0; b < kNumBars; ++b) {
      uint32_t start_x = (b * width) / kNumBars;
      uint32_t end_x = ((b + 1) * width) / kNumBars;
      start_x &= ~1;
      end_x &= ~1;
      if (b == kNumBars - 1) end_x = width;

      const auto& color = colors_[b];

      uint16_t y_val = color.y;
      uint16_t cb_val = color.cb;
      uint16_t cr_val = color.cr;

      for (uint32_t x = start_x; x < end_x; x += 2) {
        row_y[x] = static_cast<uint8_t>(y_val);

        if (x + 1 < width) {
          row_y[x + 1] = static_cast<uint8_t>(y_val);
        }

        if ((y & 1) == 0) {
          uint8_t* cb_ptr = row_cb + (x / 2) * planes.cbcr_step;
          uint8_t* cr_ptr = row_cr + (x / 2) * planes.cbcr_step;

          *cb_ptr = static_cast<uint8_t>(cb_val);
          *cr_ptr = static_cast<uint8_t>(cr_val);
        }
      }
    }
  }
}

void ColorBarFrameSource::DrawRGB(uint8_t* img, uint32_t width, uint32_t height,
                                  uint32_t stride, int bytes_per_pixel,
                                  bool is_rgba) {
  for (uint32_t y = 0; y < height; ++y) {
    uint8_t* row = img + y * stride;

    for (uint32_t bar_idx = 0; bar_idx < kNumBars; ++bar_idx) {
      uint32_t start_x = (bar_idx * width) / kNumBars;
      uint32_t end_x = ((bar_idx + 1) * width) / kNumBars;
      if (bar_idx == kNumBars - 1) end_x = width;

      const auto& c = colors_[bar_idx];

      uint8_t r = c.r;
      uint8_t g = c.g;
      uint8_t b = c.b;

      for (uint32_t x = start_x; x < end_x; ++x) {
        uint8_t* px = row + x * bytes_per_pixel;
        // Layout is typically R, G, B, [A]
        px[0] = r;
        px[1] = g;
        px[2] = b;
        if (is_rgba) {
          px[3] = 0xFF;  // Alpha opaque
        }
      }
    }
  }
}

void ColorBarFrameSource::CalculateAndAppendNoiseProfile(
    float /*gain*/, float /*max_raw_value*/, HalCameraMetadata* result) {
  // Use small epsilon to avoid divide-by-zero in downstream calculations
  double noise_profile[8];
  std::fill(std::begin(noise_profile), std::end(noise_profile), 1e-9);
  result->Set(ANDROID_SENSOR_NOISE_PROFILE, noise_profile, 8);
}

}  // namespace framesource
}  // namespace android

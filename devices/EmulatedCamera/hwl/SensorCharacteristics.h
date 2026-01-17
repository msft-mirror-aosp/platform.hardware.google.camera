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

#ifndef HW_EMULATOR_CAMERA2_SENSOR_CHARACTERISTICS_H
#define HW_EMULATOR_CAMERA2_SENSOR_CHARACTERISTICS_H

#include <android/hardware/graphics/common/1.2/types.h>
#include <hwl_types.h>
#include <system/camera_metadata.h>
#include <utils/Timers.h>

#include <unordered_map>
#include <unordered_set>

namespace android {

using google_camera_hal::ColorSpaceProfile;
using google_camera_hal::DynamicRangeProfile;

/*
 * Default to sRGB with D65 white point
 */
struct ColorFilterXYZ {
  float rX = 3.2406f;
  float rY = -1.5372f;
  float rZ = -0.4986f;
  float grX = -0.9689f;
  float grY = 1.8758f;
  float grZ = 0.0415f;
  float gbX = -0.9689f;
  float gbY = 1.8758f;
  float gbZ = 0.0415f;
  float bX = 0.0557f;
  float bY = -0.2040f;
  float bZ = 1.0570f;
};

struct ForwardMatrix {
  float rX = 0.4355f;
  float gX = 0.3848f;
  float bX = 0.1425f;
  float rY = 0.2216f;
  float gY = 0.7168f;
  float bY = 0.0605f;
  float rZ = 0.0137f;
  float gZ = 0.0967f;
  float bZ = 0.7139f;
};

struct RgbRgbMatrix {
  float rR;
  float gR;
  float bR;
  float rG;
  float gG;
  float bG;
  float rB;
  float gB;
  float bB;
};

typedef std::unordered_map<DynamicRangeProfile,
                           std::unordered_set<DynamicRangeProfile>>
    DynamicRangeProfileMap;

typedef std::unordered_map<
    ColorSpaceProfile,
    std::unordered_map<int, std::unordered_set<DynamicRangeProfile>>>
    ColorSpaceProfileMap;

struct SensorCharacteristics {
  size_t width = 0;
  size_t height = 0;
  size_t full_res_width = 0;
  size_t full_res_height = 0;
  nsecs_t exposure_time_range[2] = {0};
  nsecs_t frame_duration_range[2] = {0};
  int32_t sensitivity_range[2] = {0};
  camera_metadata_enum_android_sensor_info_color_filter_arrangement
      color_arangement = ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_RGGB;
  ColorFilterXYZ color_filter;
  ForwardMatrix forward_matrix;
  uint32_t max_raw_value = 0;
  uint32_t black_level_pattern[4] = {0};
  uint32_t max_raw_streams = 0;
  uint32_t max_processed_streams = 0;
  uint32_t max_stalling_streams = 0;
  uint32_t max_input_streams = 0;
  uint32_t physical_size[2] = {0};
  bool is_flash_supported = false;
  uint32_t lens_shading_map_size[2] = {0};
  uint32_t max_pipeline_depth = 0;
  uint32_t orientation = 0;
  bool is_front_facing = false;
  bool quad_bayer_sensor = false;
  bool is_10bit_dynamic_range_capable = false;
  DynamicRangeProfileMap dynamic_range_profiles;
  bool support_stream_use_case = false;
  int64_t end_valid_stream_use_case =
      ANDROID_SCALER_AVAILABLE_STREAM_USE_CASES_VIDEO_CALL;
  bool support_color_space_profiles = false;
  ColorSpaceProfileMap color_space_profiles;
  int32_t raw_crop_region_zoomed[4] = {0};
  int32_t raw_crop_region_unzoomed[4] = {0};
  int32_t timestamp_source = ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
};

// Maps logical/physical camera ids to sensor characteristics
typedef std::unordered_map<uint32_t, SensorCharacteristics> LogicalCharacteristics;

/*
 * Physical camera settings control
 */
struct SensorSettings {
  nsecs_t exposure_time = 0;
  nsecs_t frame_duration = 0;
  uint32_t gain = 0;  // ISO
  uint32_t lens_shading_map_mode;
  bool report_neutral_color_point = false;
  bool report_green_split = false;
  bool report_noise_profile = false;
  float zoom_ratio = 1.0f;
  bool report_rotate_and_crop = false;
  uint8_t rotate_and_crop = ANDROID_SCALER_ROTATE_AND_CROP_NONE;
  bool report_video_stab = false;
  uint8_t video_stab = ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF;
  bool report_edge_mode = false;
  uint8_t edge_mode = ANDROID_EDGE_MODE_OFF;
  uint8_t sensor_pixel_mode = ANDROID_SENSOR_PIXEL_MODE_DEFAULT;
  uint8_t test_pattern_mode = ANDROID_SENSOR_TEST_PATTERN_MODE_OFF;
  uint32_t test_pattern_data[4] = {0, 0, 0, 0};
  uint32_t screen_rotation = 0;
  uint32_t timestamp_source = ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
};

// Maps physical and logical camera ids to individual device settings
typedef std::unordered_map<uint32_t, SensorSettings> LogicalCameraSettings;

}  // namespace android

#endif  // HW_EMULATOR_CAMERA2_SENSOR_CHARACTERISTICS_H

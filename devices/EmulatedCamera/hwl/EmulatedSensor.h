/*
 * Copyright (C) 2012 The Android Open Source Project
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

/**
 * This class is a simple simulation of a typical CMOS cellphone imager chip,
 * which outputs 12-bit Bayer-mosaic raw images.
 *
 * Unlike most real image sensors, this one's native color space is linear sRGB.
 *
 * The sensor is abstracted as operating as a pipeline 3 stages deep;
 * conceptually, each frame to be captured goes through these three stages. The
 * processing step for the sensor is marked off by vertical sync signals, which
 * indicate the start of readout of the oldest frame. The interval between
 * processing steps depends on the frame duration of the frame currently being
 * captured. The stages are 1) configure, 2) capture, and 3) readout. During
 * configuration, the sensor's registers for settings such as exposure time,
 * frame duration, and gain are set for the next frame to be captured. In stage
 * 2, the image data for the frame is actually captured by the sensor. Finally,
 * in stage 3, the just-captured data is read out and sent to the rest of the
 * system.
 *
 * The sensor is assumed to be rolling-shutter, so low-numbered rows of the
 * sensor are exposed earlier in time than larger-numbered rows, with the time
 * offset between each row being equal to the row readout time.
 *
 * The characteristics of this sensor don't correspond to any actual sensor,
 * but are not far off typical sensors.
 *
 * Example timing diagram, with three frames:
 *  Frame 0-1: Frame duration 50 ms, exposure time 20 ms.
 *  Frame   2: Frame duration 75 ms, exposure time 65 ms.
 * Legend:
 *   C = update sensor registers for frame
 *   v = row in reset (vertical blanking interval)
 *   E = row capturing image data
 *   R = row being read out
 *   | = vertical sync signal
 *time(ms)|   0          55        105       155            230     270
 * Frame 0|   :configure : capture : readout :              :       :
 *  Row # | ..|CCCC______|_________|_________|              :       :
 *      0 |   :\          \vvvvvEEEER         \             :       :
 *    500 |   : \          \vvvvvEEEER         \            :       :
 *   1000 |   :  \          \vvvvvEEEER         \           :       :
 *   1500 |   :   \          \vvvvvEEEER         \          :       :
 *   2000 |   :    \__________\vvvvvEEEER_________\         :       :
 * Frame 1|   :           configure  capture      readout   :       :
 *  Row # |   :          |CCCC_____|_________|______________|       :
 *      0 |   :          :\         \vvvvvEEEER              \      :
 *    500 |   :          : \         \vvvvvEEEER              \     :
 *   1000 |   :          :  \         \vvvvvEEEER              \    :
 *   1500 |   :          :   \         \vvvvvEEEER              \   :
 *   2000 |   :          :    \_________\vvvvvEEEER______________\  :
 * Frame 2|   :          :          configure     capture    readout:
 *  Row # |   :          :         |CCCC_____|______________|_______|...
 *      0 |   :          :         :\         \vEEEEEEEEEEEEER       \
 *    500 |   :          :         : \         \vEEEEEEEEEEEEER       \
 *   1000 |   :          :         :  \         \vEEEEEEEEEEEEER       \
 *   1500 |   :          :         :   \         \vEEEEEEEEEEEEER       \
 *   2000 |   :          :         :    \_________\vEEEEEEEEEEEEER_______\
 */

#ifndef HW_EMULATOR_CAMERA2_SENSOR_H
#define HW_EMULATOR_CAMERA2_SENSOR_H

#include <android/hardware/graphics/common/1.2/types.h>
#include <hwl_types.h>

#include <algorithm>
#include <functional>

#include "Base.h"
#include "EmulatedFrameSource.h"
#include "JpegCompressor.h"
#include "SensorCharacteristics.h"
#include "utils/Mutex.h"
#include "utils/StreamConfigurationMap.h"
#include "utils/Thread.h"
#include "utils/Timers.h"

namespace android {

using google_camera_hal::ColorSpaceProfile;
using google_camera_hal::DynamicRangeProfile;
using google_camera_hal::HwlPipelineCallback;
using google_camera_hal::HwlPipelineResult;
using google_camera_hal::StreamConfiguration;
using google_camera_hal::StreamGroupState;

using hardware::graphics::common::V1_2::Dataspace;

class EmulatedSensor : private Thread, public virtual RefBase {
 public:
  EmulatedSensor();
  ~EmulatedSensor();

  static android_pixel_format_t OverrideFormat(
      android_pixel_format_t format, DynamicRangeProfile dynamic_range_profile) {
    switch (dynamic_range_profile) {
      case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_STANDARD:
      case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_STANDARD_SMPTE_2094_50:
        if (format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) {
          return HAL_PIXEL_FORMAT_YCBCR_420_888;
        }
        break;
      case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_HLG10:
      case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_HLG10_SMPTE_2094_50:
        if (format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) {
          return static_cast<android_pixel_format_t>(
              HAL_PIXEL_FORMAT_YCBCR_P010);
        }
        break;
      default:
        ALOGE("%s: Unsupported dynamic range profile 0x%x", __FUNCTION__,
              dynamic_range_profile);
    }

    return format;
  }

  static bool IsReprocessPathSupported(android_pixel_format_t input_format,
                                       android_pixel_format_t output_format) {
    if ((HAL_PIXEL_FORMAT_YCBCR_420_888 == input_format) &&
        ((HAL_PIXEL_FORMAT_YCBCR_420_888 == output_format) ||
         (HAL_PIXEL_FORMAT_BLOB == output_format))) {
      return true;
    }

    if (HAL_PIXEL_FORMAT_RAW16 == input_format &&
        HAL_PIXEL_FORMAT_RAW16 == output_format) {
      return true;
    }

    return false;
  }

  static bool AreCharacteristicsSupported(
      const SensorCharacteristics& characteristics);

  static bool IsStreamCombinationSupported(
      uint32_t logical_id, const StreamConfiguration& config,
      StreamConfigurationMap& map, StreamConfigurationMap& max_resolution_map,
      const PhysicalStreamConfigurationMap& physical_map,
      const PhysicalStreamConfigurationMap& physical_map_max_resolution,
      const LogicalCharacteristics& sensor_chars);

  static bool IsStreamCombinationSupported(
      uint32_t logical_id, const StreamConfiguration& config,
      StreamConfigurationMap& map,
      const PhysicalStreamConfigurationMap& physical_map,
      const LogicalCharacteristics& sensor_chars, bool is_max_res = false);

  /*
   * Power control
   */

  status_t StartUp(uint32_t logical_camera_id,
                   std::unique_ptr<LogicalCharacteristics> logical_chars);
  status_t ShutDown();

  /*
   * Physical camera settings control
   */
  using SensorSettings = android::SensorSettings;

  // Maps physical and logical camera ids to individual device settings
  using LogicalCameraSettings = android::LogicalCameraSettings;

  void SetCurrentRequest(std::unique_ptr<LogicalCameraSettings> logical_settings,
                         std::unique_ptr<HwlPipelineResult> result,
                         std::unique_ptr<HwlPipelineResult> partial_result,
                         std::unique_ptr<Buffers> input_buffers,
                         std::unique_ptr<Buffers> output_buffers);

  status_t Flush();

  /*
   * Synchronizing with sensor operation (vertical sync)
   */

  // Wait until the sensor outputs its next vertical sync signal, meaning it
  // is starting readout of its latest frame of data. Returns true if vertical
  // sync is signaled, false if the wait timed out.
  bool WaitForVSync(nsecs_t rel_time);

  static const nsecs_t kSupportedExposureTimeRange[2];
  static const nsecs_t kSupportedFrameDurationRange[2];
  static const int32_t kSupportedSensitivityRange[2];
  static const uint8_t kSupportedColorFilterArrangement;
  static const uint32_t kDefaultMaxRawValue;
  static const nsecs_t kDefaultExposureTime;
  static const int32_t kDefaultSensitivity;
  static const nsecs_t kDefaultFrameDuration;
  static const nsecs_t kReturnResultThreshod;
  static const uint32_t kDefaultBlackLevelPattern[4];
  static const camera_metadata_rational kDefaultColorTransform[9];
  static const float kDefaultColorCorrectionGains[4];
  static const float kDefaultToneMapCurveRed[4];
  static const float kDefaultToneMapCurveGreen[4];
  static const float kDefaultToneMapCurveBlue[4];
  static const uint8_t kPipelineDepth;

 private:
  /**
   * Logical characteristics
   */
  std::unique_ptr<LogicalCharacteristics> chars_;

  uint32_t logical_camera_id_ = 0;

  // Sensor sensitivity, approximate
  static const camera_metadata_rational kNeutralColorPoint[3];
  static const float kGreenSplit;

  static const uint32_t kMaxRAWStreams;
  static const uint32_t kMaxProcessedStreams;
  static const uint32_t kMaxStallingStreams;
  static const uint32_t kMaxInputStreams;
  static const uint32_t kMaxLensShadingMapSize[2];

  Mutex control_mutex_;  // Lock before accessing control parameters
  // Start of control parameters
  Condition vsync_;
  bool got_vsync_;
  std::unique_ptr<LogicalCameraSettings> current_settings_;
  std::unique_ptr<HwlPipelineResult> current_result_;
  std::unique_ptr<HwlPipelineResult> partial_result_;
  std::unique_ptr<Buffers> current_output_buffers_;
  std::unique_ptr<Buffers> current_input_buffers_;
  std::unique_ptr<JpegCompressor> jpeg_compressor_;

  // End of control parameters

  /**
   * Inherited Thread virtual overrides, and members only used by the
   * processing thread
   */
  bool threadLoop() override;

  nsecs_t next_capture_time_;
  nsecs_t next_readout_time_;

  std::unique_ptr<EmulatedFrameSource> frame_source_;

  bool WaitForVSyncLocked(nsecs_t reltime);

  void ReturnResults(HwlPipelineCallback callback,
                     std::unique_ptr<LogicalCameraSettings> settings,
                     std::unique_ptr<HwlPipelineResult> result,
                     bool reprocess_request,
                     std::unique_ptr<HwlPipelineResult> partial_result);

  static std::vector<StreamGroupState> GetStreamGroupState(
      const Buffers& output_buffers);

  nsecs_t getSystemTimeWithSource(uint32_t timestamp_source);
};

}  // namespace android

#endif  // HW_EMULATOR_CAMERA2_SENSOR_H

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

//#define LOG_NDEBUG 0
//#define LOG_NNDEBUG 0
#include "system/graphics-base-v1.1.h"
#define LOG_TAG "EmulatedSensor"
#define ATRACE_TAG ATRACE_TAG_CAMERA

#ifdef LOG_NNDEBUG
#define ALOGVV(...) ALOGV(__VA_ARGS__)
#else
#define ALOGVV(...) ((void)0)
#endif

#include <android/hardware/graphics/common/1.2/types.h>
#include <cutils/properties.h>
#include <inttypes.h>
#include <memory.h>
#include <system/camera_metadata.h>
#include <utils/Log.h>
#include <utils/Trace.h>

#include <cmath>

#include "EmulatedSensor.h"
#include "utils/ExifUtils.h"
#include "utils/HWLUtils.h"

namespace android {

using android::google_camera_hal::ErrorCode;
using google_camera_hal::ErrorMessage;
using google_camera_hal::HalCameraMetadata;
using google_camera_hal::NotifyMessage;
using google_camera_hal::ShutterMessage;

using android::hardware::graphics::common::V1_2::Dataspace;

// 1 us - 30 sec
const nsecs_t EmulatedSensor::kSupportedExposureTimeRange[2] = {1000LL,
                                                                30000000000LL};

// ~1/30 s - 30 sec
const nsecs_t EmulatedSensor::kSupportedFrameDurationRange[2] = {33331760LL,
                                                                 30000000000LL};

const int32_t EmulatedSensor::kSupportedSensitivityRange[2] = {100, 1600};
const int32_t EmulatedSensor::kDefaultSensitivity = 100;  // ISO
const nsecs_t EmulatedSensor::kDefaultExposureTime = ms2ns(15);
const nsecs_t EmulatedSensor::kDefaultFrameDuration = ms2ns(33);
// Deadline within we should return the results as soon as possible to
// avoid skewing the frame cycle due to external delays.
const nsecs_t EmulatedSensor::kReturnResultThreshod = 3 * kDefaultFrameDuration;

// Sensor defaults
const uint8_t EmulatedSensor::kSupportedColorFilterArrangement =
    ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_RGGB;
const uint32_t EmulatedSensor::kDefaultMaxRawValue = 4000;
const uint32_t EmulatedSensor::kDefaultBlackLevelPattern[4] = {1000, 1000, 1000,
                                                               1000};

const uint32_t EmulatedSensor::kMaxRAWStreams = 1;
const uint32_t EmulatedSensor::kMaxProcessedStreams = 3;
const uint32_t EmulatedSensor::kMaxStallingStreams = 2;
const uint32_t EmulatedSensor::kMaxInputStreams = 1;

const uint32_t EmulatedSensor::kMaxLensShadingMapSize[2]{64, 64};

const camera_metadata_rational EmulatedSensor::kNeutralColorPoint[3] = {
    {255, 1}, {255, 1}, {255, 1}};
const float EmulatedSensor::kGreenSplit = 1.f;  // No divergence
// Reduce memory usage by allowing only one buffer in sensor, one in jpeg
// compressor and one pending request to avoid stalls.
const uint8_t EmulatedSensor::kPipelineDepth = 3;

const camera_metadata_rational EmulatedSensor::kDefaultColorTransform[9] = {
    {1, 1}, {0, 1}, {0, 1}, {0, 1}, {1, 1}, {0, 1}, {0, 1}, {0, 1}, {1, 1}};
const float EmulatedSensor::kDefaultColorCorrectionGains[4] = {1.0f, 1.0f, 1.0f,
                                                               1.0f};

const float EmulatedSensor::kDefaultToneMapCurveRed[4] = {.0f, .0f, 1.f, 1.f};
const float EmulatedSensor::kDefaultToneMapCurveGreen[4] = {.0f, .0f, 1.f, 1.f};
const float EmulatedSensor::kDefaultToneMapCurveBlue[4] = {.0f, .0f, 1.f, 1.f};

EmulatedSensor::EmulatedSensor() : Thread(false), got_vsync_(false) {
}

EmulatedSensor::~EmulatedSensor() {
  ShutDown();
}

bool EmulatedSensor::AreCharacteristicsSupported(
    const SensorCharacteristics& characteristics) {
  if ((characteristics.width == 0) || (characteristics.height == 0)) {
    ALOGE("%s: Invalid sensor size %zux%zu", __FUNCTION__,
          characteristics.width, characteristics.height);
    return false;
  }

  if ((characteristics.full_res_width == 0) ||
      (characteristics.full_res_height == 0)) {
    ALOGE("%s: Invalid sensor full res size %zux%zu", __FUNCTION__,
          characteristics.full_res_width, characteristics.full_res_height);
    return false;
  }

  if (characteristics.is_10bit_dynamic_range_capable) {
    for (const auto& profile : characteristics.dynamic_range_profiles) {
      switch (profile.first) {
        case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_STANDARD:
        case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_HLG10:
        case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_STANDARD_SMPTE_2094_50:
        case ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_HLG10_SMPTE_2094_50:
          break;
        default:
          ALOGE("%s: Only support for HLG10 and SMPTE_2094_50 is available!",
                __FUNCTION__);
          return false;
      }
    }
  }

  if ((characteristics.exposure_time_range[0] >=
       characteristics.exposure_time_range[1]) ||
      ((characteristics.exposure_time_range[0] < kSupportedExposureTimeRange[0]) ||
       (characteristics.exposure_time_range[1] >
        kSupportedExposureTimeRange[1]))) {
    ALOGE("%s: Unsupported exposure range", __FUNCTION__);
    return false;
  }

  if ((characteristics.frame_duration_range[0] >=
       characteristics.frame_duration_range[1]) ||
      ((characteristics.frame_duration_range[0] <
        kSupportedFrameDurationRange[0]) ||
       (characteristics.frame_duration_range[1] >
        kSupportedFrameDurationRange[1]))) {
    ALOGE("%s: Unsupported frame duration range", __FUNCTION__);
    return false;
  }

  if ((characteristics.sensitivity_range[0] >=
       characteristics.sensitivity_range[1]) ||
      ((characteristics.sensitivity_range[0] < kSupportedSensitivityRange[0]) ||
       (characteristics.sensitivity_range[1] > kSupportedSensitivityRange[1])) ||
      (!((kDefaultSensitivity >= characteristics.sensitivity_range[0]) &&
         (kDefaultSensitivity <= characteristics.sensitivity_range[1])))) {
    ALOGE("%s: Unsupported sensitivity range", __FUNCTION__);
    return false;
  }

  if (characteristics.color_arangement != kSupportedColorFilterArrangement) {
    ALOGE("%s: Unsupported color arrangement!", __FUNCTION__);
    return false;
  }

  for (const auto& blackLevel : characteristics.black_level_pattern) {
    if (blackLevel >= characteristics.max_raw_value) {
      ALOGE("%s: Black level matches or exceeds max RAW value!", __FUNCTION__);
      return false;
    }
  }

  if ((characteristics.frame_duration_range[0] / characteristics.height) == 0) {
    ALOGE("%s: Zero row readout time!", __FUNCTION__);
    return false;
  }

  if (characteristics.max_raw_streams > kMaxRAWStreams) {
    ALOGE("%s: RAW streams maximum %u exceeds supported maximum %u",
          __FUNCTION__, characteristics.max_raw_streams, kMaxRAWStreams);
    return false;
  }

  if (characteristics.max_processed_streams > kMaxProcessedStreams) {
    ALOGE("%s: Processed streams maximum %u exceeds supported maximum %u",
          __FUNCTION__, characteristics.max_processed_streams,
          kMaxProcessedStreams);
    return false;
  }

  if (characteristics.max_stalling_streams > kMaxStallingStreams) {
    ALOGE("%s: Stalling streams maximum %u exceeds supported maximum %u",
          __FUNCTION__, characteristics.max_stalling_streams,
          kMaxStallingStreams);
    return false;
  }

  if (characteristics.max_input_streams > kMaxInputStreams) {
    ALOGE("%s: Input streams maximum %u exceeds supported maximum %u",
          __FUNCTION__, characteristics.max_input_streams, kMaxInputStreams);
    return false;
  }

  if ((characteristics.lens_shading_map_size[0] > kMaxLensShadingMapSize[0]) ||
      (characteristics.lens_shading_map_size[1] > kMaxLensShadingMapSize[1])) {
    ALOGE("%s: Lens shading map [%dx%d] exceeds supprorted maximum [%dx%d]",
          __FUNCTION__, characteristics.lens_shading_map_size[0],
          characteristics.lens_shading_map_size[1], kMaxLensShadingMapSize[0],
          kMaxLensShadingMapSize[1]);
    return false;
  }

  if (characteristics.max_pipeline_depth < kPipelineDepth) {
    ALOGE("%s: Pipeline depth %d smaller than supprorted minimum %d",
          __FUNCTION__, characteristics.max_pipeline_depth, kPipelineDepth);
    return false;
  }

  return true;
}

static void SplitStreamCombination(
    const StreamConfiguration& original_config,
    StreamConfiguration* default_mode_config,
    StreamConfiguration* max_resolution_mode_config,
    StreamConfiguration* input_stream_config) {
  // Go through the streams
  if (default_mode_config == nullptr || max_resolution_mode_config == nullptr ||
      input_stream_config == nullptr) {
    ALOGE("%s: Input stream / output stream configs are nullptr", __FUNCTION__);
    return;
  }
  for (const auto& stream : original_config.streams) {
    if (stream.stream_type == google_camera_hal::StreamType::kInput) {
      input_stream_config->streams.push_back(stream);
      continue;
    }
    if (stream.intended_for_default_resolution_mode) {
      default_mode_config->streams.push_back(stream);
    }
    if (stream.intended_for_max_resolution_mode) {
      max_resolution_mode_config->streams.push_back(stream);
    }
  }
}

bool EmulatedSensor::IsStreamCombinationSupported(
    uint32_t logical_id, const StreamConfiguration& config,
    StreamConfigurationMap& default_config_map,
    StreamConfigurationMap& max_resolution_config_map,
    const PhysicalStreamConfigurationMap& physical_map,
    const PhysicalStreamConfigurationMap& physical_map_max_resolution,
    const LogicalCharacteristics& sensor_chars) {
  StreamConfiguration default_mode_config, max_resolution_mode_config,
      input_stream_config;
  SplitStreamCombination(config, &default_mode_config,
                         &max_resolution_mode_config, &input_stream_config);

  return IsStreamCombinationSupported(logical_id, default_mode_config,
                                      default_config_map, physical_map,
                                      sensor_chars) &&
         IsStreamCombinationSupported(
             logical_id, max_resolution_mode_config, max_resolution_config_map,
             physical_map_max_resolution, sensor_chars, /*is_max_res*/ true) &&

         (IsStreamCombinationSupported(logical_id, input_stream_config,
                                       default_config_map, physical_map,
                                       sensor_chars) ||
          IsStreamCombinationSupported(
              logical_id, input_stream_config, max_resolution_config_map,
              physical_map_max_resolution, sensor_chars, /*is_max_res*/ true));
}

bool EmulatedSensor::IsStreamCombinationSupported(
    uint32_t logical_id, const StreamConfiguration& config,
    StreamConfigurationMap& config_map,
    const PhysicalStreamConfigurationMap& physical_map,
    const LogicalCharacteristics& sensor_chars, bool is_max_res) {
  uint32_t input_stream_count = 0;
  // Map from physical camera id to number of streams for that physical camera
  std::map<uint32_t, uint32_t> raw_stream_count;
  std::map<uint32_t, uint32_t> processed_stream_count;
  std::map<uint32_t, uint32_t> stalling_stream_count;

  // Only allow the stream configurations specified in
  // dynamicSizeStreamConfigurations.
  for (const auto& stream : config.streams) {
    bool is_dynamic_output =
        (stream.is_physical_camera_stream && stream.group_id != -1);
    if (stream.rotation != google_camera_hal::StreamRotation::kRotation0) {
      ALOGE("%s: Stream rotation: 0x%x not supported!", __FUNCTION__,
            stream.rotation);
      return false;
    }

    if (stream.stream_type == google_camera_hal::StreamType::kInput) {
      if (sensor_chars.at(logical_id).max_input_streams == 0) {
        ALOGE("%s: Input streams are not supported on this device!",
              __FUNCTION__);
        return false;
      }

      auto const& supported_outputs =
          config_map.GetValidOutputFormatsForInput(stream.format);
      if (supported_outputs.empty()) {
        ALOGE("%s: Input stream with format: 0x%x no supported on this device!",
              __FUNCTION__, stream.format);
        return false;
      }

      input_stream_count++;
    } else {
      if (stream.is_physical_camera_stream &&
          physical_map.find(stream.physical_camera_id) == physical_map.end()) {
        ALOGE("%s: Invalid physical camera id %d", __FUNCTION__,
              stream.physical_camera_id);
        return false;
      }

      if (is_dynamic_output) {
        auto dynamic_physical_output_formats =
            physical_map.at(stream.physical_camera_id)
                ->GetDynamicPhysicalStreamOutputFormats();
        if (dynamic_physical_output_formats.find(stream.format) ==
            dynamic_physical_output_formats.end()) {
          ALOGE("%s: Unsupported physical stream format %d", __FUNCTION__,
                stream.format);
          return false;
        }
      }

      if ((stream.dynamic_profile !=
           ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_STANDARD) &&
          (stream.dynamic_profile !=
           ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_STANDARD_SMPTE_2094_50)) {
        const SensorCharacteristics& sensor_char =
            stream.is_physical_camera_stream
                ? sensor_chars.at(stream.physical_camera_id)
                : sensor_chars.at(logical_id);
        if (!sensor_char.is_10bit_dynamic_range_capable) {
          ALOGE("%s: 10-bit dynamic range output not supported on this device!",
                __FUNCTION__);
          return false;
        }

        if ((stream.format != HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) &&
            (static_cast<android_pixel_format_v1_1_t>(stream.format) !=
             HAL_PIXEL_FORMAT_YCBCR_P010)) {
          ALOGE(
              "%s: 10-bit dynamic range profile 0x%x not supported on a non "
              "10-bit output stream"
              " pixel format 0x%x",
              __FUNCTION__, stream.dynamic_profile, stream.format);
          return false;
        }

        if ((static_cast<android_pixel_format_v1_1_t>(stream.format) ==
             HAL_PIXEL_FORMAT_YCBCR_P010) &&
            ((stream.data_space !=
              static_cast<android_dataspace_t>(Dataspace::BT2020_ITU_HLG)) &&
             (stream.data_space !=
              static_cast<android_dataspace_t>(Dataspace::BT2020_HLG)) &&
             (stream.data_space !=
              static_cast<android_dataspace_t>(Dataspace::UNKNOWN)))) {
          ALOGE(
              "%s: Unsupported stream data space 0x%x for 10-bit YUV "
              "output",
              __FUNCTION__, stream.data_space);
          return false;
        }
      }

      switch (stream.format) {
        case HAL_PIXEL_FORMAT_BLOB:
          if ((stream.data_space != HAL_DATASPACE_V0_JFIF) &&
              (stream.data_space !=
               static_cast<android_dataspace_t>(
                   aidl::android::hardware::graphics::common::Dataspace::JPEG_R)) &&
              (stream.data_space != HAL_DATASPACE_UNKNOWN)) {
            ALOGE("%s: Unsupported Blob dataspace 0x%x", __FUNCTION__,
                  stream.data_space);
            return false;
          }
          if (stream.is_physical_camera_stream) {
            stalling_stream_count[stream.physical_camera_id]++;
          } else {
            for (const auto& p : physical_map) {
              stalling_stream_count[p.first]++;
            }
          }
          break;
        case HAL_PIXEL_FORMAT_RAW16: {
          const SensorCharacteristics& sensor_char =
              stream.is_physical_camera_stream
                  ? sensor_chars.at(stream.physical_camera_id)
                  : sensor_chars.at(logical_id);
          auto sensor_height =
              is_max_res ? sensor_char.full_res_height : sensor_char.height;
          auto sensor_width =
              is_max_res ? sensor_char.full_res_width : sensor_char.width;
          if (stream.height != sensor_height || stream.width != sensor_width) {
            ALOGE(
                "%s, RAW16 buffer height %d and width %d must match sensor "
                "height: %zu"
                " and width: %zu",
                __FUNCTION__, stream.height, stream.width, sensor_height,
                sensor_width);
            return false;
          }
          if (stream.is_physical_camera_stream) {
            raw_stream_count[stream.physical_camera_id]++;
          } else {
            for (const auto& p : physical_map) {
              raw_stream_count[p.first]++;
            }
          }
        } break;
        default:
          if (stream.is_physical_camera_stream) {
            processed_stream_count[stream.physical_camera_id]++;
          } else {
            for (const auto& p : physical_map) {
              processed_stream_count[p.first]++;
            }
          }
      }

      auto output_sizes =
          is_dynamic_output
              ? physical_map.at(stream.physical_camera_id)
                    ->GetDynamicPhysicalStreamOutputSizes(stream.format)
          : stream.is_physical_camera_stream
              ? physical_map.at(stream.physical_camera_id)
                    ->GetOutputSizes(stream.format, stream.data_space)
              : config_map.GetOutputSizes(stream.format, stream.data_space);

      auto stream_size = std::make_pair(stream.width, stream.height);
      if (output_sizes.find(stream_size) == output_sizes.end()) {
        ALOGE("%s: Stream with size %dx%d and format 0x%x is not supported!",
              __FUNCTION__, stream.width, stream.height, stream.format);
        return false;
      }
    }

    if (!sensor_chars.at(logical_id).support_stream_use_case) {
      if (stream.use_case != ANDROID_SCALER_AVAILABLE_STREAM_USE_CASES_DEFAULT) {
        ALOGE("%s: Camera device doesn't support non-default stream use case!",
              __FUNCTION__);
        return false;
      }
    } else if (stream.use_case >
               sensor_chars.at(logical_id).end_valid_stream_use_case) {
      ALOGE("%s: Stream with use case %d is not supported!", __FUNCTION__,
            stream.use_case);
      return false;
    } else if (stream.use_case !=
               ANDROID_SCALER_AVAILABLE_STREAM_USE_CASES_DEFAULT) {
      if (stream.use_case ==
              ANDROID_SCALER_AVAILABLE_STREAM_USE_CASES_STILL_CAPTURE) {
        if (stream.format != HAL_PIXEL_FORMAT_YCBCR_420_888 &&
            stream.format != HAL_PIXEL_FORMAT_BLOB) {
          ALOGE("%s: Stream with use case %d isn't compatible with format %d",
              __FUNCTION__, stream.use_case, stream.format);
          return false;
        }
      } else if ((stream.format == HAL_PIXEL_FORMAT_RAW16) ^
                 (stream.use_case ==
                  ANDROID_SCALER_AVAILABLE_STREAM_USE_CASES_CROPPED_RAW)) {
        // Either both stream use case == CROPPED_RAW and format == RAW16, or
        // stream use case != CROPPED_RAW and format != RAW16 for the
        // combination to be valid.
        ALOGE(
            "%s: Stream with use case CROPPED_RAW isn't compatible with non "
            "RAW_SENSOR formats",
            __FUNCTION__);
        return false;

      } else if (stream.format != HAL_PIXEL_FORMAT_YCBCR_420_888 &&
                 stream.format != HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED &&
                 stream.format != HAL_PIXEL_FORMAT_RAW16) {
        ALOGE("%s: Stream with use case %d isn't compatible with format %d",
              __FUNCTION__, stream.use_case, stream.format);
        return false;
      }
    }
  }

  for (const auto& raw_count : raw_stream_count) {
    unsigned int max_raw_streams =
        sensor_chars.at(raw_count.first).max_raw_streams +
        (is_max_res
             ? 1
             : 0);  // The extra raw stream is allowed for remosaic reprocessing.
    if (raw_count.second > max_raw_streams) {
      ALOGE("%s: RAW streams maximum %u exceeds supported maximum %u",
            __FUNCTION__, raw_count.second, max_raw_streams);
      return false;
    }
  }

  for (const auto& stalling_count : stalling_stream_count) {
    if (stalling_count.second >
        sensor_chars.at(stalling_count.first).max_stalling_streams) {
      ALOGE("%s: Stalling streams maximum %u exceeds supported maximum %u",
            __FUNCTION__, stalling_count.second,
            sensor_chars.at(stalling_count.first).max_stalling_streams);
      return false;
    }
  }

  for (const auto& processed_count : processed_stream_count) {
    if (processed_count.second >
        sensor_chars.at(processed_count.first).max_processed_streams) {
      ALOGE("%s: Processed streams maximum %u exceeds supported maximum %u",
            __FUNCTION__, processed_count.second,
            sensor_chars.at(processed_count.first).max_processed_streams);
      return false;
    }
  }

  if (input_stream_count > sensor_chars.at(logical_id).max_input_streams) {
    ALOGE("%s: Input stream maximum %u exceeds supported maximum %u",
          __FUNCTION__, input_stream_count,
          sensor_chars.at(logical_id).max_input_streams);
    return false;
  }

  // TODO: Check session parameters. For now assuming all combinations
  // are supported.

  return true;
}

status_t EmulatedSensor::StartUp(
    uint32_t logical_camera_id,
    std::unique_ptr<LogicalCharacteristics> logical_chars) {
  if (isRunning()) {
    return OK;
  }

  if (logical_chars.get() == nullptr) {
    return BAD_VALUE;
  }

  chars_ = std::move(logical_chars);
  auto device_chars = chars_->find(logical_camera_id);
  if (device_chars == chars_->end()) {
    ALOGE(
        "%s: Logical camera id: %u absent from logical camera characteristics!",
        __FUNCTION__, logical_camera_id);
    return BAD_VALUE;
  }

  for (const auto& it : *chars_) {
    if (!AreCharacteristicsSupported(it.second)) {
      ALOGE("%s: Sensor characteristics for camera id: %u not supported!",
            __FUNCTION__, it.first);
      return BAD_VALUE;
    }
  }

  logical_camera_id_ = logical_camera_id;
  frame_source_ =
      std::make_unique<EmulatedFrameSource>(*chars_, logical_camera_id);
  jpeg_compressor_ = std::make_unique<JpegCompressor>();

  auto res = run(LOG_TAG, ANDROID_PRIORITY_URGENT_DISPLAY);
  if (res != OK) {
    ALOGE("Unable to start up sensor capture thread: %d", res);
  }

  return res;
}

status_t EmulatedSensor::ShutDown() {
  int res;
  res = requestExitAndWait();
  if (res != OK) {
    ALOGE("Unable to shut down sensor capture thread: %d", res);
  }
  return res;
}

void EmulatedSensor::SetCurrentRequest(
    std::unique_ptr<LogicalCameraSettings> logical_settings,
    std::unique_ptr<HwlPipelineResult> result,
    std::unique_ptr<HwlPipelineResult> partial_result,
    std::unique_ptr<Buffers> input_buffers,
    std::unique_ptr<Buffers> output_buffers) {
  Mutex::Autolock lock(control_mutex_);
  current_settings_ = std::move(logical_settings);
  current_result_ = std::move(result);
  current_input_buffers_ = std::move(input_buffers);
  current_output_buffers_ = std::move(output_buffers);
  partial_result_ = std::move(partial_result);
}

bool EmulatedSensor::WaitForVSyncLocked(nsecs_t reltime) {
  got_vsync_ = false;
  while (!got_vsync_) {
    auto res = vsync_.waitRelative(control_mutex_, reltime);
    if (res != OK && res != TIMED_OUT) {
      ALOGE("%s: Error waiting for VSync signal: %d", __FUNCTION__, res);
      return false;
    }
  }

  return got_vsync_;
}

bool EmulatedSensor::WaitForVSync(nsecs_t reltime) {
  Mutex::Autolock lock(control_mutex_);

  return WaitForVSyncLocked(reltime);
}

status_t EmulatedSensor::Flush() {
  Mutex::Autolock lock(control_mutex_);
  auto ret = WaitForVSyncLocked(kSupportedFrameDurationRange[1]);

  // First recreate the jpeg compressor. This will abort any ongoing processing
  // and flush any pending jobs.
  jpeg_compressor_ = std::make_unique<JpegCompressor>();

  // Then return any pending frames here
  if ((current_input_buffers_.get() != nullptr) &&
      (!current_input_buffers_->empty())) {
    current_input_buffers_->clear();
  }
  if ((current_output_buffers_.get() != nullptr) &&
      (!current_output_buffers_->empty())) {
    for (const auto& buffer : *current_output_buffers_) {
      buffer->stream_buffer.status = BufferStatus::kError;
    }

    if ((current_result_.get() != nullptr) &&
        (current_result_->result_metadata.get() != nullptr)) {
      if (current_output_buffers_->at(0)->callback.notify != nullptr) {
        NotifyMessage msg = ErrorMessage{
            .frame_number = current_output_buffers_->at(0)->frame_number,
            .error_stream_id = -1,
            .error_code = ErrorCode::kErrorResult,
        };

        current_output_buffers_->at(0)->callback.notify(
            current_result_->pipeline_id, msg);
      }
    }

    current_output_buffers_->clear();
  }

  return ret ? OK : TIMED_OUT;
}

nsecs_t EmulatedSensor::getSystemTimeWithSource(uint32_t timestamp_source) {
  if (timestamp_source == ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME) {
    return systemTime(SYSTEM_TIME_BOOTTIME);
  }
  return systemTime(SYSTEM_TIME_MONOTONIC);
}

bool EmulatedSensor::threadLoop() {
  ATRACE_CALL();
  /**
   * Sensor capture operation main loop.
   *
   */

  /**
   * Stage 1: Read in latest control parameters
   */
  std::unique_ptr<Buffers> next_buffers;
  std::unique_ptr<Buffers> next_input_buffer;
  std::unique_ptr<HwlPipelineResult> next_result;
  std::unique_ptr<HwlPipelineResult> partial_result;
  std::unique_ptr<LogicalCameraSettings> settings;
  HwlPipelineCallback callback = {
      .process_pipeline_result = nullptr,
      .process_pipeline_batch_result = nullptr,
      .notify = nullptr,
  };
  {
    Mutex::Autolock lock(control_mutex_);
    std::swap(settings, current_settings_);
    std::swap(next_buffers, current_output_buffers_);
    std::swap(next_input_buffer, current_input_buffers_);
    std::swap(next_result, current_result_);
    std::swap(partial_result, partial_result_);

    // Signal VSync for start of readout
    ALOGVV("Sensor VSync");
    got_vsync_ = true;
    vsync_.signal();
  }

  auto frame_duration = EmulatedSensor::kSupportedFrameDurationRange[0];
  auto exposure_time = EmulatedSensor::kSupportedExposureTimeRange[0];
  uint32_t timestamp_source = ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
  // Frame duration must always be the same among all physical devices
  if ((settings.get() != nullptr) && (!settings->empty())) {
    frame_duration = settings->begin()->second.frame_duration;
    exposure_time = settings->begin()->second.exposure_time;
    timestamp_source = settings->begin()->second.timestamp_source;
  }

  nsecs_t start_real_time = getSystemTimeWithSource(timestamp_source);
  // Stagefright cares about system time for timestamps, so base simulated
  // time on that.
  nsecs_t frame_end_real_time = start_real_time + frame_duration;

  /**
   * Stage 2: Capture new image
   */
  next_capture_time_ = frame_end_real_time;
  next_readout_time_ = frame_end_real_time + exposure_time;

  if (frame_source_) {
    frame_source_->ResetSensorBinningInfo();
  }

  bool reprocess_request = false;
  if ((next_input_buffer.get() != nullptr) && (!next_input_buffer->empty())) {
    if (next_input_buffer->size() > 1) {
      ALOGW("%s: Reprocess supports only single input!", __FUNCTION__);
    }

    camera_metadata_ro_entry_t entry;
    auto ret =
        next_result->result_metadata->Get(ANDROID_SENSOR_TIMESTAMP, &entry);
    if ((ret == OK) && (entry.count == 1)) {
      next_capture_time_ = entry.data.i64[0];
    } else {
      ALOGW("%s: Reprocess timestamp absent!", __FUNCTION__);
    }

    ret =
        next_result->result_metadata->Get(ANDROID_SENSOR_EXPOSURE_TIME, &entry);
    if ((ret == OK) && (entry.count == 1)) {
      next_readout_time_ = next_capture_time_ + entry.data.i64[0];
    } else {
      next_readout_time_ = next_capture_time_;
    }

    reprocess_request = true;
  }

  if ((next_buffers != nullptr) && (settings != nullptr)) {
    callback = next_buffers->at(0)->callback;
    std::vector<StreamGroupState> stream_group_state =
        GetStreamGroupState(*next_buffers);
    uint32_t frame_number = next_buffers->at(0)->frame_number;
    if (callback.notify != nullptr) {
      NotifyMessage msg = ShutterMessage{
          .frame_number = frame_number,
          .timestamp_ns = static_cast<uint64_t>(next_capture_time_),
          .readout_timestamp_ns = static_cast<uint64_t>(next_readout_time_),
          .stream_group_state = stream_group_state};
      callback.notify(next_result->pipeline_id, msg);
    }

    if (callback.notify_override_pending_buffer != nullptr &&
        stream_group_state.size() > 0) {
      callback.notify_override_pending_buffer(frame_number, stream_group_state);
    }

    auto b = next_buffers->begin();
    while (b != next_buffers->end()) {
      auto device_settings = settings->find((*b)->camera_id);
      if (device_settings == settings->end()) {
        ALOGE("%s: Sensor settings absent for device: %d", __func__,
              (*b)->camera_id);
        b = next_buffers->erase(b);
        continue;
      }

      auto device_chars = chars_->find((*b)->camera_id);
      if (device_chars == chars_->end()) {
        ALOGE("%s: Sensor characteristics absent for device: %d", __func__,
              (*b)->camera_id);
        b = next_buffers->erase(b);
        continue;
      }

      ALOGVV("Starting next capture: Exposure: %" PRIu64 " ms, gain: %d",
             ns2ms(device_settings->second.exposure_time),
             device_settings->second.gain);

      (*b)->stream_buffer.status = BufferStatus::kOk;

      const SensorBuffer* input_buffer = nullptr;
      if (reprocess_request && !next_input_buffer->empty()) {
        input_buffer = next_input_buffer->begin()->get();
      }

      switch ((*b)->format) {
        case PixelFormat::BLOB:
          if ((*b)->dataSpace == HAL_DATASPACE_V0_JFIF ||
              (*b)->dataSpace ==
                  static_cast<android_dataspace_t>(
                      aidl::android::hardware::graphics::common::Dataspace::JPEG_R)) {
            bool is_jpeg_r =
                (*b)->dataSpace ==
                static_cast<android_dataspace_t>(
                    aidl::android::hardware::graphics::common::Dataspace::JPEG_R);

            if (is_jpeg_r && reprocess_request) {
              ALOGE(
                  "%s: Reprocess requests with output format JPEG_R are not "
                  "supported!",
                  __FUNCTION__);
              (*b)->stream_buffer.status = BufferStatus::kError;
              break;
            }

            bool treat_as_reprocess = reprocess_request;
            if (device_chars->second.quad_bayer_sensor && reprocess_request &&
                input_buffer != nullptr &&
                input_buffer->format == PixelFormat::RAW16) {
              treat_as_reprocess = false;
            }

            EmulatedFrameSource::YUV420Frame yuv_input{};
            if (treat_as_reprocess && input_buffer != nullptr) {
              yuv_input.width = input_buffer->width;
              yuv_input.height = input_buffer->height;
              yuv_input.planes = input_buffer->plane.img_y_crcb;
            }

            auto jpeg_input = std::make_unique<JpegYUV420Input>();
            jpeg_input->width = (*b)->width;
            jpeg_input->height = (*b)->height;
            jpeg_input->color_space = (*b)->color_space;

            // JPEG R has specific buffer requirements (planar)
            size_t buffer_size =
                is_jpeg_r ? (*b)->width * (*b)->height * 3
                          : (jpeg_input->width * jpeg_input->height * 3) / 2;
            auto img = new uint8_t[buffer_size];

            if (is_jpeg_r) {
              jpeg_input->yuv_planes = {
                  .img_y = img,
                  .img_cb = img + (*b)->width * (*b)->height * 2,
                  .img_cr = img + (*b)->width * (*b)->height * 2 + 2,
                  .y_stride = (*b)->width * 2,
                  .cbcr_stride = (*b)->width * 2,
                  .cbcr_step = 2,
                  .bytesPerPixel = 2};
            } else {
              jpeg_input->yuv_planes = {
                  .img_y = img,
                  .img_cb = img + jpeg_input->width * jpeg_input->height,
                  .img_cr =
                      img + (jpeg_input->width * jpeg_input->height * 5) / 4,
                  .y_stride = jpeg_input->width,
                  .cbcr_stride = jpeg_input->width / 2,
                  .cbcr_step = 1};
            }
            jpeg_input->buffer_owner = true;

            EmulatedFrameSource::YUV420Frame yuv_output{
                .width = jpeg_input->width,
                .height = jpeg_input->height,
                .planes = jpeg_input->yuv_planes};
            // Pass color space for conversion if needed
            yuv_output.color_space = (*b)->color_space;

            status_t ret = frame_source_->RenderYUV420(
                (*b)->camera_id, next_capture_time_, device_settings->second,
                yuv_output,
                (treat_as_reprocess && input_buffer) ? &yuv_input : nullptr);

            if (ret != OK) {
              (*b)->stream_buffer.status = BufferStatus::kError;
              break;
            }

            auto jpeg_job = std::make_unique<JpegYUV420Job>();
            jpeg_job->exif_utils = std::unique_ptr<ExifUtils>(
                ExifUtils::Create(device_chars->second));
            jpeg_job->input = std::move(jpeg_input);
            // If jpeg compression is successful, then the jpeg compressor
            // must set the corresponding status.
            (*b)->stream_buffer.status = BufferStatus::kError;
            std::swap(jpeg_job->output, *b);
            jpeg_job->result_metadata =
                HalCameraMetadata::Clone(next_result->result_metadata.get());

            Mutex::Autolock lock(control_mutex_);
            jpeg_compressor_->QueueYUV420(std::move(jpeg_job));
          } else {
            ALOGE("%s: Format %x with dataspace %x is TODO", __FUNCTION__,
                  (*b)->format, (*b)->dataSpace);
            (*b)->stream_buffer.status = BufferStatus::kError;
          }
          break;
        default:
          status_t res = frame_source_->ProduceFrame(
              (*b)->camera_id, next_capture_time_, device_settings->second,
              (*b).get(), input_buffer);
          if (res != OK) {
            (*b)->stream_buffer.status = BufferStatus::kError;
          }
      }

      b = next_buffers->erase(b);
    }
  }

  if (reprocess_request) {
    auto input_buffer = next_input_buffer->begin();
    while (input_buffer != next_input_buffer->end()) {
      (*input_buffer++)->stream_buffer.status = BufferStatus::kOk;
    }
    next_input_buffer->clear();
  }

  nsecs_t work_done_real_time = getSystemTimeWithSource(timestamp_source);
  // Returning the results at this point is not entirely correct from timing
  // perspective. Under ideal conditions where 'ReturnResults' completes
  // in less than 'time_accuracy' we need to return the results after the
  // frame cycle expires. However under real conditions various system
  // components like SurfaceFlinger, Encoder, LMK etc. could be consuming most
  // of the resources and the duration of "ReturnResults" can get comparable to
  // 'kDefaultFrameDuration'. This will skew the frame cycle and can result in
  // potential frame drops. To avoid this scenario when we are running under
  // tight deadlines (less than 'kReturnResultThreshod') try to return the
  // results immediately. In all other cases with more relaxed deadlines
  // the occasional bump during 'ReturnResults' should not have any
  // noticeable effect.
  if ((work_done_real_time + kReturnResultThreshod) > frame_end_real_time) {
    ReturnResults(callback, std::move(settings), std::move(next_result),
                  reprocess_request, std::move(partial_result));
  }

  work_done_real_time = getSystemTimeWithSource(timestamp_source);
  ALOGVV("Sensor vertical blanking interval");
  const nsecs_t time_accuracy = 2e6;  // 2 ms of imprecision is ok
  if (work_done_real_time < frame_end_real_time - time_accuracy) {
    timespec t;
    t.tv_sec = (frame_end_real_time - work_done_real_time) / 1000000000L;
    t.tv_nsec = (frame_end_real_time - work_done_real_time) % 1000000000L;

    int ret;
    do {
      ret = nanosleep(&t, &t);
    } while (ret != 0);
  }

  ReturnResults(callback, std::move(settings), std::move(next_result),
                reprocess_request, std::move(partial_result));
  return true;
}

void EmulatedSensor::ReturnResults(
    HwlPipelineCallback callback,
    std::unique_ptr<LogicalCameraSettings> settings,
    std::unique_ptr<HwlPipelineResult> result, bool reprocess_request,
    std::unique_ptr<HwlPipelineResult> partial_result) {
  if ((callback.process_pipeline_result != nullptr) &&
      (result.get() != nullptr) && (result->result_metadata.get() != nullptr)) {
    auto logical_settings = settings->find(logical_camera_id_);
    if (logical_settings == settings->end()) {
      ALOGE("%s: Logical camera id: %u not found in settings!", __FUNCTION__,
            logical_camera_id_);
      return;
    }
    auto device_chars = chars_->find(logical_camera_id_);
    if (device_chars == chars_->end()) {
      ALOGE("%s: Sensor characteristics absent for device: %d", __func__,
            logical_camera_id_);
      return;
    }
    result->result_metadata->Set(ANDROID_SENSOR_TIMESTAMP, &next_capture_time_,
                                 1);

    camera_metadata_ro_entry_t lensEntry;
    auto lensRet = result->result_metadata->Get(
        ANDROID_STATISTICS_LENS_INTRINSIC_SAMPLES, &lensEntry);
    if ((lensRet == OK) && (lensEntry.count > 0)) {
      result->result_metadata->Set(ANDROID_STATISTICS_LENS_INTRINSIC_TIMESTAMPS,
                                   &next_capture_time_, 1);
    }

    if (frame_source_->HasBinningInfo(logical_camera_id_)) {
      BinningState state = frame_source_->GetBinningState(logical_camera_id_);
      uint8_t raw_binned_factor_used = 0;
      if (!reprocess_request && state.raw_binning_factor_used) {
        raw_binned_factor_used = 1;
      }
      result->result_metadata->Set(ANDROID_SENSOR_RAW_BINNING_FACTOR_USED,
                                   &raw_binned_factor_used, 1);

      if (state.has_cropped_raw_stream) {
        if (state.raw_in_sensor_zoom_applied) {
          result->result_metadata->Set(
              ANDROID_SCALER_RAW_CROP_REGION,
              device_chars->second.raw_crop_region_zoomed, 4);

        } else {
          result->result_metadata->Set(
              ANDROID_SCALER_RAW_CROP_REGION,
              device_chars->second.raw_crop_region_unzoomed, 4);
        }
      }
    }

    if (logical_settings->second.lens_shading_map_mode ==
        ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_ON) {
      if ((device_chars->second.lens_shading_map_size[0] > 0) &&
          (device_chars->second.lens_shading_map_size[1] > 0)) {
        // Perfect lens, no actual shading needed.
        std::vector<float> lens_shading_map(
            device_chars->second.lens_shading_map_size[0] *
                device_chars->second.lens_shading_map_size[1] * 4,
            1.f);

        result->result_metadata->Set(ANDROID_STATISTICS_LENS_SHADING_MAP,
                                     lens_shading_map.data(),
                                     lens_shading_map.size());
      }
    }
    if (logical_settings->second.report_video_stab) {
      result->result_metadata->Set(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
                                   &logical_settings->second.video_stab, 1);
    }
    if (logical_settings->second.report_edge_mode) {
      result->result_metadata->Set(ANDROID_EDGE_MODE,
                                   &logical_settings->second.edge_mode, 1);
    }
    if (logical_settings->second.report_neutral_color_point) {
      result->result_metadata->Set(ANDROID_SENSOR_NEUTRAL_COLOR_POINT,
                                   kNeutralColorPoint,
                                   ARRAY_SIZE(kNeutralColorPoint));
    }
    if (logical_settings->second.report_green_split) {
      result->result_metadata->Set(ANDROID_SENSOR_GREEN_SPLIT, &kGreenSplit, 1);
    }
    if (logical_settings->second.report_noise_profile) {
      float base_gain_factor = EmulatedFrameSource::GetBaseGainFactor(
          device_chars->second.max_raw_value);
      frame_source_->CalculateAndAppendNoiseProfile(
          logical_settings->second.gain, base_gain_factor,
          result->result_metadata.get());
    }
    if (logical_settings->second.report_rotate_and_crop) {
      result->result_metadata->Set(ANDROID_SCALER_ROTATE_AND_CROP,
                                   &logical_settings->second.rotate_and_crop, 1);
    }

    if (!result->physical_camera_results.empty()) {
      for (auto& it : result->physical_camera_results) {
        auto physical_settings = settings->find(it.first);
        if (physical_settings == settings->end()) {
          ALOGE("%s: Physical settings for camera id: %u are absent!",
                __FUNCTION__, it.first);
          continue;
        }

        if (frame_source_->HasBinningInfo(it.first)) {
          BinningState physical_state = frame_source_->GetBinningState(it.first);
          uint8_t raw_binned_factor_used = 0;
          if (!reprocess_request && physical_state.raw_binning_factor_used) {
            raw_binned_factor_used = 1;
          }
          it.second->Set(ANDROID_SENSOR_RAW_BINNING_FACTOR_USED,
                         &raw_binned_factor_used, 1);
        }

        // Sensor timestamp for all physical devices must be the same.
        it.second->Set(ANDROID_SENSOR_TIMESTAMP, &next_capture_time_, 1);
        if (physical_settings->second.report_neutral_color_point) {
          it.second->Set(ANDROID_SENSOR_NEUTRAL_COLOR_POINT, kNeutralColorPoint,
                         ARRAY_SIZE(kNeutralColorPoint));
        }
        if (physical_settings->second.report_green_split) {
          it.second->Set(ANDROID_SENSOR_GREEN_SPLIT, &kGreenSplit, 1);
        }
        if (physical_settings->second.report_noise_profile) {
          auto physical_chars = chars_->find(it.first);
          if (physical_chars == chars_->end()) {
            ALOGE("%s: Sensor characteristics absent for device: %d", __func__,
                  it.first);
          } else {
            float base_gain_factor = EmulatedFrameSource::GetBaseGainFactor(
                physical_chars->second.max_raw_value);
            frame_source_->CalculateAndAppendNoiseProfile(
                physical_settings->second.gain, base_gain_factor,
                it.second.get());
          }
        }
      }
    }

    // Partial result count for partial result is set to a value
    // only when partial results are supported
    if (partial_result->partial_result != 0) {
      callback.process_pipeline_result(std::move(partial_result));
    }
    callback.process_pipeline_result(std::move(result));
  }
}

std::vector<StreamGroupState> EmulatedSensor::GetStreamGroupState(
    const Buffers& outputBuffers) {
  std::unordered_map<int32_t, std::vector<int32_t>> stream_group_state_map;
  for (const auto& sensorBuffer : outputBuffers) {
    if (sensorBuffer->group_id == -1) {
      continue;
    }
    if (!sensorBuffer->group_concurrency_enabled) {
      continue;
    }
    int32_t stream_id = sensorBuffer->stream_buffer.stream_id;
    stream_group_state_map[sensorBuffer->group_id].push_back(stream_id);
  }

  std::vector<StreamGroupState> res;
  for (auto& [group_id, value] : stream_group_state_map) {
    res.push_back({group_id, std::move(value)});
  }
  return res;
}

}  // namespace android

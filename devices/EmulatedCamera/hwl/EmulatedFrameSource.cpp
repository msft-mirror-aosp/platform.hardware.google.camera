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

#define LOG_TAG "EmulatedFrameSource"
#define ATRACE_TAG ATRACE_TAG_CAMERA
#include "EmulatedFrameSource.h"

#include <android/hardware/graphics/common/1.2/types.h>
#include <cutils/properties.h>
#include <libyuv.h>
#include <log/log.h>
#include <utils/Trace.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "utils/HWLUtils.h"

#ifdef LOG_NNDEBUG
#define ALOGVV(...) ALOGV(__VA_ARGS__)
#else
#define ALOGVV(...) ((void)0)
#endif

namespace android {
namespace framesource {

using android::hardware::graphics::common::V1_2::Dataspace;

// Copied from ColorSpace.java (see Named)
enum ColorSpaceNamed {
  SRGB,
  LINEAR_SRGB,
  EXTENDED_SRGB,
  LINEAR_EXTENDED_SRGB,
  BT709,
  BT2020,
  DCI_P3,
  DISPLAY_P3,
  NTSC_1953,
  SMPTE_C,
  ADOBE_RGB,
  PRO_PHOTO_RGB,
  ACES,
  ACESCG,
  CIE_XYZ,
  CIE_LAB
};

// Sensor sensitivity
const uint32_t EmulatedFrameSource::kRegularSceneHandshake = 1;
const uint32_t EmulatedFrameSource::kReducedSceneHandshake = 2;

const float EmulatedFrameSource::kSaturationVoltage = 0.520f;
const uint32_t EmulatedFrameSource::kSaturationElectrons = 2000;
const float EmulatedFrameSource::kVoltsPerLuxSecond = 0.100f;

const float EmulatedFrameSource::kElectronsPerLuxSecond =
    EmulatedFrameSource::kSaturationElectrons /
    EmulatedFrameSource::kSaturationVoltage *
    EmulatedFrameSource::kVoltsPerLuxSecond;

const float EmulatedFrameSource::kReadNoiseStddevBeforeGain =
    1.177;  // in electrons
const float EmulatedFrameSource::kReadNoiseStddevAfterGain =
    2.100;  // in digital counts
const float EmulatedFrameSource::kReadNoiseVarBeforeGain =
    EmulatedFrameSource::kReadNoiseStddevBeforeGain *
    EmulatedFrameSource::kReadNoiseStddevBeforeGain;
const float EmulatedFrameSource::kReadNoiseVarAfterGain =
    EmulatedFrameSource::kReadNoiseStddevAfterGain *
    EmulatedFrameSource::kReadNoiseStddevAfterGain;

const int32_t EmulatedFrameSource::kFixedBitPrecision = 64;  // 6-bit
// In fixed-point math, saturation point of sensor after gain
const int32_t EmulatedFrameSource::kSaturationPoint = kFixedBitPrecision * 255;

// All XY matrix coefficients sourced from
// https://developer.android.com/reference/kotlin/android/graphics/ColorSpace.Named
// and XYZ coefficients calculated using the method found in
// ColorSpace.Rgb.computeXyzMatrix
struct XyzMatrix {
  float xR = 3.2406f;
  float yR = -1.5372f;
  float zR = -0.4986f;
  float xG = -0.9689f;
  float yG = 1.8758f;
  float zG = 0.0415f;
  float xB = 0.0557f;
  float yB = -0.2040f;
  float zB = 1.0570f;
};

static const XyzMatrix kSrgbXyzMatrix = {3.2406f,  -1.5372f, -0.4986f,
                                         -0.9689f, 1.8758f,  0.0415f,
                                         0.0557f,  -0.2040f, 1.0570f};

static const XyzMatrix kDisplayP3Matrix = {2.4931f,  -0.9316f, -0.4023f,
                                           -0.8291f, 1.7627f,  0.0234f,
                                           0.0361f,  -0.0761f, 0.9570f};

static const XyzMatrix kBt709Matrix = {3.2410f,  -1.5374f, -0.4986f,
                                       -0.9692f, 1.8760f,  0.0416f,
                                       0.0556f,  -0.2040f, 1.0570f};

static const XyzMatrix kBt2020Matrix = {1.7167f,  -0.3556f, -0.2534f,
                                        -0.6666f, 1.6164f,  0.0158f,
                                        0.0177f,  -0.0428f, 0.9421f};

/** A few utility functions for math, normal distributions */

// Take advantage of IEEE floating-point format to calculate an approximate
// square root. Accurate to within +-3.6%
static float sqrtf_approx(float r) {
  // Modifier is based on IEEE floating-point representation; the
  // manipulations boil down to finding approximate log2, dividing by two, and
  // then inverting the log2. A bias is added to make the relative error
  // symmetric about the real answer.
  const int32_t modifier = 0x1FBB4000;

  int32_t r_i;
  memcpy(&r_i, &r, sizeof(r));
  r_i = (r_i >> 1) + modifier;

  float result;
  memcpy(&result, &r_i, sizeof(result));
  return result;
}

EmulatedFrameSource::EmulatedFrameSource(const LogicalCharacteristics& chars,
                                         uint32_t camera_id)
    : chars_(std::make_unique<LogicalCharacteristics>(chars)) {
  InitializeGammaTables();
  const auto& device_chars = chars_->at(camera_id);
  scene_ = std::make_unique<EmulatedScene>(
      device_chars.full_res_width, device_chars.full_res_height,
      kElectronsPerLuxSecond, device_chars.orientation,
      device_chars.is_front_facing);
}

EmulatedFrameSource::~EmulatedFrameSource() {
}

void EmulatedFrameSource::InitializeGammaTables() {
  gamma_table_sRGB_.resize(kSaturationPoint + 1);
  gamma_table_smpte170m_.resize(kSaturationPoint + 1);
  gamma_table_hlg_.resize(kSaturationPoint + 1);
  for (int32_t i = 0; i <= kSaturationPoint; i++) {
    gamma_table_sRGB_[i] = ApplysRGBGamma(i, kSaturationPoint);
    gamma_table_smpte170m_[i] = ApplySMPTE170MGamma(i, kSaturationPoint);
    gamma_table_hlg_[i] = ApplyHLGGamma(i, kSaturationPoint);
  }
}

void EmulatedFrameSource::ConfigureScene(uint32_t camera_id, nsecs_t timestamp,
                                         const SensorSettings& settings) {
  const auto& chars = chars_->at(camera_id);
  scene_->Initialize(chars.full_res_width, chars.full_res_height,
                     kElectronsPerLuxSecond);
  scene_->SetExposureDuration((float)settings.exposure_time / 1e9);
  scene_->SetColorFilterXYZ(
      chars.color_filter.rX, chars.color_filter.rY, chars.color_filter.rZ,
      chars.color_filter.grX, chars.color_filter.grY, chars.color_filter.grZ,
      chars.color_filter.gbX, chars.color_filter.gbY, chars.color_filter.gbZ,
      chars.color_filter.bX, chars.color_filter.bY, chars.color_filter.bZ);
  scene_->SetTestPattern(settings.test_pattern_mode ==
                         ANDROID_SENSOR_TEST_PATTERN_MODE_SOLID_COLOR);
  scene_->SetTestPatternData(const_cast<uint32_t*>(settings.test_pattern_data));
  scene_->SetScreenRotation(settings.screen_rotation);

  uint32_t handshake_divider =
      (settings.video_stab == ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_ON) ||
              (settings.video_stab ==
               ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_PREVIEW_STABILIZATION)
          ? kReducedSceneHandshake
          : kRegularSceneHandshake;
  scene_->CalculateScene(timestamp, handshake_divider);
}

status_t EmulatedFrameSource::ProduceFrame(uint32_t camera_id, nsecs_t timestamp,
                                           const SensorSettings& settings,
                                           SensorBuffer* buffer,
                                           const SensorBuffer* input_buffer) {
  ATRACE_CALL();
  if (buffer == nullptr) {
    return BAD_VALUE;
  }

  ConfigureScene(camera_id, timestamp, settings);

  const auto& chars = chars_->at(camera_id);

  bool treat_as_reprocess = (input_buffer != nullptr);
  if (chars.quad_bayer_sensor && treat_as_reprocess &&
      input_buffer->format == PixelFormat::RAW16 &&
      buffer->format != PixelFormat::RAW16) {
    treat_as_reprocess = false;
  }

  switch (buffer->format) {
    case PixelFormat::RAW16: {
      if (buffer->is_input) {
        ALOGE("%s: Reprocess requests with input RAW16 not supported here!",
              __FUNCTION__);
        return BAD_VALUE;
      }

      if (treat_as_reprocess) {
        if (!chars.quad_bayer_sensor) {
          ALOGE("%s: Reprocess requests with output format %x not supported!",
                __FUNCTION__, buffer->format);
          return BAD_VALUE;
        }
        if (input_buffer->width != buffer->width ||
            input_buffer->height != buffer->height) {
          ALOGE(
              "%s: RAW16 input dimensions %ux%u don't match output buffer "
              "dimensions %ux%u",
              __FUNCTION__, input_buffer->width, input_buffer->height,
              buffer->width, buffer->height);
          return BAD_VALUE;
        }
        return RemosaicRAW16Image((uint16_t*)input_buffer->plane.img.img,
                                  (uint16_t*)buffer->plane.img.img,
                                  buffer->plane.img.stride_in_bytes, chars);
      }

      uint64_t min_full_res_raw_size =
          2 * chars.full_res_width * chars.full_res_height;
      uint64_t min_default_raw_size = 2 * chars.width * chars.height;
      bool max_res_request = (settings.sensor_pixel_mode ==
                              ANDROID_SENSOR_PIXEL_MODE_MAXIMUM_RESOLUTION);
      bool default_mode_for_qb = chars.quad_bayer_sensor && !max_res_request;
      size_t buffer_size = buffer->plane.img.buffer_size;

      if (default_mode_for_qb) {
        if (buffer_size < min_default_raw_size) {
          ALOGE(
              "%s: Output buffer size too small for RAW capture in default "
              "mode, expected %" PRIu64 ", got %zu, for camera id %d",
              __FUNCTION__, min_default_raw_size, buffer_size, camera_id);
          return BAD_VALUE;
        }
        if (settings.zoom_ratio > 2.0f &&
            (buffer->use_case ==
             ANDROID_SCALER_AVAILABLE_STREAM_USE_CASES_CROPPED_RAW)) {
          CaptureRawInSensorZoom(buffer->plane.img.img,
                                 buffer->plane.img.stride_in_bytes,
                                 settings.gain, chars);
        } else {
          CaptureRawBinned(buffer->plane.img.img,
                           buffer->plane.img.stride_in_bytes, settings.gain,
                           chars);
        }
      } else {
        if (buffer_size < min_full_res_raw_size) {
          ALOGE(
              "%s: Output buffer size too small for RAW capture in max res "
              "mode, expected %" PRIu64 ", got %zu, for camera id %d",
              __FUNCTION__, min_full_res_raw_size, buffer_size, camera_id);
          return BAD_VALUE;
        }
        CaptureRawFullRes(buffer->plane.img.img,
                          buffer->plane.img.stride_in_bytes, settings.gain,
                          chars);
      }
      break;
    }
    case PixelFormat::RGB_888:
      if (treat_as_reprocess) {
        ALOGE("%s: Reprocess requests with output format %x not supported!",
              __FUNCTION__, buffer->format);
        return BAD_VALUE;
      }
      if (buffer->color_space !=
          ANDROID_REQUEST_AVAILABLE_COLOR_SPACE_PROFILES_MAP_UNSPECIFIED) {
        CalculateRgbRgbMatrix(buffer->color_space, chars);
      }
      CaptureRGB(buffer->plane.img.img, buffer->width, buffer->height,
                 buffer->plane.img.stride_in_bytes, RGBLayout::RGB,
                 settings.gain, buffer->color_space, chars);
      break;
    case PixelFormat::RGBA_8888:
      if (treat_as_reprocess) {
        ALOGE("%s: Reprocess requests with output format %x not supported!",
              __FUNCTION__, buffer->format);
        return BAD_VALUE;
      }
      if (buffer->color_space !=
          ANDROID_REQUEST_AVAILABLE_COLOR_SPACE_PROFILES_MAP_UNSPECIFIED) {
        CalculateRgbRgbMatrix(buffer->color_space, chars);
      }
      CaptureRGB(buffer->plane.img.img, buffer->width, buffer->height,
                 buffer->plane.img.stride_in_bytes, RGBLayout::RGBA,
                 settings.gain, buffer->color_space, chars);
      break;
    case PixelFormat::YCRCB_420_SP:
    case PixelFormat::YCBCR_420_888:
    case PixelFormat::YCBCR_P010: {
      if (buffer->color_space !=
          ANDROID_REQUEST_AVAILABLE_COLOR_SPACE_PROFILES_MAP_UNSPECIFIED) {
        CalculateRgbRgbMatrix(buffer->color_space, chars);
      }

      YUV420Frame output_frame;
      output_frame.width = buffer->width;
      output_frame.height = buffer->height;
      output_frame.planes = buffer->plane.img_y_crcb;
      // Output frame struct also has color space member
      output_frame.color_space = buffer->color_space;

      bool rotate =
          settings.rotate_and_crop == ANDROID_SCALER_ROTATE_AND_CROP_90;
      ProcessType process_type;
      if (treat_as_reprocess) {
        process_type = REPROCESS;
      } else {
        process_type = (settings.edge_mode == ANDROID_EDGE_MODE_HIGH_QUALITY)
                           ? HIGH_QUALITY
                           : REGULAR;
      }

      YUV420Frame input_frame;
      if (treat_as_reprocess) {
        input_frame.width = input_buffer->width;
        input_frame.height = input_buffer->height;
        input_frame.planes = input_buffer->plane.img_y_crcb;
      }

      status_t ret = ProcessYUV420(input_frame, output_frame, settings.gain,
                                   process_type, settings.zoom_ratio, rotate,
                                   buffer->color_space, chars);
      if (ret != OK) return ret;
      break;
    }
    case PixelFormat::Y16:
      if (treat_as_reprocess) {
        ALOGE("%s: Reprocess requests with output format %x not supported!",
              __FUNCTION__, buffer->format);
        return BAD_VALUE;
      }
      if (buffer->dataSpace == HAL_DATASPACE_DEPTH) {
        CaptureDepth(buffer->plane.img.img, settings.gain, buffer->width,
                     buffer->height, buffer->plane.img.stride_in_bytes, chars);
      } else {
        ALOGE("%s: Format %x with dataspace %x is TODO", __FUNCTION__,
              buffer->format, buffer->dataSpace);
        return BAD_VALUE;
      }
      break;
    default:
      ALOGE("%s: Unknown format %x", __FUNCTION__, buffer->format);
      return BAD_VALUE;
  }

  return OK;
}

void EmulatedFrameSource::CalculateAndAppendNoiseProfile(
    float gain /*in ISO*/, float max_raw_value,
    HalCameraMetadata* result /*out*/) {
  if (result != nullptr) {
    float base_gain_factor = GetBaseGainFactor(max_raw_value);
    float total_gain = gain / 100.0 * base_gain_factor;
    float noise_var_gain = total_gain * total_gain;
    float read_noise_var =
        kReadNoiseVarBeforeGain * noise_var_gain + kReadNoiseVarAfterGain;
    // Noise profile is the same across all 4 CFA channels
    double noise_profile[2 * 4] = {
        noise_var_gain, read_noise_var, noise_var_gain, read_noise_var,
        noise_var_gain, read_noise_var, noise_var_gain, read_noise_var};
    result->Set(ANDROID_SENSOR_NOISE_PROFILE, noise_profile,
                ARRAY_SIZE(noise_profile));
  }
}

EmulatedScene::ColorChannels EmulatedFrameSource::GetQuadBayerColor(uint32_t x,
                                                                    uint32_t y) {
  // Row within larger set of quad bayer filter
  uint32_t row_mod = y % 4;
  // Column within larger set of quad bayer filter
  uint32_t col_mod = x % 4;

  // Row is within the left quadrants of a quad bayer sensor
  if (row_mod < 2) {
    if (col_mod < 2) {
      return EmulatedScene::ColorChannels::R;
    }
    return EmulatedScene::ColorChannels::Gr;
  } else {
    if (col_mod < 2) {
      return EmulatedScene::ColorChannels::Gb;
    }
    return EmulatedScene::ColorChannels::B;
  }
}

void EmulatedFrameSource::RemosaicQuadBayerBlock(uint16_t* img_in,
                                                 uint16_t* img_out, int xstart,
                                                 int ystart,
                                                 int row_stride_in_bytes) {
  uint32_t quad_block_copy_idx_map[16] = {0, 2, 1, 3, 8,  10, 6,  11,
                                          4, 9, 5, 7, 12, 14, 13, 15};
  uint16_t quad_block_copy[16];
  uint32_t i = 0;
  for (uint32_t row = 0; row < 4; row++) {
    uint16_t* quad_bayer_row =
        img_in + (ystart + row) * (row_stride_in_bytes / 2) + xstart;
    for (uint32_t j = 0; j < 4; j++, i++) {
      quad_block_copy[i] = quad_bayer_row[j];
    }
  }

  for (uint32_t row = 0; row < 4; row++) {
    uint16_t* regular_bayer_row =
        img_out + (ystart + row) * (row_stride_in_bytes / 2) + xstart;
    for (uint32_t j = 0; j < 4; j++) {
      uint32_t idx = quad_block_copy_idx_map[row + 4 * j];
      regular_bayer_row[j] = quad_block_copy[idx];
    }
  }
}

status_t EmulatedFrameSource::RemosaicRAW16Image(
    uint16_t* img_in, uint16_t* img_out, size_t row_stride_in_bytes,
    const SensorCharacteristics& chars) {
  if (chars.full_res_width % 2 != 0 || chars.full_res_height % 2 != 0) {
    ALOGE(
        "%s RAW16 Image with quad CFA, height %zu and width %zu, not multiples "
        "of 4",
        __FUNCTION__, chars.full_res_height, chars.full_res_width);
    return BAD_VALUE;
  }
  for (uint32_t i = 0; i < chars.full_res_width; i += 4) {
    for (uint32_t j = 0; j < chars.full_res_height; j += 4) {
      RemosaicQuadBayerBlock(img_in, img_out, i, j, row_stride_in_bytes);
    }
  }
  return OK;
}

void EmulatedFrameSource::CaptureRawBinned(uint8_t* img,
                                           size_t row_stride_in_bytes,
                                           uint32_t gain,
                                           const SensorCharacteristics& chars) {
  CaptureRaw(img, row_stride_in_bytes, gain, chars, /*in_sensor_zoom*/ false,
             /*binned*/ true);
  return;
}

void EmulatedFrameSource::CaptureRawInSensorZoom(
    uint8_t* img, size_t row_stride_in_bytes, uint32_t gain,
    const SensorCharacteristics& chars) {
  CaptureRaw(img, row_stride_in_bytes, gain, chars, /*in_sensor_zoom*/ true,
             /*binned*/ false);
  return;
}

void EmulatedFrameSource::CaptureRawFullRes(uint8_t* img,
                                            size_t row_stride_in_bytes,
                                            uint32_t gain,
                                            const SensorCharacteristics& chars) {
  CaptureRaw(img, row_stride_in_bytes, gain, chars, /*inSensorZoom*/ false,
             /*binned*/ false);
  return;
}

void EmulatedFrameSource::CaptureRaw(uint8_t* img, size_t row_stride_in_bytes,
                                     uint32_t gain,
                                     const SensorCharacteristics& chars,
                                     bool in_sensor_zoom, bool binned) {
  ATRACE_CALL();
  if (in_sensor_zoom && binned) {
    ALOGE("%s: Can't perform in-sensor zoom in binned mode", __FUNCTION__);
    return;
  }
  float total_gain = gain / 100.0 * GetBaseGainFactor(chars.max_raw_value);
  float noise_var_gain = total_gain * total_gain;
  float read_noise_var =
      kReadNoiseVarBeforeGain * noise_var_gain + kReadNoiseVarAfterGain;

  scene_->SetReadoutPixel(0, 0);
  // RGGB
  int bayer_select[4] = {EmulatedScene::R, EmulatedScene::Gr, EmulatedScene::Gb,
                         EmulatedScene::B};
  const float raw_zoom_ratio = in_sensor_zoom ? 2.0f : 1.0f;
  unsigned int image_width =
      in_sensor_zoom || binned ? chars.width : chars.full_res_width;
  unsigned int image_height =
      in_sensor_zoom || binned ? chars.height : chars.full_res_height;
  const float norm_left_top = 0.5f - 0.5f / raw_zoom_ratio;
  for (unsigned int out_y = 0; out_y < image_height; out_y++) {
    int* bayer_row = bayer_select + (out_y & 0x1) * 2;
    uint16_t* px = (uint16_t*)img + out_y * (row_stride_in_bytes / 2);

    float norm_y = out_y / (image_height * raw_zoom_ratio);
    int y = static_cast<int>(chars.full_res_height * (norm_left_top + norm_y));
    y = std::min(std::max(y, 0), (int)chars.full_res_height - 1);

    for (unsigned int out_x = 0; out_x < image_width; out_x++) {
      int color_idx = chars.quad_bayer_sensor && !(in_sensor_zoom || binned)
                          ? GetQuadBayerColor(out_x, out_y)
                          : bayer_row[out_x & 0x1];
      float norm_x = out_x / (image_width * raw_zoom_ratio);
      int x = static_cast<int>(chars.full_res_width * (norm_left_top + norm_x));
      x = std::min(std::max(x, 0), (int)chars.full_res_width - 1);

      uint32_t electron_count;
      scene_->SetReadoutPixel(x, y);
      electron_count = scene_->GetPixelElectrons()[color_idx];

      // TODO: Better pixel saturation curve?
      electron_count = (electron_count < kSaturationElectrons)
                           ? electron_count
                           : kSaturationElectrons;

      // TODO: Better A/D saturation curve?
      uint16_t raw_count = electron_count * total_gain;
      raw_count =
          (raw_count < chars.max_raw_value) ? raw_count : chars.max_raw_value;

      // Calculate noise value
      // TODO: Use more-correct Gaussian instead of uniform noise
      float photon_noise_var = electron_count * noise_var_gain;
      float noise_stddev = sqrtf_approx(read_noise_var + photon_noise_var);
      // Scaled to roughly match gaussian/uniform noise stddev
      float noise_sample = rand_r(&rand_seed_) * (2.5 / (1.0 + RAND_MAX)) - 1.25;

      raw_count += chars.black_level_pattern[color_idx];
      raw_count += noise_stddev * noise_sample;

      *px++ = raw_count;
    }
    // TODO: Handle this better
    // simulatedTime += mRowReadoutTime;
  }
  ALOGVV("Raw sensor image captured");
}

void EmulatedFrameSource::CaptureRGB(uint8_t* img, uint32_t width,
                                     uint32_t height, uint32_t stride,
                                     RGBLayout layout, uint32_t gain,
                                     int32_t color_space,
                                     const SensorCharacteristics& chars) {
  ATRACE_CALL();
  float total_gain = gain / 100.0 * GetBaseGainFactor(chars.max_raw_value);
  // In fixed-point math, calculate total scaling from electrons to 8bpp
  int scale64x = 64 * total_gain * 255 / chars.max_raw_value;
  uint32_t inc_h = ceil((float)chars.full_res_width / width);
  uint32_t inc_v = ceil((float)chars.full_res_height / height);

  for (unsigned int y = 0, outy = 0; y < chars.full_res_height;
       y += inc_v, outy++) {
    scene_->SetReadoutPixel(0, y);
    uint8_t* px = img + outy * stride;
    for (unsigned int x = 0; x < chars.full_res_width; x += inc_h) {
      uint32_t r_count, g_count, b_count;
      // TODO: Perfect demosaicing is a cheat
      const uint32_t* pixel = scene_->GetPixelElectrons();
      r_count = pixel[EmulatedScene::R] * scale64x;
      g_count = pixel[EmulatedScene::Gr] * scale64x;
      b_count = pixel[EmulatedScene::B] * scale64x;

      if (color_space !=
          ANDROID_REQUEST_AVAILABLE_COLOR_SPACE_PROFILES_MAP_UNSPECIFIED) {
        RgbToRgb(&r_count, &g_count, &b_count);
      }

      uint8_t r = r_count < 255 * 64 ? r_count / 64 : 255;
      uint8_t g = g_count < 255 * 64 ? g_count / 64 : 255;
      uint8_t b = b_count < 255 * 64 ? b_count / 64 : 255;
      switch (layout) {
        case RGB:
          *px++ = r;
          *px++ = g;
          *px++ = b;
          break;
        case RGBA:
          *px++ = r;
          *px++ = g;
          *px++ = b;
          *px++ = 255;
          break;
        case ARGB:
          *px++ = 255;
          *px++ = r;
          *px++ = g;
          *px++ = b;
          break;
        default:
          ALOGE("%s: RGB layout: %d not supported", __FUNCTION__, layout);
          return;
      }
      for (unsigned int j = 1; j < inc_h; j++) scene_->GetPixelElectrons();
    }
  }
  ALOGVV("RGB sensor image captured");
}

void EmulatedFrameSource::CaptureYUV420(YCbCrPlanes yuv_layout, uint32_t width,
                                        uint32_t height, uint32_t gain,
                                        float zoom_ratio, bool rotate,
                                        int32_t color_space,
                                        const SensorCharacteristics& chars) {
  ATRACE_CALL();
  float total_gain = gain / 100.0 * GetBaseGainFactor(chars.max_raw_value);
  // Using fixed-point math with 6 bits of fractional precision.
  // In fixed-point math, calculate total scaling from electrons to 8bpp
  const int scale64x =
      kFixedBitPrecision * total_gain * 255 / chars.max_raw_value;
  // Fixed-point coefficients for RGB-YUV transform
  // Based on JFIF RGB->YUV transform.
  // Cb/Cr offset scaled by 64x twice since they're applied post-multiply
  const int rgb_to_y[] = {19, 37, 7};
  const int rgb_to_cb[] = {-10, -21, 32, 524288};
  const int rgb_to_cr[] = {32, -26, -5, 524288};
  // Scale back to 8bpp non-fixed-point
  const int scale_out = 64;
  const int scale_out_sq = scale_out * scale_out;  // after multiplies

  // inc = how many pixels to skip while reading every next pixel
  const float aspect_ratio = static_cast<float>(width) / height;

  // precalculate normalized coordinates and dimensions
  const float norm_left_top = 0.5f - 0.5f / zoom_ratio;
  const float norm_rot_top = norm_left_top;
  const float norm_width = 1 / zoom_ratio;
  const float norm_rot_width = norm_width / aspect_ratio;
  const float norm_rot_height = norm_width;
  const float norm_rot_left =
      norm_left_top + (norm_width + norm_rot_width) * 0.5f;

  for (unsigned int out_y = 0; out_y < height; out_y++) {
    uint8_t* px_y = yuv_layout.img_y + out_y * yuv_layout.y_stride;
    uint8_t* px_cb = yuv_layout.img_cb + (out_y / 2) * yuv_layout.cbcr_stride;
    uint8_t* px_cr = yuv_layout.img_cr + (out_y / 2) * yuv_layout.cbcr_stride;

    for (unsigned int out_x = 0; out_x < width; out_x++) {
      int x, y;
      if (rotate) {
        float norm_x = static_cast<float>(out_x) / width;
        float norm_y = static_cast<float>(out_y) / height;
        x = static_cast<int>(chars.full_res_width *
                             (norm_rot_left - norm_y * norm_rot_width));
        y = static_cast<int>(chars.full_res_height *
                             (norm_rot_top + norm_x * norm_rot_height));
      } else {
        float norm_x = out_x / (width * zoom_ratio);
        float norm_y = out_y / (height * zoom_ratio);
        x = static_cast<int>(chars.full_res_width * (norm_left_top + norm_x));
        y = static_cast<int>(chars.full_res_height * (norm_left_top + norm_y));
      }
      x = std::min(std::max(x, 0), (int)chars.full_res_width - 1);
      y = std::min(std::max(y, 0), (int)chars.full_res_height - 1);
      scene_->SetReadoutPixel(x, y);

      uint32_t r_count, g_count, b_count;
      // TODO: Perfect demosaicing is a cheat
      const uint32_t* pixel = rotate ? scene_->GetPixelElectronsColumn()
                                     : scene_->GetPixelElectrons();
      r_count = pixel[EmulatedScene::R] * scale64x;
      g_count = pixel[EmulatedScene::Gr] * scale64x;
      b_count = pixel[EmulatedScene::B] * scale64x;

      if (color_space !=
          ANDROID_REQUEST_AVAILABLE_COLOR_SPACE_PROFILES_MAP_UNSPECIFIED) {
        RgbToRgb(&r_count, &g_count, &b_count);
      }

      r_count = r_count < kSaturationPoint ? r_count : kSaturationPoint;
      g_count = g_count < kSaturationPoint ? g_count : kSaturationPoint;
      b_count = b_count < kSaturationPoint ? b_count : kSaturationPoint;

      // Gamma correction
      r_count = GammaTable(r_count, color_space);
      g_count = GammaTable(g_count, color_space);
      b_count = GammaTable(b_count, color_space);

      uint8_t y8 = (rgb_to_y[0] * r_count + rgb_to_y[1] * g_count +
                    rgb_to_y[2] * b_count) /
                   scale_out_sq;
      if (yuv_layout.bytesPerPixel == 1) {
        *px_y = y8;
      } else if (yuv_layout.bytesPerPixel == 2) {
        *(reinterpret_cast<uint16_t*>(px_y)) = htole16(y8 << 8);
      } else {
        ALOGE("%s: Unsupported bytes per pixel value: %zu", __func__,
              yuv_layout.bytesPerPixel);
        return;
      }
      px_y += yuv_layout.bytesPerPixel;

      if (out_y % 2 == 0 && out_x % 2 == 0) {
        uint8_t cb8 = (rgb_to_cb[0] * r_count + rgb_to_cb[1] * g_count +
                       rgb_to_cb[2] * b_count + rgb_to_cb[3]) /
                      scale_out_sq;
        uint8_t cr8 = (rgb_to_cr[0] * r_count + rgb_to_cr[1] * g_count +
                       rgb_to_cr[2] * b_count + rgb_to_cr[3]) /
                      scale_out_sq;
        if (yuv_layout.bytesPerPixel == 1) {
          *px_cb = cb8;
          *px_cr = cr8;
        } else if (yuv_layout.bytesPerPixel == 2) {
          *(reinterpret_cast<uint16_t*>(px_cb)) = htole16(cb8 << 8);
          *(reinterpret_cast<uint16_t*>(px_cr)) = htole16(cr8 << 8);
        } else {
          ALOGE("%s: Unsupported bytes per pixel value: %zu", __func__,
                yuv_layout.bytesPerPixel);
          return;
        }
        px_cr += yuv_layout.cbcr_step;
        px_cb += yuv_layout.cbcr_step;
      }
    }
  }
  ALOGVV("YUV420 sensor image captured");
}

void EmulatedFrameSource::CaptureDepth(uint8_t* img, uint32_t gain,
                                       uint32_t width, uint32_t height,
                                       uint32_t stride,
                                       const SensorCharacteristics& chars) {
  ATRACE_CALL();
  float total_gain = gain / 100.0 * GetBaseGainFactor(chars.max_raw_value);
  // In fixed-point math, calculate scaling factor to 13bpp millimeters
  int scale64x = 64 * total_gain * 8191 / chars.max_raw_value;
  uint32_t inc_h = ceil((float)chars.full_res_width / width);
  uint32_t inc_v = ceil((float)chars.full_res_height / height);

  for (unsigned int y = 0, out_y = 0; y < chars.full_res_height;
       y += inc_v, out_y++) {
    scene_->SetReadoutPixel(0, y);
    uint16_t* px = (uint16_t*)(img + (out_y * stride));
    for (unsigned int x = 0; x < chars.full_res_width; x += inc_h) {
      uint32_t depth_count;
      // TODO: Make up real depth scene instead of using green channel
      // as depth
      const uint32_t* pixel = scene_->GetPixelElectrons();
      depth_count = pixel[EmulatedScene::Gr] * scale64x;

      *px++ = depth_count < 8191 * 64 ? depth_count / 64 : 0;
      for (unsigned int j = 1; j < inc_h; j++) scene_->GetPixelElectrons();
    }
    // TODO: Handle this better
    // simulatedTime += mRowReadoutTime;
  }
  ALOGVV("Depth sensor image captured");
}

status_t EmulatedFrameSource::ProcessYUV420(
    const YUV420Frame& input, const YUV420Frame& output, uint32_t gain,
    ProcessType process_type, float zoom_ratio, bool rotate_and_crop,
    int32_t color_space, const SensorCharacteristics& chars) {
  ATRACE_CALL();
  size_t input_width, input_height;
  YCbCrPlanes input_planes, output_planes;
  std::vector<uint8_t> temp_yuv, temp_output_uv, temp_input_uv;

  // Overwrite HIGH_QUALITY to REGULAR for Emulator if property
  // ro.boot.qemu.camera_hq_edge_processing is false;
  if (process_type == HIGH_QUALITY &&
      !property_get_bool("ro.boot.qemu.camera_hq_edge_processing", false)) {
    process_type = REGULAR;
  }

  size_t bytes_per_pixel = output.planes.bytesPerPixel;
  switch (process_type) {
    case HIGH_QUALITY:
      CaptureYUV420(output.planes, output.width, output.height, gain,
                    zoom_ratio, rotate_and_crop, color_space, chars);
      return OK;
    case REPROCESS:
      input_width = input.width;
      input_height = input.height;
      input_planes = input.planes;

      // libyuv only supports planar YUV420 during scaling.
      // Split the input U/V plane in separate planes if needed.
      if (input_planes.cbcr_step == 2) {
        temp_input_uv.resize(input_width * input_height / 2);
        auto temp_uv_buffer = temp_input_uv.data();
        input_planes.img_cb = temp_uv_buffer;
        input_planes.img_cr = temp_uv_buffer + (input_width * input_height) / 4;
        input_planes.cbcr_stride = input_width / 2;
        if (input.planes.img_cb < input.planes.img_cr) {
          libyuv::SplitUVPlane(input.planes.img_cb, input.planes.cbcr_stride,
                               input_planes.img_cb, input_planes.cbcr_stride,
                               input_planes.img_cr, input_planes.cbcr_stride,
                               input_width / 2, input_height / 2);
        } else {
          libyuv::SplitUVPlane(input.planes.img_cr, input.planes.cbcr_stride,
                               input_planes.img_cr, input_planes.cbcr_stride,
                               input_planes.img_cb, input_planes.cbcr_stride,
                               input_width / 2, input_height / 2);
        }
      }
      break;
    case REGULAR:
    default:
      // Generate the smallest possible frame with the expected AR and
      // then scale using libyuv.
      float aspect_ratio = static_cast<float>(output.width) / output.height;
      zoom_ratio = std::max(1.f, zoom_ratio);
      input_width = EmulatedScene::kSceneWidth * aspect_ratio;
      input_height = EmulatedScene::kSceneHeight;
      temp_yuv.reserve((input_width * input_height * 3 * bytes_per_pixel) / 2);
      auto temp_yuv_buffer = temp_yuv.data();
      input_planes = {
          .img_y = temp_yuv_buffer,
          .img_cb =
              temp_yuv_buffer + input_width * input_height * bytes_per_pixel,
          .img_cr = temp_yuv_buffer +
                    (input_width * input_height * bytes_per_pixel * 5) / 4,
          .y_stride = static_cast<uint32_t>(input_width * bytes_per_pixel),
          .cbcr_stride =
              static_cast<uint32_t>(input_width * bytes_per_pixel) / 2,
          .cbcr_step = 1,
          .bytesPerPixel = bytes_per_pixel};
      CaptureYUV420(input_planes, input_width, input_height, gain, zoom_ratio,
                    rotate_and_crop, color_space, chars);
  }

  output_planes = output.planes;
  // libyuv only supports planar YUV420 during scaling.
  // Treat the output UV space as planar first and then
  // interleave in the second step.
  if (output_planes.cbcr_step == 2) {
    temp_output_uv.resize(output.width * output.height * bytes_per_pixel / 2);
    auto temp_uv_buffer = temp_output_uv.data();
    output_planes.img_cb = temp_uv_buffer;
    output_planes.img_cr =
        temp_uv_buffer + output.width * output.height * bytes_per_pixel / 4;
    output_planes.cbcr_stride = output.width * bytes_per_pixel / 2;
  }

  // NOTE: libyuv takes strides in pixels, not bytes.
  int ret = 0;
  if (bytes_per_pixel == 2) {
    ret = I420Scale_16((const uint16_t*)input_planes.img_y,
                       input_planes.y_stride / bytes_per_pixel,
                       (const uint16_t*)input_planes.img_cb,
                       input_planes.cbcr_stride / bytes_per_pixel,
                       (const uint16_t*)input_planes.img_cr,
                       input_planes.cbcr_stride / bytes_per_pixel, input_width,
                       input_height, (uint16_t*)output_planes.img_y,
                       output_planes.y_stride / bytes_per_pixel,
                       (uint16_t*)output_planes.img_cb,
                       output_planes.cbcr_stride / bytes_per_pixel,
                       (uint16_t*)output_planes.img_cr,
                       output_planes.cbcr_stride / bytes_per_pixel,
                       output.width, output.height, libyuv::kFilterNone);
  } else {
    ret = I420Scale(input_planes.img_y, input_planes.y_stride,
                    input_planes.img_cb, input_planes.cbcr_stride,
                    input_planes.img_cr, input_planes.cbcr_stride, input_width,
                    input_height, output_planes.img_y, output_planes.y_stride,
                    output_planes.img_cb, output_planes.cbcr_stride,
                    output_planes.img_cr, output_planes.cbcr_stride,
                    output.width, output.height, libyuv::kFilterNone);
  }
  if (ret != 0) {
    ALOGE("%s: Failed during YUV scaling: %d", __FUNCTION__, ret);
    return ret;
  }

  // Merge U/V Planes for the interleaved case
  if (output_planes.cbcr_step == 2) {
    if (output.planes.img_cb < output.planes.img_cr) {
      if (bytes_per_pixel == 2) {
        libyuv::MergeUVPlane_16((const uint16_t*)output_planes.img_cb,
                                output_planes.cbcr_stride / bytes_per_pixel,
                                (const uint16_t*)output_planes.img_cr,
                                output_planes.cbcr_stride / bytes_per_pixel,
                                (uint16_t*)output.planes.img_cb,
                                output.planes.cbcr_stride / bytes_per_pixel,
                                output.width / 2, output.height / 2,
                                /*depth*/ 16);
      } else {
        libyuv::MergeUVPlane(output_planes.img_cb, output_planes.cbcr_stride,
                             output_planes.img_cr, output_planes.cbcr_stride,
                             output.planes.img_cb, output.planes.cbcr_stride,
                             output.width / 2, output.height / 2);
      }
    } else {
      if (bytes_per_pixel == 2) {
        libyuv::MergeUVPlane_16((const uint16_t*)output_planes.img_cr,
                                output_planes.cbcr_stride / bytes_per_pixel,
                                (const uint16_t*)output_planes.img_cb,
                                output_planes.cbcr_stride / bytes_per_pixel,
                                (uint16_t*)output.planes.img_cr,
                                output.planes.cbcr_stride / bytes_per_pixel,
                                output.width / 2, output.height / 2,
                                /*depth*/ 16);
      } else {
        libyuv::MergeUVPlane(output_planes.img_cr, output_planes.cbcr_stride,
                             output_planes.img_cb, output_planes.cbcr_stride,
                             output.planes.img_cr, output.planes.cbcr_stride,
                             output.width / 2, output.height / 2);
      }
    }
  }

  return ret;
}

int32_t EmulatedFrameSource::ApplysRGBGamma(int32_t value, int32_t saturation) {
  float n_value = (static_cast<float>(value) / saturation);
  n_value = (n_value <= 0.0031308f)
                ? n_value * 12.92f
                : 1.055f * pow(n_value, 0.4166667f) - 0.055f;
  return n_value * saturation;
}

int32_t EmulatedFrameSource::ApplySMPTE170MGamma(int32_t value,
                                                 int32_t saturation) {
  float n_value = (static_cast<float>(value) / saturation);
  n_value = (n_value <= 0.018f) ? n_value * 4.5f
                                : 1.099f * pow(n_value, 0.45f) - 0.099f;
  return n_value * saturation;
}

int32_t EmulatedFrameSource::ApplyST2084Gamma(int32_t value,
                                              int32_t saturation) {
  float n_value = (static_cast<float>(value) / saturation);
  float c2 = 32.f * 2413.f / 4096.f;
  float c3 = 32.f * 2392.f / 4096.f;
  float c1 = c3 - c2 + 1.f;
  float m = 128.f * 2523.f / 4096.f;
  float n = 0.25f * 2610.f / 4096.f;
  n_value = pow((c1 + c2 * pow(n_value, n)) / (1 + c3 * pow(n_value, n)), m);
  return n_value * saturation;
}

int32_t EmulatedFrameSource::ApplyHLGGamma(int32_t value, int32_t saturation) {
  float n_value = (static_cast<float>(value) / saturation);
  // The full HLG gamma curve has additional parameters for n_value > 1, but n_value
  // in the emulated camera is always <= 1 due to lack of HDR display features.
  n_value = 0.5f * pow(n_value, 0.5f);
  return n_value * saturation;
}

int32_t EmulatedFrameSource::GammaTable(int32_t value, int32_t color_space) {
  switch (color_space) {
    case ColorSpaceNamed::BT709:
      return gamma_table_smpte170m_[value];
    case ColorSpaceNamed::BT2020:
      return gamma_table_hlg_[value];  // Assume HLG
    case ColorSpaceNamed::DISPLAY_P3:
    case ColorSpaceNamed::SRGB:
    default:
      return gamma_table_sRGB_[value];
  }

  return 0;
}

void EmulatedFrameSource::CalculateRgbRgbMatrix(
    int32_t color_space, const SensorCharacteristics& chars) {
  const XyzMatrix* xyzMatrix;
  switch (color_space) {
    case ColorSpaceNamed::DISPLAY_P3:
      xyzMatrix = &kDisplayP3Matrix;
      break;
    case ColorSpaceNamed::BT709:
      xyzMatrix = &kBt709Matrix;
      break;
    case ColorSpaceNamed::BT2020:
      xyzMatrix = &kBt2020Matrix;
      break;
    case ColorSpaceNamed::SRGB:
    default:
      xyzMatrix = &kSrgbXyzMatrix;
      break;
  }

  // Calculate the RGB->RGB matrix to convert from the sensor's color space to
  // the destination color space. This is done by converting from sensor RGB to
  // XYZ and then from XYZ to the destination RGB.

  // M = Destination_RGB->XYZ * Sensor_XYZ->RGB
  rgb_rgb_matrix_.rR = xyzMatrix->xR * chars.forward_matrix.rX +
                       xyzMatrix->yR * chars.forward_matrix.rY +
                       xyzMatrix->zR * chars.forward_matrix.rZ;
  rgb_rgb_matrix_.gR = xyzMatrix->xR * chars.forward_matrix.gX +
                       xyzMatrix->yR * chars.forward_matrix.gY +
                       xyzMatrix->zR * chars.forward_matrix.gZ;
  rgb_rgb_matrix_.bR = xyzMatrix->xR * chars.forward_matrix.bX +
                       xyzMatrix->yR * chars.forward_matrix.bY +
                       xyzMatrix->zR * chars.forward_matrix.bZ;
  rgb_rgb_matrix_.rG = xyzMatrix->xG * chars.forward_matrix.rX +
                       xyzMatrix->yG * chars.forward_matrix.rY +
                       xyzMatrix->zG * chars.forward_matrix.rZ;
  rgb_rgb_matrix_.gG = xyzMatrix->xG * chars.forward_matrix.gX +
                       xyzMatrix->yG * chars.forward_matrix.gY +
                       xyzMatrix->zG * chars.forward_matrix.gZ;
  rgb_rgb_matrix_.bG = xyzMatrix->xG * chars.forward_matrix.bX +
                       xyzMatrix->yG * chars.forward_matrix.bY +
                       xyzMatrix->zG * chars.forward_matrix.bZ;
  rgb_rgb_matrix_.rB = xyzMatrix->xB * chars.forward_matrix.rX +
                       xyzMatrix->yB * chars.forward_matrix.rY +
                       xyzMatrix->zB * chars.forward_matrix.rZ;
  rgb_rgb_matrix_.gB = xyzMatrix->xB * chars.forward_matrix.gX +
                       xyzMatrix->yB * chars.forward_matrix.gY +
                       xyzMatrix->zB * chars.forward_matrix.gZ;
  rgb_rgb_matrix_.bB = xyzMatrix->xB * chars.forward_matrix.bX +
                       xyzMatrix->yB * chars.forward_matrix.bY +
                       xyzMatrix->zB * chars.forward_matrix.bZ;
}

void EmulatedFrameSource::RgbToRgb(uint32_t* r_count, uint32_t* g_count,
                                   uint32_t* b_count) {
  uint32_t r = *r_count;
  uint32_t g = *g_count;
  uint32_t b = *b_count;
  *r_count = (uint32_t)std::max(
      r * rgb_rgb_matrix_.rR + g * rgb_rgb_matrix_.gR + b * rgb_rgb_matrix_.bR,
      0.0f);
  *g_count = (uint32_t)std::max(
      r * rgb_rgb_matrix_.rG + g * rgb_rgb_matrix_.gG + b * rgb_rgb_matrix_.bG,
      0.0f);
  *b_count = (uint32_t)std::max(
      r * rgb_rgb_matrix_.rB + g * rgb_rgb_matrix_.gB + b * rgb_rgb_matrix_.bB,
      0.0f);
}

}  // namespace framesource
}  // namespace android

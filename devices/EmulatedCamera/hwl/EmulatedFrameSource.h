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

#ifndef HW_EMULATOR_CAMERA2_EMULATED_FRAME_SOURCE_H
#define HW_EMULATOR_CAMERA2_EMULATED_FRAME_SOURCE_H

#include <android/hardware/graphics/common/1.2/types.h>
#include <system/camera_metadata.h>

#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include "Base.h"
#include "EmulatedScene.h"
#include "IFrameSource.h"
#include "SensorCharacteristics.h"

namespace android {
namespace framesource {

using google_camera_hal::HalCameraMetadata;

class EmulatedFrameSource : public IFrameSource {
 public:
  EmulatedFrameSource(const LogicalCharacteristics& chars, uint32_t camera_id);
  virtual ~EmulatedFrameSource();

  // Main entry point for producing a frame into a sensor buffer
  status_t ProduceFrame(uint32_t camera_id, nsecs_t timestamp,
                        const SensorSettings& settings, SensorBuffer* buffer,
                        const SensorBuffer* input_buffer) override;

  // Helper for JPEG compression (renders directly to memory)
  status_t RenderYUV420(uint32_t camera_id, nsecs_t timestamp,
                        const SensorSettings& settings,
                        const YUV420Frame& output_frame,
                        const YUV420Frame* input_frame) override;

  void CalculateAndAppendNoiseProfile(float gain /*in ISO*/,
                                      float base_gain_factor,
                                      HalCameraMetadata* result /*out*/) override;

  float GetBaseGainFactor(float max_raw_value) const override {
    return max_raw_value / EmulatedFrameSource::kSaturationElectrons;
  }

  bool HasBinningInfo(uint32_t camera_id) const override;
  BinningState GetBinningState(uint32_t camera_id) const override;

  void ResetSensorBinningInfo() override {
    sensor_binning_factor_info_.clear();
  }

 private:
  struct SensorBinningFactorInfo {
    bool has_raw_stream = false;
    bool has_non_raw_stream = false;
    bool quad_bayer_sensor = false;
    bool max_res_request = false;
    bool has_cropped_raw_stream = false;
    bool raw_in_sensor_zoom_applied = false;
  };

  // Internal state
  std::unique_ptr<LogicalCharacteristics> chars_;

  std::unique_ptr<EmulatedScene> scene_;
  std::map<uint32_t, SensorBinningFactorInfo> sensor_binning_factor_info_;

  // Lookup tables
  std::vector<int32_t> gamma_table_sRGB_;
  std::vector<int32_t> gamma_table_smpte170m_;
  std::vector<int32_t> gamma_table_hlg_;
  RgbRgbMatrix rgb_rgb_matrix_;
  unsigned int rand_seed_ = 1;

  // Constants
  static const uint32_t kRegularSceneHandshake;
  static const uint32_t kReducedSceneHandshake;

  static const float kSaturationVoltage;
  static const uint32_t kSaturationElectrons;
  static const float kVoltsPerLuxSecond;
  static const float kElectronsPerLuxSecond;

  static const float kReadNoiseStddevBeforeGain;  // In electrons
  static const float kReadNoiseStddevAfterGain;   // In raw digital units
  static const float kReadNoiseVarBeforeGain;
  static const float kReadNoiseVarAfterGain;

  static const int32_t kFixedBitPrecision;
  static const int32_t kSaturationPoint;

  // Private helpers
  void InitializeGammaTables();
  void ConfigureScene(uint32_t camera_id, nsecs_t timestamp,
                      const SensorSettings& settings);

  static EmulatedScene::ColorChannels GetQuadBayerColor(uint32_t x, uint32_t y);

  static void RemosaicQuadBayerBlock(uint16_t* img_in, uint16_t* img_out,
                                     int xstart, int ystart,
                                     int row_stride_in_bytes);

  static status_t RemosaicRAW16Image(uint16_t* img_in, uint16_t* img_out,
                                     size_t row_stride_in_bytes,
                                     const SensorCharacteristics& chars);

  void CaptureRawBinned(uint8_t* img, size_t row_stride_in_bytes, uint32_t gain,
                        const SensorCharacteristics& chars);

  void CaptureRawFullRes(uint8_t* img, size_t row_stride_in_bytes,
                         uint32_t gain, const SensorCharacteristics& chars);
  void CaptureRawInSensorZoom(uint8_t* img, size_t row_stride_in_bytes,
                              uint32_t gain, const SensorCharacteristics& chars);
  void CaptureRaw(uint8_t* img, size_t row_stride_in_bytes, uint32_t gain,
                  const SensorCharacteristics& chars, bool in_sensor_zoom,
                  bool binned);

  enum RGBLayout { RGB, RGBA, ARGB };
  void CaptureRGB(uint8_t* img, uint32_t width, uint32_t height,
                  uint32_t stride, RGBLayout layout, uint32_t gain,
                  int32_t color_space, const SensorCharacteristics& chars);
  void CaptureYUV420(YCbCrPlanes yuv_layout, uint32_t width, uint32_t height,
                     uint32_t gain, float zoom_ratio, bool rotate,
                     int32_t color_space, const SensorCharacteristics& chars);
  void CaptureDepth(uint8_t* img, uint32_t gain, uint32_t width, uint32_t height,
                    uint32_t stride, const SensorCharacteristics& chars);
  void RgbToRgb(uint32_t* r_count, uint32_t* g_count, uint32_t* b_count);
  void CalculateRgbRgbMatrix(int32_t color_space,
                             const SensorCharacteristics& chars);

  enum ProcessType { REPROCESS, HIGH_QUALITY, REGULAR };
  status_t ProcessYUV420(const YUV420Frame& input, const YUV420Frame& output,
                         uint32_t gain, ProcessType process_type,
                         float zoom_ratio, bool rotate_and_crop,
                         int32_t color_space,
                         const SensorCharacteristics& chars);

  inline int32_t ApplysRGBGamma(int32_t value, int32_t saturation);
  inline int32_t ApplySMPTE170MGamma(int32_t value, int32_t saturation);
  inline int32_t ApplyST2084Gamma(int32_t value, int32_t saturation);
  inline int32_t ApplyHLGGamma(int32_t value, int32_t saturation);
  inline int32_t GammaTable(int32_t value, int32_t color_space);
};

}  // namespace framesource
}  // namespace android

#endif  // HW_EMULATOR_CAMERA2_EMULATED_FRAME_SOURCE_H

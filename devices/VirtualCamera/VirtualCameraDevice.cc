/*
 * Copyright (C) 2023 The Android Open Source Project
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

#include "VirtualCameraDevice.h"

// #define LOG_NDEBUG 0
#define LOG_TAG "VirtualCameraDevice"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "CameraMetadata.h"
#include "VirtualCameraSession.h"
#include "aidl/android/hardware/camera/common/Status.h"
#include "aidl/android/hardware/camera/device/CameraMetadata.h"
#include "android/binder_auto_utils.h"
#include "android/binder_status.h"
#include "log/log.h"
#include "util/Util.h"

namespace android {
namespace services {
namespace virtualcamera {

using ::aidl::android::hardware::camera::common::CameraResourceCost;
using ::aidl::android::hardware::camera::common::Status;
using ::aidl::android::hardware::camera::device::CameraMetadata;
using ::aidl::android::hardware::camera::device::ICameraDeviceCallback;
using ::aidl::android::hardware::camera::device::ICameraDeviceSession;
using ::aidl::android::hardware::camera::device::ICameraInjectionSession;
using ::aidl::android::hardware::camera::device::StreamConfiguration;
using ::aidl::android::hardware::camera::device::StreamRotation;
using ::aidl::android::hardware::camera::device::StreamType;
using ::aidl::android::hardware::graphics::common::PixelFormat;

namespace {
// Prefix of camera name - "device@1.1/virtual/{numerical_id}"
const char* kDevicePathPrefix = "device@1.1/virtual/";

void convertToAidl(const camera_metadata_t* src, CameraMetadata* dest) {
  if (src == nullptr) {
    return;
  }

  size_t size = get_camera_metadata_size(src);
  auto* src_start = (uint8_t*)src;
  uint8_t* src_end = src_start + size;
  dest->metadata.assign(src_start, src_end);
}

// TODO(b/301023410) - Create utility class to interact with CameraMetadata
// instead of the mess below.
// TODO(b/301023410) - Populate camera characteristics according to camera
// configuration.
CameraMetadata initCameraCharacteristics() {
  android::hardware::camera::common::helper::CameraMetadata cameraCharacteristics;

  const uint8_t hardware_level = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL;
  cameraCharacteristics.update(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
                               &hardware_level, 1);

  const uint8_t flash_available = ANDROID_FLASH_INFO_AVAILABLE_FALSE;
  cameraCharacteristics.update(ANDROID_FLASH_INFO_AVAILABLE, &flash_available,
                               1);

  const uint8_t lens_facing = ANDROID_LENS_FACING_EXTERNAL;
  cameraCharacteristics.update(ANDROID_LENS_FACING, &lens_facing, 1);

  const int32_t sensor_orientation = 0;
  cameraCharacteristics.update(ANDROID_SENSOR_ORIENTATION, &sensor_orientation,
                               1);

  const uint8_t faceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
  cameraCharacteristics.update(
      ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES, &faceDetectMode, 1);

  const std::array<uint8_t, 2> afAvailableModes{ANDROID_CONTROL_AF_MODE_AUTO,
                                                ANDROID_CONTROL_AF_MODE_OFF};
  cameraCharacteristics.update(ANDROID_CONTROL_AF_AVAILABLE_MODES,
                               afAvailableModes.data(), afAvailableModes.size());

  const std::array<int32_t, 8> availableStreamConfigurations{
      ANDROID_SCALER_AVAILABLE_FORMATS_IMPLEMENTATION_DEFINED,
      640,
      480,
      ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
      ANDROID_SCALER_AVAILABLE_FORMATS_BLOB,
      640,
      480,
      ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT};
  cameraCharacteristics.update(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
                               availableStreamConfigurations.data(),
                               availableStreamConfigurations.size());

  const std::array<int32_t, 2> fpsRanges{10, 30};
  cameraCharacteristics.update(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
                               fpsRanges.data(), fpsRanges.size());

  const std::array<int64_t, 8> minFrameDurations{
      ANDROID_SCALER_AVAILABLE_FORMATS_IMPLEMENTATION_DEFINED,
      640,
      480,
      1000000000LL / 30,
      ANDROID_SCALER_AVAILABLE_FORMATS_BLOB,
      640,
      480,
      1000000000LL / 30};  // 30 FPS
  cameraCharacteristics.update(ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
                               minFrameDurations.data(),
                               minFrameDurations.size());

  const std::array<int64_t, 8> stallDurations{
      ANDROID_SCALER_AVAILABLE_FORMATS_IMPLEMENTATION_DEFINED,
      640,
      480,
      0,
      ANDROID_SCALER_AVAILABLE_FORMATS_BLOB,
      640,
      480,
      0};
  cameraCharacteristics.update(ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
                               stallDurations.data(), stallDurations.size());

  const std::array<int32_t, 3> maxControlRegions{0, 0, 0};
  cameraCharacteristics.update(ANDROID_CONTROL_MAX_REGIONS,
                               maxControlRegions.data(),
                               maxControlRegions.size());

  const std::array<int32_t, 4> activeArraySize{0, 0, 640, 480};
  cameraCharacteristics.update(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
                               activeArraySize.data(), activeArraySize.size());

  int32_t awbRegions = 0;
  cameraCharacteristics.update(ANDROID_CONTROL_AWB_REGIONS, &awbRegions, 1);

  int32_t aeRegions = 0;
  cameraCharacteristics.update(ANDROID_CONTROL_AE_REGIONS, &aeRegions, 1);

  int32_t afRegions = 0;
  cameraCharacteristics.update(ANDROID_CONTROL_AF_REGIONS, &afRegions, 1);

  const std::array<int32_t, 2> aeCompensationRange{0, 1};
  cameraCharacteristics.update(ANDROID_CONTROL_AE_COMPENSATION_RANGE,
                               aeCompensationRange.data(),
                               aeCompensationRange.size());

  const camera_metadata_rational_t controlAeCompensationStep[] = {{0, 1}};
  cameraCharacteristics.update(ANDROID_CONTROL_AE_COMPENSATION_STEP,
                               controlAeCompensationStep, 1);

  int32_t jpegMaxSize = 640 * 480;
  cameraCharacteristics.update(ANDROID_JPEG_MAX_SIZE, &jpegMaxSize, 1);

  const float scalerAvailableMaxDigitalZoom = 1.f;
  cameraCharacteristics.update(ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
                               &scalerAvailableMaxDigitalZoom, 1);

  const std::array<uint8_t, 2> controlAvailableModes{ANDROID_CONTROL_MODE_OFF,
                                                     ANDROID_CONTROL_MODE_AUTO};
  cameraCharacteristics.update(ANDROID_CONTROL_AVAILABLE_MODES,
                               controlAvailableModes.data(),
                               controlAvailableModes.size());

  const std::array<int32_t, 1> availableRequestKeys{ANDROID_CONTROL_AF_MODE};
  cameraCharacteristics.update(ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
                               availableRequestKeys.data(),
                               availableRequestKeys.size());

  const std::array<int32_t, 1> availableResultKeys{ANDROID_CONTROL_AF_MODE};
  cameraCharacteristics.update(ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
                               availableResultKeys.data(),
                               availableResultKeys.size());

  const std::array<uint8_t, 1> availableRequestCapabilities{
      ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE};
  cameraCharacteristics.update(ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
                               availableRequestCapabilities.data(),
                               availableRequestCapabilities.size());
  const std::array<int32_t, 19> availableCharacteristicKeys{
      ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
      ANDROID_FLASH_INFO_AVAILABLE,
      ANDROID_CONTROL_AF_AVAILABLE_MODES,
      ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
      ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
      ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
      ANDROID_CONTROL_MAX_REGIONS,
      ANDROID_CONTROL_AE_REGIONS,
      ANDROID_CONTROL_AF_REGIONS,
      ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
      ANDROID_CONTROL_AE_COMPENSATION_RANGE,
      ANDROID_CONTROL_AE_COMPENSATION_STEP,
      ANDROID_LENS_FACING,
      ANDROID_SENSOR_ORIENTATION,
      ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
      ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
      ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
      ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
      ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS};

  cameraCharacteristics.update(ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
                               availableCharacteristicKeys.data(),
                               availableCharacteristicKeys.size());

  CameraMetadata aidlCameraCharacteristics;
  const camera_metadata_t* metadata = cameraCharacteristics.getAndLock();
  convertToAidl(metadata, &aidlCameraCharacteristics);
  cameraCharacteristics.unlock(metadata);
  return aidlCameraCharacteristics;
}

}  // namespace

VirtualCameraDevice::VirtualCameraDevice(const uint32_t cameraId)
    : mCameraId(cameraId) {
  mCameraCharacteristics = initCameraCharacteristics();
}

ndk::ScopedAStatus VirtualCameraDevice::getCameraCharacteristics(
    CameraMetadata* _aidl_return) {
  ALOGV("%s", __func__);
  if (_aidl_return == nullptr) {
    return cameraStatus(Status::ILLEGAL_ARGUMENT);
  }

  *_aidl_return = mCameraCharacteristics;
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraDevice::getPhysicalCameraCharacteristics(
    const std::string& in_physicalCameraId, CameraMetadata* _aidl_return) {
  ALOGV("%s physicalCameraId %s", __func__, in_physicalCameraId.c_str());
  (void)_aidl_return;

  // VTS tests expect this call to fail with illegal argument status for
  // all publicly advertised camera ids.
  // Because we don't support physical camera ids, we just always
  // fail with illegal argument (there's no valid argument to provide).
  return cameraStatus(Status::ILLEGAL_ARGUMENT);
}

ndk::ScopedAStatus VirtualCameraDevice::getResourceCost(
    CameraResourceCost* _aidl_return) {
  ALOGV("%s", __func__);
  if (_aidl_return == nullptr) {
    return cameraStatus(Status::ILLEGAL_ARGUMENT);
  }
  _aidl_return->resourceCost = 100;  // ¯\_(ツ)_/¯
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraDevice::isStreamCombinationSupported(
    const StreamConfiguration& in_streams, bool* _aidl_return) {
  ALOGV("%s", __func__);

  if (_aidl_return == nullptr) {
    return cameraStatus(Status::ILLEGAL_ARGUMENT);
  }

  for (const auto& stream : in_streams.streams) {
    ALOGV("Configuration queried: %s", stream.toString().c_str());

    if (stream.streamType == StreamType::INPUT) {
      ALOGW("Input stream type is not supported");
      *_aidl_return = false;
      return ndk::ScopedAStatus::ok();
    }

    // TODO(b/301023410) remove hardcoded format checks, verify against configuration.
    if (stream.width != 640 || stream.height != 480 ||
        stream.rotation != StreamRotation::ROTATION_0 ||
        (stream.format != PixelFormat::IMPLEMENTATION_DEFINED &&
         stream.format != PixelFormat::YCBCR_420_888 &&
         stream.format != PixelFormat::BLOB)) {
      *_aidl_return = false;
      return ndk::ScopedAStatus::ok();
    }
  }

  *_aidl_return = true;
  return ndk::ScopedAStatus::ok();
};

ndk::ScopedAStatus VirtualCameraDevice::open(
    const std::shared_ptr<ICameraDeviceCallback>& in_callback,
    std::shared_ptr<ICameraDeviceSession>* _aidl_return) {
  ALOGV("%s", __func__);

  *_aidl_return = ndk::SharedRefBase::make<VirtualCameraSession>(
      in_callback, std::to_string(mCameraId));

  return ndk::ScopedAStatus::ok();
};

ndk::ScopedAStatus VirtualCameraDevice::openInjectionSession(
    const std::shared_ptr<ICameraDeviceCallback>& in_callback,
    std::shared_ptr<ICameraInjectionSession>* _aidl_return) {
  ALOGV("%s", __func__);

  (void)in_callback;
  (void)_aidl_return;
  return cameraStatus(Status::OPERATION_NOT_SUPPORTED);
}

ndk::ScopedAStatus VirtualCameraDevice::setTorchMode(bool in_on) {
  ALOGV("%s -> on = %s", __func__, in_on ? "on" : "off");
  return cameraStatus(Status::OPERATION_NOT_SUPPORTED);
}

ndk::ScopedAStatus VirtualCameraDevice::turnOnTorchWithStrengthLevel(
    int32_t in_torchStrength) {
  ALOGV("%s -> torchStrength = %d", __func__, in_torchStrength);
  return cameraStatus(Status::OPERATION_NOT_SUPPORTED);
}

ndk::ScopedAStatus VirtualCameraDevice::getTorchStrengthLevel(
    int32_t* _aidl_return) {
  (void)_aidl_return;
  return cameraStatus(Status::OPERATION_NOT_SUPPORTED);
}

binder_status_t VirtualCameraDevice::dump(int fd, const char** args,
                                          uint32_t numArgs) {
  // TODO(b/301023410) Implement.
  (void)fd;
  (void)args;
  (void)numArgs;
  return STATUS_OK;
}

std::string VirtualCameraDevice::getCameraName() const {
  return std::string(kDevicePathPrefix) + std::to_string(mCameraId);
}

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

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

#ifndef ANDROID_SERVICES_VIRTUALCAMERA_VIRTUALCAMERASESSION_H
#define ANDROID_SERVICES_VIRTUALCAMERA_VIRTUALCAMERASESSION_H

#include <deque>
#include <list>
#include <map>
#include <memory>

#include "VirtualCameraStream.h"
#include "aidl/android/hardware/camera/common/Status.h"
#include "aidl/android/hardware/camera/device/BnCameraDeviceSession.h"
#include "aidl/android/hardware/camera/device/BufferRequest.h"
#include "aidl/android/hardware/camera/device/ICameraDeviceCallback.h"
#include "aidl/android/hardware/camera/device/Stream.h"
#include "android-base/unique_fd.h"
#include "android/hardware_buffer.h"
#include "utils/Mutex.h"

namespace android {

template <typename T, typename U>
struct AidlMessageQueue;
namespace services {
namespace virtualcamera {

// Implementation of ICameraDeviceSession AIDL interface to allow camera
// framework to read image data from open virtual camera device. This class
// encapsulates possibly several image streams for the same session.
class VirtualCameraSession
    : public ::aidl::android::hardware::camera::device::BnCameraDeviceSession {
 public:
  VirtualCameraSession(
      std::shared_ptr<
          ::aidl::android::hardware::camera::device::ICameraDeviceCallback>
          cameraDeviceCallback,
      const std::string& cameraId);

  virtual ~VirtualCameraSession() override = default;

  ndk::ScopedAStatus close() override;

  ndk::ScopedAStatus configureStreams(
      const ::aidl::android::hardware::camera::device::StreamConfiguration&
          in_requestedConfiguration,
      std::vector<::aidl::android::hardware::camera::device::HalStream>*
          _aidl_return) override;

  ndk::ScopedAStatus constructDefaultRequestSettings(
      ::aidl::android::hardware::camera::device::RequestTemplate in_type,
      ::aidl::android::hardware::camera::device::CameraMetadata* _aidl_return)
      override;

  ndk::ScopedAStatus flush() override;

  ndk::ScopedAStatus getCaptureRequestMetadataQueue(
      ::aidl::android::hardware::common::fmq::MQDescriptor<
          int8_t, ::aidl::android::hardware::common::fmq::SynchronizedReadWrite>*
          _aidl_return) override;

  ndk::ScopedAStatus getCaptureResultMetadataQueue(
      ::aidl::android::hardware::common::fmq::MQDescriptor<
          int8_t, ::aidl::android::hardware::common::fmq::SynchronizedReadWrite>*
          _aidl_return) override;

  ndk::ScopedAStatus isReconfigurationRequired(
      const ::aidl::android::hardware::camera::device::CameraMetadata&
          in_oldSessionParams,
      const ::aidl::android::hardware::camera::device::CameraMetadata&
          in_newSessionParams,
      bool* _aidl_return) override;

  ndk::ScopedAStatus processCaptureRequest(
      const std::vector<::aidl::android::hardware::camera::device::CaptureRequest>&
          in_requests,
      const std::vector<::aidl::android::hardware::camera::device::BufferCache>&
          in_cachesToRemove,
      int32_t* _aidl_return) override;

  ndk::ScopedAStatus signalStreamFlush(const std::vector<int32_t>& in_streamIds,
                                       int32_t in_streamConfigCounter) override;

  ndk::ScopedAStatus switchToOffline(
      const std::vector<int32_t>& in_streamsToKeep,
      ::aidl::android::hardware::camera::device::CameraOfflineSessionInfo*
          out_offlineSessionInfo,
      std::shared_ptr<
          ::aidl::android::hardware::camera::device::ICameraOfflineSession>*
          _aidl_return) override;

  ndk::ScopedAStatus repeatingRequestEnd(
      int32_t in_frameNumber, const std::vector<int32_t>& in_streamIds) override;

 private:
  void removeBufferCaches(
      const std::vector<::aidl::android::hardware::camera::device::BufferCache>&
          cachesToRemove);

  std::shared_ptr<AHardwareBuffer> fetchHardwareBuffer(
      const ::aidl::android::hardware::camera::device::StreamBuffer& streamBuffer);

  ndk::ScopedAStatus processCaptureRequest(
      const ::aidl::android::hardware::camera::device::CaptureRequest& request);

  const std::string mCameraId;

  std::mutex mLock;

  std::shared_ptr<::aidl::android::hardware::camera::device::ICameraDeviceCallback>
      mCameraDeviceCallback GUARDED_BY(mLock);

  // Mapping stream id -> VirtualCameraStream.
  std::map<int, VirtualCameraStream> mStreams GUARDED_BY(mLock);

  using RequestMetadataQueue = AidlMessageQueue<
      int8_t, ::aidl::android::hardware::common::fmq::SynchronizedReadWrite>;
  std::unique_ptr<RequestMetadataQueue> mRequestMetadataQueue;

  using ResultMetadataQueue = AidlMessageQueue<
      int8_t, ::aidl::android::hardware::common::fmq::SynchronizedReadWrite>;
  std::shared_ptr<ResultMetadataQueue> mResultMetadataQueue;

  std::atomic_bool mFirstRequest{true};
};

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

#endif  // ANDROID_SERVICES_VIRTUAL_CAMERA_VIRTUALCAMERASESSION_H

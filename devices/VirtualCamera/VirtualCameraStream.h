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
#ifndef ANDROID_SERVICES_VIRTUALCAMERA_VIRTUALCAMERASTREAM_H
#define ANDROID_SERVICES_VIRTUALCAMERA_VIRTUALCAMERASTREAM_H

#include <memory>
#include <mutex>
#include <unordered_map>

#include "aidl/android/hardware/camera/device/Stream.h"
#include "aidl/android/hardware/camera/device/StreamBuffer.h"
#include "android/hardware_buffer.h"
#include "utils/Mutex.h"

namespace android {
namespace services {
namespace virtualcamera {

// Encapsulates buffer management for the set of buffers belonging to the single
// camera stream.
class VirtualCameraStream {
 public:
  VirtualCameraStream(
      int streamId,
      const ::aidl::android::hardware::camera::device::Stream& stream);

  // Get AHardwareBuffer instance corresponding to StreamBuffer from camera AIDL.
  // In case this is the first occurrence of the buffer, this will perform mapping
  // and stores hardware buffer in cache for further use.
  //
  // Returns nullptr in case buffer cannot be mapped or retrieved from the cache.
  std::shared_ptr<AHardwareBuffer> getHardwareBuffer(
      const ::aidl::android::hardware::camera::device::StreamBuffer& buffer)
      EXCLUDES(mLock);

  // Un-maps the previously mapped buffer and removes it from the stream cache.
  // Returns true if removal is successful, false otherwise.
  bool removeBuffer(int bufferId) EXCLUDES(mLock);

 private:
  const int mId;
  const ::aidl::android::hardware::camera::device::Stream mStreamConfig;
  std::mutex mLock;

  // Cache for already mapped buffers, mapping bufferId -> AHardwareBuffer instance.
  std::unordered_map<int, std::shared_ptr<AHardwareBuffer>> mBuffers
      GUARDED_BY(mLock);
};

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

#endif  // ANDROID_SERVICES_VIRTUALCAMERA_VIRTUALCAMERASTREAM_H

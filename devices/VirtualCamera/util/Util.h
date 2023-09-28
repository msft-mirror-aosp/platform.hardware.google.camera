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

#ifndef ANDROID_SERVICES_VIRTUALCAMERA_UTIL_H
#define ANDROID_SERVICES_VIRTUALCAMERA_UTIL_H

#include <cstdint>

#include "aidl/android/hardware/camera/common/Status.h"
#include "aidl/android/hardware/camera/device/StreamBuffer.h"
#include "android/binder_auto_utils.h"

namespace android {
namespace services {
namespace virtualcamera {

// Converts camera AIDL status to ndk::ScopedAStatus
inline ndk::ScopedAStatus cameraStatus(
    const ::aidl::android::hardware::camera::common::Status status) {
  return ndk::ScopedAStatus::fromServiceSpecificError(
      static_cast<int32_t>(status));
}

// RAII wrapper to extract sync fence file descriptor
// from AIDL NativeHandle and ensure it's properly closed.
class FenceGuard {
 public:
  explicit FenceGuard(
      const ::aidl::android::hardware::common::NativeHandle& handle);

  // Make this move-only.
  FenceGuard(FenceGuard&&) = default;
  FenceGuard& operator=(FenceGuard&&) = default;
  FenceGuard(const FenceGuard&) = delete;
  FenceGuard& operator=(const FenceGuard&) = delete;

  // Returns underlying file descriptor, or -1 if there's none.
  int get() const;

  ~FenceGuard();

 private:
  int mFd = -1;
};

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

#endif  // ANDROID_SERVICES_VIRTUALCAMERA_UTIL_H

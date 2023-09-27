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
#include "VirtualCameraStream.h"

// #define LOG_NDEBUG 0
#define LOG_TAG "VirtualCameraStream"

#include <cstdint>
#include <memory>
#include <mutex>
#include <tuple>
#include <utility>

#include "aidl/android/hardware/camera/device/Stream.h"
#include "aidl/android/hardware/camera/device/StreamBuffer.h"
#include "aidlcommonsupport/NativeHandle.h"
#include "android/hardware_buffer.h"
#include "cutils/native_handle.h"
#include "log/log.h"
#include "vndk/hardware_buffer.h"

namespace android {
namespace services {
namespace virtualcamera {

using ::aidl::android::hardware::camera::device::Stream;
using ::aidl::android::hardware::camera::device::StreamBuffer;
using ::aidl::android::hardware::common::NativeHandle;

namespace {

std::shared_ptr<AHardwareBuffer> importBuffer(const NativeHandle& aidlHandle,
                                              const Stream& streamConfig) {
  std::unique_ptr<native_handle_t, int (*)(native_handle_t*)> nativeHandle(
      ::android::makeFromAidl(aidlHandle), native_handle_delete);

  const AHardwareBuffer_Desc bufferDesc = {
      .width = static_cast<uint32_t>(streamConfig.width),
      .height = static_cast<uint32_t>(streamConfig.height),
      .layers = static_cast<uint32_t>(1),
      .format = static_cast<uint32_t>(streamConfig.format),
      .usage = static_cast<uint32_t>(streamConfig.usage),
      // TODO(b/301023410) Verify how stride needs to
      // be computed for various buffer formats.
      .stride = static_cast<uint32_t>(streamConfig.width),
      .rfu0 = 0,
      .rfu1 = 0};

  AHardwareBuffer* hwBufferPtr = nullptr;

  int ret = AHardwareBuffer_createFromHandle(
      &bufferDesc, nativeHandle.get(),
      AHARDWAREBUFFER_CREATE_FROM_HANDLE_METHOD_CLONE, &hwBufferPtr);
  if (ret != NO_ERROR) {
    ALOGE("Failed to import buffer from native handle, err = %d, Stream = %s",
          ret, streamConfig.toString().c_str());
    return nullptr;
  }

  return std::shared_ptr<AHardwareBuffer>(hwBufferPtr, AHardwareBuffer_release);
}

}  // namespace

VirtualCameraStream::VirtualCameraStream(int id, const Stream& stream)
    : mId(id), mStreamConfig(stream) {
}

std::shared_ptr<AHardwareBuffer> VirtualCameraStream::getHardwareBuffer(
    const StreamBuffer& buffer) {
  if (buffer.streamId != mStreamConfig.id) {
    ALOGE("Caller requesting buffer belonging to stream %d from stream %d",
          buffer.streamId, mStreamConfig.id);
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(mLock);

  auto it = mBuffers.find(buffer.bufferId);
  if (it != mBuffers.end()) {
    return it->second;
  }

  ALOGV("Importing buffer %ld for stream %d", buffer.bufferId, buffer.streamId);

  auto hwBufferPtr = importBuffer(buffer.buffer, mStreamConfig);
  if (hwBufferPtr != nullptr) {
    mBuffers.emplace(std::piecewise_construct,
                     std::forward_as_tuple(buffer.bufferId),
                     std::forward_as_tuple(hwBufferPtr));
  }

  return hwBufferPtr;
}

bool VirtualCameraStream::removeBuffer(int bufferId) {
  std::lock_guard<std::mutex> lock(mLock);

  return mBuffers.erase(bufferId) == 1;
}

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

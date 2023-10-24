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

// #define LOG_NDEBUG 0
#define LOG_TAG "VirtualCameraStream"
#include "VirtualCameraStream.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <tuple>
#include <utility>

#include "EGL/egl.h"
#include "aidl/android/hardware/camera/device/Stream.h"
#include "aidl/android/hardware/camera/device/StreamBuffer.h"
#include "aidlcommonsupport/NativeHandle.h"
#include "android/hardware_buffer.h"
#include "cutils/native_handle.h"
#include "log/log.h"
#include "vndk/hardware_buffer.h"

namespace android {
namespace companion {
namespace virtualcamera {

using ::aidl::android::hardware::camera::device::Stream;
using ::aidl::android::hardware::camera::device::StreamBuffer;
using ::aidl::android::hardware::common::NativeHandle;
using ::aidl::android::hardware::graphics::common::PixelFormat;

namespace {

AHardwareBuffer_Desc createBufferDescriptorForStream(const Stream& stream) {
  if (stream.format == PixelFormat::BLOB) {
    return {// BLOB needs to have width = bufferSize & height = 1.
            .width = static_cast<uint32_t>(stream.bufferSize),
            .height = 1,
            .layers = 1,
            .format = static_cast<uint32_t>(stream.format),
            .usage = static_cast<uint32_t>(stream.usage),
            .stride = 0,
            .rfu0 = 0,
            .rfu1 = 0};
  } else {
    return {.width = static_cast<uint32_t>(stream.width),
            .height = static_cast<uint32_t>(stream.height),
            .layers = 1,
            .format = static_cast<uint32_t>(stream.format),
            .usage = static_cast<uint32_t>(stream.usage),
            // TODO(b/301023410) Verify how stride needs to
            // be computed for various buffer formats.
            .stride = static_cast<uint32_t>(stream.width),
            .rfu0 = 0,
            .rfu1 = 0};
  }
}

std::shared_ptr<AHardwareBuffer> importBuffer(const NativeHandle& aidlHandle,
                                              const Stream& streamConfig) {
  if (aidlHandle.fds.empty()) {
    ALOGE("Empty handle - nothing to import");
    return nullptr;
  }
  std::unique_ptr<native_handle_t, int (*)(native_handle_t*)> nativeHandle(
      ::android::makeFromAidl(aidlHandle), native_handle_delete);

  const AHardwareBuffer_Desc bufferDesc =
      createBufferDescriptorForStream(streamConfig);
  AHardwareBuffer* hwBufferPtr = nullptr;

  int ret = AHardwareBuffer_createFromHandle(
      &bufferDesc, nativeHandle.get(),
      AHARDWAREBUFFER_CREATE_FROM_HANDLE_METHOD_CLONE, &hwBufferPtr);
  if (ret != NO_ERROR) {
    ALOGE(
        "%s: Failed to import buffer from native handle, err = %d, Stream = %s",
        __func__, ret, streamConfig.toString().c_str());
    return nullptr;
  }

  return std::shared_ptr<AHardwareBuffer>(hwBufferPtr, AHardwareBuffer_release);
}

}  // namespace

VirtualCameraStream::VirtualCameraStream(const Stream& stream)
    : mStreamConfig(stream) {
}

std::shared_ptr<AHardwareBuffer> VirtualCameraStream::getHardwareBuffer(
    const StreamBuffer& buffer) {
  if (buffer.streamId != mStreamConfig.id) {
    ALOGE("%s: Caller requesting buffer belonging to stream %d from stream %d",
          __func__, buffer.streamId, mStreamConfig.id);
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(mLock);
  return getHardwareBufferLocked(buffer);
}

std::shared_ptr<EglFrameBuffer> VirtualCameraStream::getEglFrameBuffer(
    const EGLDisplay eglDisplay,
    const ::aidl::android::hardware::camera::device::StreamBuffer& buffer) {
  if (buffer.streamId != mStreamConfig.id) {
    ALOGE("%s: Caller requesting buffer belonging to stream %d from stream %d",
          __func__, buffer.streamId, mStreamConfig.id);
    return nullptr;
  }

  const FramebufferMapKey key(buffer.bufferId, eglDisplay);

  std::lock_guard<std::mutex> lock(mLock);

  auto it = mEglFramebuffers.find(key);
  if (it != mEglFramebuffers.end()) {
    return it->second;
  }

  std::shared_ptr<AHardwareBuffer> hwBufferPtr = getHardwareBufferLocked(buffer);
  if (hwBufferPtr == nullptr) {
    return nullptr;
  }
  std::shared_ptr<EglFrameBuffer> framebufferPtr =
      std::make_shared<EglFrameBuffer>(eglDisplay, hwBufferPtr);
  mEglFramebuffers.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                           std::forward_as_tuple(framebufferPtr));

  return framebufferPtr;
}

std::shared_ptr<AHardwareBuffer> VirtualCameraStream::getHardwareBufferLocked(
    const StreamBuffer& buffer) {
  auto it = mBuffers.find(buffer.bufferId);
  if (it != mBuffers.end()) {
    return it->second;
  }

  ALOGV("%s: Importing buffer %" PRId64 " for stream %d", __func__,
        buffer.bufferId, buffer.streamId);

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

Stream VirtualCameraStream::getStreamConfig() const {
  return mStreamConfig;
}

}  // namespace virtualcamera
}  // namespace companion
}  // namespace android

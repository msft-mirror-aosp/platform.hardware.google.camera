/*
 * Copyright (C) 2018 The Android Open Source Project
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

#define LOG_TAG "goog_gralloc_wrapper"

#include "goog_gralloc_wrapper.h"

#include <utils/Log.h>

namespace android {
namespace camera_sensor_listener {

GoogGrallocWrapper::GoogGrallocWrapper()
    : allocator_(GraphicBufferAllocator::get()),
      mapper_(GraphicBufferMapper::get()) {
}

GoogGrallocWrapper::~GoogGrallocWrapper() {
  for (auto buffer_handle : cloned_buffers_) {
    auto buffer = const_cast<native_handle_t*>(buffer_handle);
    native_handle_close(buffer);
    native_handle_delete(buffer);
  }
  cloned_buffers_.clear();

  for (auto buffer_handle : imported_buffers_) {
    auto buffer = const_cast<native_handle_t*>(buffer_handle);
    if (status_t status = mapper_.freeBuffer(buffer); status != OK) {
      ALOGE("%s: Failed to free buffer %p: %s", __FUNCTION__, buffer,
            statusToString(status).c_str());
    }
  }
  imported_buffers_.clear();
}

GraphicBufferAllocator& GoogGrallocWrapper::GetAllocator() const {
  return allocator_;
}

std::string GoogGrallocWrapper::DumpDebugInfo() const {
  std::string debug_info;
  allocator_.dump(debug_info);
  return debug_info;
}

// When import is false, this simply calls IAllocator::allocate. When import
// is true, the returned buffers are also imported into the mapper.
// Either case, the returned buffers must be freed with freeBuffer.
std::vector<const native_handle_t*> GoogGrallocWrapper::Allocate(
    uint32_t width, uint32_t height, PixelFormat format, uint32_t layer_count,
    uint64_t usage, uint32_t buffer_count, bool import, uint32_t* out_stride) {
  std::vector<const native_handle_t*> buffer_handles;
  buffer_handles.reserve(buffer_count);
  for (int i = 0; i < buffer_count; ++i) {
    buffer_handle_t handle = nullptr;
    if (import) {
      allocator_.allocate(width, height, format, layer_count, usage, &handle,
                          out_stride, "SensorListener");
      if (handle != nullptr) {
        imported_buffers_.insert(handle);
      }
    } else {
      allocator_.allocateRawHandle(width, height, format, layer_count, usage,
                                   &handle, out_stride, "SensorListener");
      if (handle != nullptr) {
        cloned_buffers_.insert(handle);
      }
    }
    buffer_handles.push_back(handle);
  }

  return buffer_handles;
}

const native_handle_t* GoogGrallocWrapper::AllocateOneBuffer(
    uint32_t width, uint32_t height, PixelFormat format, uint32_t layer_count,
    uint64_t usage, bool import, uint32_t* out_stride) {
  auto buffers = Allocate(width, height, format, layer_count, usage,
                          /*buffer_count=*/1, import, out_stride);
  return buffers[0];
}

GraphicBufferMapper& GoogGrallocWrapper::GetMapper() const {
  return mapper_;
}

void GoogGrallocWrapper::FreeBuffer(const native_handle_t* buffer_handle) {
  auto buffer = const_cast<native_handle_t*>(buffer_handle);

  if (imported_buffers_.erase(buffer_handle)) {
    if (status_t status = mapper_.freeBuffer(buffer); status != OK) {
      ALOGE("%s: Failed to free %p: %s", __FUNCTION__, buffer,
            statusToString(status).c_str());
    }
  } else if (cloned_buffers_.erase(buffer_handle)) {
    native_handle_close(buffer);
    native_handle_delete(buffer);
  } else {
    ALOGE("%s: buffer %p is not in cloned_buffers_ or imported_buffers_",
          __FUNCTION__, buffer);
  }
}

// We use fd to pass fences in and out of the mapper.  The ownership of the fd
// is always transferred with each of these functions.
void* GoogGrallocWrapper::Lock(const native_handle_t* buffer_handle,
                               uint64_t cpu_usage, const Rect& access_region,
                               int acquire_fence) {
  auto buffer = const_cast<native_handle_t*>(buffer_handle);
  if (!imported_buffers_.contains(buffer)) {
    ALOGE("%s: buffer %p is not imported", __FUNCTION__, buffer);
    return nullptr;
  }

  void* locked_buffer = nullptr;
  status_t status = mapper_.lockAsync(buffer, cpu_usage, access_region,
                                      &locked_buffer, acquire_fence);
  if (status != OK) {
    ALOGE("%s: Failed to lock buffer %p: %s", __FUNCTION__, buffer,
          statusToString(status).c_str());
  }

  if (acquire_fence >= 0) {
    close(acquire_fence);
  }

  return locked_buffer;
}

int GoogGrallocWrapper::Unlock(const native_handle_t* buffer_handle) {
  auto buffer = const_cast<native_handle_t*>(buffer_handle);
  if (!imported_buffers_.contains(buffer)) {
    ALOGE("%s: buffer %p is not imported", __FUNCTION__, buffer);
    return -1;
  }

  int release_fence = -1;
  status_t status = mapper_.unlockAsync(buffer, &release_fence);
  if (status != OK) {
    ALOGE("%s: Failed to unlock buffer %p: %s", __FUNCTION__, buffer,
          statusToString(status).c_str());
  }

  return release_fence;
}

}  // namespace camera_sensor_listener
}  // namespace android

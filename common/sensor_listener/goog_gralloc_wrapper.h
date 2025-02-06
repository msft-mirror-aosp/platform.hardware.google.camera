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

#ifndef VENDOR_GOOGLE_CAMERA_SENSOR_LISTENER_GOOG_GRALLOC_WRAPPER_H_
#define VENDOR_GOOGLE_CAMERA_SENSOR_LISTENER_GOOG_GRALLOC_WRAPPER_H_

#include <android/frameworks/sensorservice/1.0/ISensorManager.h>
#include <sys/mman.h>
#include <ui/GraphicBufferAllocator.h>
#include <ui/GraphicBufferMapper.h>
#include <utils/StrongPointer.h>

#include <unordered_set>

namespace android {
namespace camera_sensor_listener {

// GoogGrallocWrapper is a wrapper class to
// ::android::GraphicBufferAllocator and
// ::android::GraphicBufferMapper.
// It can used by direct channel based sensor listener class.
class GoogGrallocWrapper {
 public:
  // Constructor.
  GoogGrallocWrapper();

  // Destructor.
  ~GoogGrallocWrapper();

  // Get the reference of GraphicBufferAllocator.
  GraphicBufferAllocator& GetAllocator() const;

  // Wrapper of IAllocator dumpDebugInfo method.
  std::string DumpDebugInfo() const;

  // Wrapper of GraphicBufferAllocator::allocate and
  // GraphicBufferAllocator::allocateRawHandle.
  std::vector<const native_handle_t*> Allocate(
      uint32_t width, uint32_t height, PixelFormat format, uint32_t layer_count,
      uint64_t usage, uint32_t buffer_count, bool import = true,
      uint32_t* out_stride = nullptr);

  // Special case of Allocate, where allocated buffer count is 1.
  const native_handle_t* AllocateOneBuffer(uint32_t width, uint32_t height,
                                           PixelFormat format,
                                           uint32_t layer_count, uint64_t usage,
                                           bool import = true,
                                           uint32_t* out_stride = nullptr);

  // Get the reference of GraphicBufferMapper.
  GraphicBufferMapper& GetMapper() const;

  // Wrapper of GraphicBufferMapper::freeBuffer.
  void FreeBuffer(const native_handle_t* buffer_handle);

  // Wrapper of GraphicBufferMapper::lockAsync.
  // We use fd to pass fences in and out of the mapper.  The ownership of the fd
  // is always transferred with each of these functions.
  void* Lock(const native_handle_t* buffer_handle, uint64_t cpu_usage,
             const Rect& access_region, int acquire_fence);

  // Wrapper of GraphicBufferMapper::unlockAsync.
  int Unlock(const native_handle_t* buffer_handle);

 private:
  GraphicBufferAllocator& allocator_;
  GraphicBufferMapper& mapper_;

  // Set of cloned buffer handles.
  // Keep track of all cloned and imported handles.  When a test fails with
  // ASSERT_*, the destructor will free the handles for the test.
  std::unordered_set<const native_handle_t*> cloned_buffers_;

  // Set of imported buffer handles.
  std::unordered_set<const native_handle_t*> imported_buffers_;
};

}  // namespace camera_sensor_listener
}  // namespace android

#endif  // VENDOR_GOOGLE_CAMERA_SENSOR_LISTENER_GOOG_GRALLOC_WRAPPER_H_

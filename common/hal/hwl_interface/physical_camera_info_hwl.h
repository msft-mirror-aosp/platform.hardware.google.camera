/*
 * Copyright (C) 2024 The Android Open Source Project
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

#ifndef HARDWARE_GOOGLE_CAMERA_HAL_HWL_INTERFACE_PHYSICAL_CAMERA_INFO_HWL_H_
#define HARDWARE_GOOGLE_CAMERA_HAL_HWL_INTERFACE_PHYSICAL_CAMERA_INFO_HWL_H_

#include <memory>
#include <vector>

#include "hal_camera_metadata.h"

namespace android {
namespace google_camera_hal {

class PhysicalCameraInfoHwl {
 public:
  // Return the physical camera ID that this camera device is associated
  // with. If the camera device does not have multiple physical camera devices,
  // this method should return an empty std::vector.
  virtual std::vector<uint32_t> GetPhysicalCameraIds() const {
    return std::vector<uint32_t>();
  };

  // Get the characteristics of the physical camera of this camera device.
  // characteristics will be filled by CameraDeviceHwl.
  virtual status_t GetPhysicalCameraCharacteristics(
      uint32_t physical_camera_id,
      std::unique_ptr<HalCameraMetadata>* characteristics) const = 0;
  virtual ~PhysicalCameraInfoHwl() = default;
};

}  // namespace google_camera_hal
}  // namespace android

#endif  // HARDWARE_GOOGLE_CAMERA_HAL_HWL_INTERFACE_PHYSICAL_CAMERA_INFO_HWL_H_

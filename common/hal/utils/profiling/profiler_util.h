/*
 * Copyright (C) 2019 The Android Open Source Project
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

#ifndef HARDWARE_GOOGLE_CAMERA_HAL_UTILS_PROFILER_UTIL_H
#define HARDWARE_GOOGLE_CAMERA_HAL_UTILS_PROFILER_UTIL_H

#include <string>

namespace android {
namespace google_camera_hal {

constexpr char kFirstFrame[] = "First frame";
constexpr char kHalTotal[] = "HAL Total";
constexpr char kIdleString[] = "<-- IDLE -->";
constexpr char kOverall[] = "Overall";

enum class EventType {
  kNone,
  kOpen,
  kConfigureStream,
  kFlush,
  kClose,
  kFirstFrameStart,
  kFirstFrameEnd,
};

inline std::string EventTypeToString(EventType type) {
  switch (type) {
    case (EventType::kNone):
      return std::string("None");
    case (EventType::kOpen):
      return std::string("Open");
    case (EventType::kConfigureStream):
      return std::string("ConfigureStream");
    case (EventType::kFlush):
      return std::string("Flush");
    case (EventType::kClose):
      return std::string("Close");
    case (EventType::kFirstFrameStart):
      return std::string("FirstFrameStart");
    case (EventType::kFirstFrameEnd):
      return std::string("FirstFrameEnd");
  }
}

}  // namespace google_camera_hal
}  // namespace android

#endif  // HARDWARE_GOOGLE_CAMERA_HAL_UTILS_PROFILER_UTIL_H

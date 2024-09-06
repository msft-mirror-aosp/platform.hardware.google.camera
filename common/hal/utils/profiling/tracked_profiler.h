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
#ifndef HARDWARE_GOOGLE_CAMERA_HAL_UTILS_TRACKED_PROFILER_H
#define HARDWARE_GOOGLE_CAMERA_HAL_UTILS_TRACKED_PROFILER_H

#include <memory>
#include <mutex>
#include <string>

#include "aidl_profiler.h"
#include "profiler.h"
#include "profiler_util.h"

namespace android {
namespace google_camera_hal {

using google::camera_common::Profiler;

class TrackedProfiler {
  /* Tracks the progress of a profiling operation (open, close,
   * reconfiguration) by accepting or rejecting new AIDL events (open, flush,
   * configure streams, first frame start, first frame end, or close)
   */
 public:
  TrackedProfiler(std::shared_ptr<Profiler> profiler,
                  std::string camera_id_string, EventType initial_state)
      : state_(initial_state),
        profiler_(profiler),
        camera_id_string_(camera_id_string){};

  void SetUseCase(std::string name);
  bool ShouldDelete(EventType incoming);
  void UpdateStateLocked(EventType incoming);
  std::unique_ptr<android::google_camera_hal::AidlScopedProfiler>
  AcceptNextState(EventType incoming);
  bool AcceptFirstFrameStart();
  bool AcceptFirstFrameEnd();
  void DeleteProfilerLocked();
  void DeleteProfiler();
  void IdleStartLocked();
  void IdleStart();
  void IdleEndLocked();

  EventType GetState() {
    return state_;
  }

  std::mutex tracked_api_mutex_;
  EventType state_ = EventType::kNone;
  std::shared_ptr<Profiler> profiler_ = nullptr;
  const std::string camera_id_string_;
  uint8_t config_count_ = 0;
  uint8_t flush_count_ = 0;
  uint8_t idle_start_count_ = 0;
  uint8_t idle_end_count_ = 0;
};

}  // namespace google_camera_hal
}  // namespace android

#endif  // HARDWARE_GOOGLE_CAMERA_HAL_UTILS_TRACKED_PROFILER_H

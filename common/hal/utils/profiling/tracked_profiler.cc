/*
 * Copyright (C) 2022 The Android Open Source Project
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
#define LOG_TAG "GCH_TrackedProfiler"
#include "tracked_profiler.h"

#include <log/log.h>

#include <cstdint>
#include <memory>
#include <mutex>

#include "aidl_profiler.h"
#include "profiler.h"
#include "profiler_util.h"

namespace android {
namespace google_camera_hal {

using google::camera_common::Profiler;

void TrackedProfiler::SetUseCase(std::string name) {
  profiler_->SetUseCase(name);
}

bool TrackedProfiler::ShouldDelete(EventType incoming) {
  if (incoming == EventType::kOpen &&
      (state_ == EventType::kConfigureStream || state_ == EventType::kOpen)) {
    return true;
  }
  if (state_ == EventType::kFirstFrameEnd || state_ == EventType::kClose) {
    return true;
  }
  return false;
}

void TrackedProfiler::UpdateStateLocked(EventType incoming) {
  state_ = incoming;
}

std::unique_ptr<AidlScopedProfiler> TrackedProfiler::AcceptNextState(
    EventType incoming) {
  std::lock_guard<std::mutex> lock(tracked_api_mutex_);
  int32_t id = 0;
  switch (state_) {
    case (EventType::kNone):
      if (incoming == EventType::kOpen) {
        SetUseCase(camera_id_string_ + "-Open");
        break;
      } else if (incoming == EventType::kFlush) {
        break;
      }
      return nullptr;
    case (EventType::kOpen):
      if (incoming == EventType::kConfigureStream) {
        break;
      }
      return nullptr;
    case (EventType::kConfigureStream):
      if (incoming == EventType::kConfigureStream ||
          incoming == EventType::kFirstFrameStart) {
        break;
      }
      return nullptr;
    case (EventType::kFirstFrameStart):
      if (incoming == EventType::kFirstFrameEnd) {
        break;
      }
      return nullptr;
    case (EventType::kFlush):
      if (incoming == EventType::kFlush || incoming == EventType::kClose) {
        SetUseCase(camera_id_string_ + "-Close");
        break;
      }
      if (incoming == EventType::kConfigureStream) {
        SetUseCase(camera_id_string_ + "-Reconfiguration");
        break;
      }
      return nullptr;
    case (EventType::kClose):
      return nullptr;
    case (EventType::kFirstFrameEnd):
      ALOGE("%s: Warning: Operation %s should have already been deleted.",
            __FUNCTION__, EventTypeToString(state_).c_str());
      return nullptr;
  }
  UpdateStateLocked(incoming);
  IdleEndLocked();
  if (incoming == EventType::kConfigureStream) {
    id = config_count_++;
  } else if (incoming == EventType::kFlush) {
    id = flush_count_++;
  }

  return std::make_unique<AidlScopedProfiler>(
      profiler_, EventTypeToString(incoming), id, [this, incoming]() {
        if (incoming == EventType::kClose) {
          DeleteProfiler();
        } else {
          IdleStart();
        }
      });
}

bool TrackedProfiler::AcceptFirstFrameStart() {
  std::lock_guard<std::mutex> lock(tracked_api_mutex_);
  if (state_ == EventType::kConfigureStream) {
    UpdateStateLocked(EventType::kFirstFrameStart);
    IdleEndLocked();
    profiler_->Start(kFirstFrame, Profiler::kInvalidRequestId);
    profiler_->Start(kHalTotal, Profiler::kInvalidRequestId);
    return true;
  }
  return false;
}

bool TrackedProfiler::AcceptFirstFrameEnd() {
  std::lock_guard<std::mutex> lock(tracked_api_mutex_);
  if (state_ == EventType::kFirstFrameStart) {
    UpdateStateLocked(EventType::kFirstFrameEnd);
    profiler_->End(kFirstFrame, Profiler::kInvalidRequestId);
    profiler_->End(kHalTotal, Profiler::kInvalidRequestId);
    DeleteProfilerLocked();
    return true;
  }
  return false;
}

void TrackedProfiler::DeleteProfilerLocked() {
  if (profiler_ != nullptr) {
    profiler_->End(kOverall, Profiler::kInvalidRequestId);
    profiler_ = nullptr;  // Deletes the camera_latency_profiler, causing it
                          // to write the data to the Camer latency analyzer
  }
}

void TrackedProfiler::DeleteProfiler() {
  std::lock_guard<std::mutex> lock(tracked_api_mutex_);
  DeleteProfilerLocked();
}

void TrackedProfiler::IdleStartLocked() {
  if (idle_start_count_ > idle_end_count_) {
    ALOGE("%s: Error: Starting another idle before previous finished.",
          __FUNCTION__);
  } else if (idle_start_count_ < idle_end_count_) {
    ALOGE("%s: Error: More idles have finished than have started.",
          __FUNCTION__);
  }
  if (profiler_ != nullptr) {
    profiler_->Start(kIdleString, idle_start_count_++);
  }
}

void TrackedProfiler::IdleStart() {
  std::lock_guard<std::mutex> lock(tracked_api_mutex_);
  IdleStartLocked();
}

void TrackedProfiler::IdleEndLocked() {
  if (profiler_ != nullptr && idle_start_count_ - 1 == idle_end_count_) {
    profiler_->End(kIdleString, idle_end_count_++);
  }
}

}  // namespace google_camera_hal
}  // namespace android
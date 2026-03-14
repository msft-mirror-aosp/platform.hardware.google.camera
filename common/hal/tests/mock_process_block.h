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

#ifndef HARDWARE_GOOGLE_CAMERA_HAL_TESTS_MOCK_PROCESS_BLOCK_H_
#define HARDWARE_GOOGLE_CAMERA_HAL_TESTS_MOCK_PROCESS_BLOCK_H_

#include <gmock/gmock.h>
#include <process_block.h>
#include <result_processor.h>

namespace android {
namespace google_camera_hal {

// Defines a ProcessBlock mock using gmock.
class MockProcessBlock : public ProcessBlock {
 public:
  MOCK_METHOD(status_t, ConfigureStreams,
              (const StreamConfiguration& stream_config,
               const StreamConfiguration& overall_config),
              (override));

  MOCK_METHOD(status_t, SetResultProcessor,
              (std::unique_ptr<ResultProcessor> result_processor), (override));

  MOCK_METHOD(status_t, GetConfiguredHalStreams,
              (std::vector<HalStream> * hal_streams), (const, override));

  MOCK_METHOD(status_t, ProcessRequest,
              (ProcessBlockRequest process_block_request,
               const CaptureRequest& remaining_session_request),
              (override));

  MOCK_METHOD(status_t, ProcessBatchRequest,
              (std::vector<ProcessBlockRequest> process_block_request,
               const std::vector<CaptureRequest>& remaining_session_request),
              (override));

  MOCK_METHOD(status_t, Flush, (), (override));

  MOCK_METHOD(void, RepeatingRequestEnd,
              (int32_t frame_number, const std::vector<int32_t>& stream_ids),
              (override));
};

}  // namespace google_camera_hal
}  // namespace android

#endif  // HARDWARE_GOOGLE_CAMERA_HAL_TESTS_MOCK_PROCESS_BLOCK_H_
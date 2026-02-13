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

#ifndef HARDWARE_GOOGLE_CAMERA_HAL_GOOGLE_CAMERA_HAL_HAL_UTILS_H_
#define HARDWARE_GOOGLE_CAMERA_HAL_GOOGLE_CAMERA_HAL_HAL_UTILS_H_

#include "hal_types.h"
#include "hwl_types.h"
#include "process_block.h"
#include "utils.h"

namespace android {
namespace google_camera_hal {
namespace hal_utils {

// Create a HWL pipeline request for a pipeline based on a capture request.
HwlPipelineRequest CreateHwlPipelineRequest(uint32_t pipeline_id,
                                            CaptureRequest request);

// Convert a HWL result to a capture result.
std::unique_ptr<CaptureResult> ConvertToCaptureResult(
    std::unique_ptr<HwlPipelineResult> hwl_result);

// Return if the request contains an output buffer.
bool ContainsOutputBuffer(const CaptureRequest& request,
                          const buffer_handle_t& buffer);

// Return if all output buffers in remaining_session_request are included in
// process_block_request.
bool AreAllRemainingBuffersRequested(
    const ProcessBlockRequest& process_block_request,
    const CaptureRequest& remaining_session_request);

// Return if this is an IR camera.
bool IsIrCamera(const HalCameraMetadata* characteristics);

// Return if this is an MONO camera.
bool IsMonoCamera(const HalCameraMetadata* characteristics);

// Return if this is a bayer camera.
bool IsBayerCamera(const HalCameraMetadata* characteristics);

// Return if this is a HDR+ request
bool IsRequestHdrplusCompatible(const CaptureRequest& request,
                                int32_t preview_stream_id);

// Return true if this is a fixed-focus camera.
bool IsFixedFocusCamera(const HalCameraMetadata* characteristics);

// Return if HDR+ stream is supported
bool IsStreamHdrplusCompatible(const StreamConfiguration& stream_config,
                               const HalCameraMetadata* characteristics);

// Set ANDROID_CONTROL_ENABLE_ZSL metadata
status_t SetEnableZslMetadata(HalCameraMetadata* metadata, bool enable);

// Set hybrid AE vendor tag
status_t SetHybridAeMetadata(HalCameraMetadata* metadata, bool enable);

// Modify the request of realtime pipeline for HDR+
status_t ModifyRealtimeRequestForHdrplus(HalCameraMetadata* metadata,
                                         const bool hybrid_ae_enable = true);

// Get ANDROID_STATISTICS_FACE_DETECT_MODE
status_t GetFdMode(const CaptureRequest& request, uint8_t* face_detect_mode);

// Remove face detect information
status_t RemoveFdInfoFromResult(HalCameraMetadata* metadata);

// Force lens shading map mode on
status_t ForceLensShadingMapModeOn(HalCameraMetadata* metadata);

// Get lens shading map mode
status_t GetLensShadingMapMode(const CaptureRequest& request,
                               uint8_t* lens_shading_mode);

// Remove lens shading information
status_t RemoveLsInfoFromResult(HalCameraMetadata* metadata);

// Dump the information in the stream configuration
void DumpStreamConfiguration(const StreamConfiguration& stream_configuration,
                             const std::string& title);

// Dump the information in the HAL configured streams
void DumpHalConfiguredStreams(
    const std::vector<HalStream>& hal_configured_streams,
    const std::string& title);

// Dump the information in a capture request
void DumpCaptureRequest(const CaptureRequest& request, const std::string& title);

// Dump the information in a capture result
void DumpCaptureResult(const ProcessBlockResult& result,
                       const std::string& title);

// Dump the information in a capture result
void DumpCaptureResult(const CaptureResult& result, const std::string& title);

// Dump the information in a notification
void DumpNotify(const NotifyMessage& message, const std::string& title);

// Dump Stream
void DumpStream(const Stream& stream, const std::string& title);

// Dump HalStream
void DumpHalStream(const HalStream& hal_stream, const std::string& title);

// Dump the information in a buffer return
void DumpBufferReturn(const std::vector<StreamBuffer>& stream_buffers,
                      const std::string& title);

// Dump the information in a buffer request
void DumpBufferRequest(const std::vector<BufferRequest>& hal_buffer_requests,
                       const std::vector<BufferReturn>* hal_buffer_returns,
                       const std::string& title);

}  // namespace hal_utils
}  // namespace google_camera_hal
}  // namespace android

#endif  // HARDWARE_GOOGLE_CAMERA_HAL_GOOGLE_CAMERA_HAL_HAL_UTILS_H_
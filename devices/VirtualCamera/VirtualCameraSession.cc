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
#define LOG_TAG "VirtualCameraSession"
#include "VirtualCameraSession.h"

#include <atomic>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <tuple>
#include <utility>

#include "CameraMetadata.h"
#include "EGL/egl.h"
#include "VirtualCameraStream.h"
#include "aidl/android/hardware/camera/common/Status.h"
#include "aidl/android/hardware/camera/device/BufferCache.h"
#include "aidl/android/hardware/camera/device/CaptureRequest.h"
#include "aidl/android/hardware/camera/device/HalStream.h"
#include "aidl/android/hardware/camera/device/NotifyMsg.h"
#include "aidl/android/hardware/camera/device/ShutterMsg.h"
#include "aidl/android/hardware/camera/device/StreamConfiguration.h"
#include "aidl/android/hardware/camera/device/StreamRotation.h"
#include "aidl/android/hardware/graphics/common/BufferUsage.h"
#include "android/hardware_buffer.h"
#include "android/native_window.h"
#include "android/native_window_aidl.h"
#include "fmq/AidlMessageQueue.h"
#include "log/log.h"
#include "system/camera_metadata.h"
#include "system/graphics-sw.h"
#include "util/EglDisplayContext.h"
#include "util/EglFramebuffer.h"
#include "util/EglProgram.h"
#include "util/MetadataBuilder.h"
#include "util/TestPatternHelper.h"
#include "util/Util.h"

namespace android {
namespace companion {
namespace virtualcamera {

using ::aidl::android::companion::virtualcamera::IVirtualCameraCallback;
using ::aidl::android::hardware::camera::common::Status;
using ::aidl::android::hardware::camera::device::BufferCache;
using ::aidl::android::hardware::camera::device::BufferStatus;
using ::aidl::android::hardware::camera::device::CameraMetadata;
using ::aidl::android::hardware::camera::device::CameraOfflineSessionInfo;
using ::aidl::android::hardware::camera::device::CaptureRequest;
using ::aidl::android::hardware::camera::device::ErrorCode;
using ::aidl::android::hardware::camera::device::ErrorMsg;
using ::aidl::android::hardware::camera::device::HalStream;
using ::aidl::android::hardware::camera::device::ICameraDeviceCallback;
using ::aidl::android::hardware::camera::device::ICameraOfflineSession;
using ::aidl::android::hardware::camera::device::NotifyMsg;
using ::aidl::android::hardware::camera::device::RequestTemplate;
using ::aidl::android::hardware::camera::device::ShutterMsg;
using ::aidl::android::hardware::camera::device::StreamBuffer;
using ::aidl::android::hardware::camera::device::StreamConfiguration;
using ::aidl::android::hardware::camera::device::StreamRotation;
using ::aidl::android::hardware::common::fmq::MQDescriptor;
using ::aidl::android::hardware::common::fmq::SynchronizedReadWrite;
using ::aidl::android::hardware::graphics::common::BufferUsage;
using ::aidl::android::hardware::graphics::common::PixelFormat;
using ::android::base::unique_fd;

namespace {

using metadata_ptr =
    std::unique_ptr<camera_metadata_t, void (*)(camera_metadata_t*)>;

using namespace std::chrono_literals;

static constexpr std::chrono::milliseconds kAcquireFenceTimeout = 500ms;

// Size of request/result metadata fast message queue.
// Setting to 0 to always disables FMQ.
static constexpr size_t kMetadataMsgQueueSize = 0;

// Maximum number of buffers to use per single stream.
static constexpr size_t kMaxStreamBuffers = 2;

// Whether to use EGL for rendering - will be removed soon.
static constexpr bool kUseEGL = true;
// Whether to render through Surface -> EGL Texture -> Camera buffer.
// // TODO(b/301023410) Will be removed once we expose surface to the client API.
static constexpr bool kRenderThroughSurfaceTexture = true;

CameraMetadata createDefaultRequestSettings(RequestTemplate type) {
  hardware::camera::common::V1_0::helper::CameraMetadata metadataHelper;

  camera_metadata_enum_android_control_capture_intent_t intent =
      ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
  switch (type) {
    case RequestTemplate::PREVIEW:
      intent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
      break;
    case RequestTemplate::STILL_CAPTURE:
      intent = ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE;
      break;
    case RequestTemplate::VIDEO_RECORD:
      intent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD;
      break;
    case RequestTemplate::VIDEO_SNAPSHOT:
      intent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT;
      break;
    default:
      // Leave default.
      break;
  }

  auto metadata = MetadataBuilder().setControlCaptureIntent(intent).build();
  return (metadata != nullptr) ? std::move(*metadata) : CameraMetadata();
}

CameraMetadata createCaptureResultMetadata(
    const std::chrono::nanoseconds timestamp) {
  std::unique_ptr<CameraMetadata> metadata =
      MetadataBuilder().setSensorTimestamp(timestamp).build();
  if (metadata == nullptr) {
    ALOGE("%s: Failed to build capture result metadata", __func__);
    return CameraMetadata();
  }
  return std::move(*metadata);
}

NotifyMsg createShutterNotifyMsg(int frameNumber,
                                 std::chrono::nanoseconds timestamp) {
  NotifyMsg msg;
  msg.set<NotifyMsg::Tag::shutter>(ShutterMsg{
      .frameNumber = frameNumber,
      .timestamp = timestamp.count(),
  });
  return msg;
}

NotifyMsg createErrorNotifyMsg(int frameNumber, int streamId, ErrorCode error) {
  NotifyMsg msg;
  msg.set<NotifyMsg::Tag::error>(ErrorMsg{.frameNumber = frameNumber,
                                          .errorStreamId = streamId,
                                          .errorCode = error});
  return msg;
}

}  // namespace

VirtualCameraSession::VirtualCameraSession(
    const std::string& cameraId,
    std::shared_ptr<ICameraDeviceCallback> cameraDeviceCallback,
    std::shared_ptr<IVirtualCameraCallback> virtualCameraClientCallback)
    : mCameraId(cameraId),
      mCameraDeviceCallback(cameraDeviceCallback),
      mVirtualCameraClientCallback(virtualCameraClientCallback) {
  mRequestMetadataQueue = std::make_unique<RequestMetadataQueue>(
      kMetadataMsgQueueSize, false /* non blocking */);
  if (!mRequestMetadataQueue->isValid()) {
    ALOGE("%s: invalid request fmq", __func__);
  }

  mResultMetadataQueue = std::make_shared<ResultMetadataQueue>(
      kMetadataMsgQueueSize, false /* non blocking */);
  if (!mResultMetadataQueue->isValid()) {
    ALOGE("%s: invalid result fmq", __func__);
  }

  if (kUseEGL) {
    mEglDisplayContext = std::make_unique<EglDisplayContext>();
    if (kRenderThroughSurfaceTexture) {
      mEglTextureProgram = std::make_unique<EglTextureProgram>();
      // TODO(b/301023410) Initialize the texture based on supported formats and
      // formats requested by client in configure call.
      mEglSurfaceTexture = std::make_unique<EglSurfaceTexture>(640, 480);
    } else {
      mEglTestPatternProgram = std::make_unique<EglTestPatternProgram>();
    }
  }
}

ndk::ScopedAStatus VirtualCameraSession::close() {
  ALOGV("%s", __func__);

  if (mVirtualCameraClientCallback != nullptr) {
    mVirtualCameraClientCallback->onStreamClosed(/*streamId=*/0);
  }

  std::lock_guard<std::mutex> lock(mLock);
  mStreams.clear();
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::configureStreams(
    const StreamConfiguration& in_requestedConfiguration,
    std::vector<HalStream>* _aidl_return) {
  ALOGV("%s: requestedConfiguration: %s", __func__,
        in_requestedConfiguration.toString().c_str());

  if (_aidl_return == nullptr) {
    return cameraStatus(Status::ILLEGAL_ARGUMENT);
  }

  auto& streams = in_requestedConfiguration.streams;
  auto& halStreams = *_aidl_return;
  halStreams.clear();
  halStreams.resize(in_requestedConfiguration.streams.size());

  {
    std::lock_guard<std::mutex> lock(mLock);
    for (int i = 0; i < in_requestedConfiguration.streams.size(); ++i) {
      // TODO(b/301023410) remove hardcoded format checks, verify against configuration.
      if (streams[i].width != 640 || streams[i].height != 480 ||
          streams[i].rotation != StreamRotation::ROTATION_0 ||
          (streams[i].format != PixelFormat::IMPLEMENTATION_DEFINED &&
           streams[i].format != PixelFormat::YCBCR_420_888 &&
           streams[i].format != PixelFormat::BLOB)) {
        mStreams.clear();
        return cameraStatus(Status::ILLEGAL_ARGUMENT);
      }
      memset(&halStreams[i], 0x00, sizeof(HalStream));
      halStreams[i].id = streams[i].id;
      halStreams[i].physicalCameraId = streams[i].physicalCameraId;
      halStreams[i].overrideDataSpace = streams[i].dataSpace;
      halStreams[i].maxBuffers = kMaxStreamBuffers;
      halStreams[i].overrideFormat = PixelFormat::YCBCR_420_888;
      halStreams[i].producerUsage = kUseEGL ? BufferUsage::GPU_RENDER_TARGET
                                            : BufferUsage::CPU_WRITE_OFTEN;
      halStreams[i].supportOffline = false;

      mStreams.emplace(std::piecewise_construct,
                       std::forward_as_tuple(streams[i].id),
                       std::forward_as_tuple(streams[i]));
    }
  }

  if (mVirtualCameraClientCallback != nullptr && kUseEGL) {
    // TODO(b/301023410) Pass streamId based on client input stream id once
    // support for multiple input streams is implemented. For now we always
    // create single texture.
    mVirtualCameraClientCallback->onStreamConfigured(
        /*streamId=*/0,
        aidl::android::view::Surface(mEglSurfaceTexture->getSurface().get()),
        mEglSurfaceTexture->getWidth(), mEglSurfaceTexture->getHeight(),
        PixelFormat::YCBCR_420_888);
  }

  mFirstRequest.store(true);
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::constructDefaultRequestSettings(
    RequestTemplate in_type, CameraMetadata* _aidl_return) {
  ALOGV("%s: type %d", __func__, in_type);

  switch (in_type) {
    case RequestTemplate::PREVIEW:
    case RequestTemplate::STILL_CAPTURE:
    case RequestTemplate::VIDEO_RECORD: {
      *_aidl_return = createDefaultRequestSettings(in_type);
      return ndk::ScopedAStatus::ok();
    }
    case RequestTemplate::VIDEO_SNAPSHOT:
    case RequestTemplate::MANUAL:
    case RequestTemplate::ZERO_SHUTTER_LAG:
      // Don't support VIDEO_SNAPSHOT, MANUAL, ZSL templates
      return ndk::ScopedAStatus::fromServiceSpecificError(
          static_cast<int32_t>(Status::ILLEGAL_ARGUMENT));
      ;
    default:
      ALOGE("%s: unknown request template type %d", __FUNCTION__,
            static_cast<int>(in_type));
      return ndk::ScopedAStatus::fromServiceSpecificError(
          static_cast<int32_t>(Status::ILLEGAL_ARGUMENT));
      ;
  }
}

ndk::ScopedAStatus VirtualCameraSession::flush() {
  ALOGV("%s", __func__);
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::getCaptureRequestMetadataQueue(
    MQDescriptor<int8_t, SynchronizedReadWrite>* _aidl_return) {
  ALOGV("%s", __func__);
  *_aidl_return = mRequestMetadataQueue->dupeDesc();
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::getCaptureResultMetadataQueue(
    MQDescriptor<int8_t, SynchronizedReadWrite>* _aidl_return) {
  ALOGV("%s", __func__);
  *_aidl_return = mResultMetadataQueue->dupeDesc();
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::isReconfigurationRequired(
    const CameraMetadata& in_oldSessionParams,
    const CameraMetadata& in_newSessionParams, bool* _aidl_return) {
  ALOGV("%s: oldSessionParams: %s newSessionParams: %s", __func__,
        in_newSessionParams.toString().c_str(),
        in_oldSessionParams.toString().c_str());

  if (_aidl_return == nullptr) {
    return ndk::ScopedAStatus::fromServiceSpecificError(
        static_cast<int32_t>(Status::ILLEGAL_ARGUMENT));
  }

  *_aidl_return = true;
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::processCaptureRequest(
    const std::vector<CaptureRequest>& in_requests,
    const std::vector<BufferCache>& in_cachesToRemove, int32_t* _aidl_return) {
  ALOGV("%s", __func__);

  if (!in_cachesToRemove.empty()) {
    removeBufferCaches(in_cachesToRemove);
  }

  for (const auto& captureRequest : in_requests) {
    auto status = processCaptureRequest(captureRequest);
    if (!status.isOk()) {
      return status;
    }
  }
  *_aidl_return = in_requests.size();
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::signalStreamFlush(
    const std::vector<int32_t>& in_streamIds, int32_t in_streamConfigCounter) {
  ALOGV("%s", __func__);

  (void)in_streamIds;
  (void)in_streamConfigCounter;
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::switchToOffline(
    const std::vector<int32_t>& in_streamsToKeep,
    CameraOfflineSessionInfo* out_offlineSessionInfo,
    std::shared_ptr<ICameraOfflineSession>* _aidl_return) {
  ALOGV("%s", __func__);

  (void)in_streamsToKeep;
  (void)out_offlineSessionInfo;

  if (_aidl_return == nullptr) {
    return ndk::ScopedAStatus::fromServiceSpecificError(
        static_cast<int32_t>(Status::ILLEGAL_ARGUMENT));
  }

  *_aidl_return = nullptr;
  return cameraStatus(Status::OPERATION_NOT_SUPPORTED);
}

ndk::ScopedAStatus VirtualCameraSession::repeatingRequestEnd(
    int32_t in_frameNumber, const std::vector<int32_t>& in_streamIds) {
  ALOGV("%s", __func__);
  (void)in_frameNumber;
  (void)in_streamIds;
  return ndk::ScopedAStatus::ok();
}

void VirtualCameraSession::removeBufferCaches(
    const std::vector<BufferCache>& cachesToRemove) {
  std::lock_guard<std::mutex> lock(mLock);
  for (const auto& bufferCache : cachesToRemove) {
    auto it = mStreams.find(bufferCache.streamId);
    if (it == mStreams.end()) {
      ALOGE("%s: Ask to remove buffer %" PRId64 " from unknown stream %d",
            __func__, bufferCache.bufferId, bufferCache.streamId);
      continue;
    }
    if (it->second.removeBuffer(bufferCache.bufferId)) {
      ALOGD("%s: Successfully removed buffer %" PRId64
            " from cache of stream %d",
            __func__, bufferCache.bufferId, bufferCache.streamId);
    } else {
      ALOGE("%s: Failed to buffer %" PRId64 " from cache of stream %d",
            __func__, bufferCache.bufferId, bufferCache.streamId);
    }
  }
}

std::shared_ptr<AHardwareBuffer> VirtualCameraSession::fetchHardwareBuffer(
    const StreamBuffer& streamBuffer) {
  std::lock_guard<std::mutex> lock(mLock);
  auto it = mStreams.find(streamBuffer.streamId);
  if (it == mStreams.end()) {
    ALOGE("%s: StreamBuffer references buffer of unknown streamId %d", __func__,
          streamBuffer.streamId);
    return nullptr;
  }
  return it->second.getHardwareBuffer(streamBuffer);
}

std::shared_ptr<EglFrameBuffer> VirtualCameraSession::fetchEglFramebuffer(
    const EGLDisplay eglDisplay, const StreamBuffer& streamBuffer) {
  std::lock_guard<std::mutex> lock(mLock);
  auto it = mStreams.find(streamBuffer.streamId);
  if (it == mStreams.end()) {
    ALOGE("%s: StreamBuffer references buffer of unknown streamId %d", __func__,
          streamBuffer.streamId);
    return nullptr;
  }
  return it->second.getEglFrameBuffer(eglDisplay, streamBuffer);
}

ndk::ScopedAStatus VirtualCameraSession::processCaptureRequest(
    const CaptureRequest& request) {
  ALOGD("%s: request: %s", __func__, request.toString().c_str());

  if (mFirstRequest.exchange(false) && request.settings.metadata.empty()) {
    return cameraStatus(Status::ILLEGAL_ARGUMENT);
  }

  std::shared_ptr<ICameraDeviceCallback> cameraCallback = nullptr;
  {
    std::lock_guard<std::mutex> lock(mLock);
    cameraCallback = mCameraDeviceCallback;
  }

  if (cameraCallback == nullptr) {
    ALOGE(
        "%s: processCaptureRequest called, but there's no camera callback "
        "configured",
        __func__);
    return cameraStatus(Status::INTERNAL_ERROR);
  }

  const std::chrono::nanoseconds timestamp =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch());

  ::aidl::android::hardware::camera::device::CaptureResult captureResult;
  captureResult.fmqResultSize = 0;
  captureResult.frameNumber = request.frameNumber;
  captureResult.result = request.settings;
  captureResult.partialResult = 1;
  captureResult.inputBuffer.streamId = -1;
  captureResult.outputBuffers.resize(request.outputBuffers.size());
  captureResult.physicalCameraMetadata.resize(0);
  captureResult.result = createCaptureResultMetadata(timestamp);

  for (int i = 0; i < request.outputBuffers.size(); ++i) {
    StreamBuffer& outBuffer = captureResult.outputBuffers[i];
    outBuffer.streamId = request.outputBuffers[i].streamId;
    outBuffer.bufferId = request.outputBuffers[i].bufferId;
    if (renderIntoStreamBuffer(request, request.outputBuffers[i]).isOk()) {
      cameraCallback->notify(
          {createShutterNotifyMsg(request.frameNumber, timestamp)});
      outBuffer.status = BufferStatus::OK;
    } else {
      cameraCallback->notify({createErrorNotifyMsg(
          request.frameNumber, outBuffer.bufferId, ErrorCode::ERROR_BUFFER)});
      outBuffer.status = BufferStatus::ERROR;
    }
  }

  std::vector<::aidl::android::hardware::camera::device::CaptureResult>
      captureResults(1);
  captureResults[0] = std::move(captureResult);

  auto status = cameraCallback->processCaptureResult(captureResults);
  if (!status.isOk()) {
    ALOGE("%s: processCaptureResult call failed: %s", __func__,
          status.getDescription().c_str());
    return cameraStatus(Status::INTERNAL_ERROR);
  }

  ALOGD("%s: Successfully called processCaptureResult", __func__);

  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VirtualCameraSession::renderIntoStreamBuffer(
    const ::aidl::android::hardware::camera::device::CaptureRequest& request,
    const ::aidl::android::hardware::camera::device::StreamBuffer& streamBuffer) {
  sp<Fence> fence = importFence(streamBuffer.acquireFence);
  if (kUseEGL && mEglDisplayContext->isInitialized()) {
    // Wait for fence to clear.
    if (fence->isValid()) {
      status_t ret = fence->wait(kAcquireFenceTimeout.count());
      if (ret != 0) {
        ALOGE("Timeout while waiting for the acquire fence for buffer%" PRId64
              " for streamId %d",
              streamBuffer.bufferId, streamBuffer.streamId);
        return cameraStatus(Status::INTERNAL_ERROR);
      }
    }

    // Render test pattern using EGL.
    std::shared_ptr<EglFrameBuffer> framebuffer =
        fetchEglFramebuffer(mEglDisplayContext->getEglDisplay(), streamBuffer);
    if (framebuffer == nullptr) {
      ALOGE(
          "%s: Failed to get EGL framebuffer corresponding to buffer id "
          "%" PRId64 " for streamId %d",
          __func__, streamBuffer.bufferId, streamBuffer.streamId);
      return cameraStatus(Status::ILLEGAL_ARGUMENT);
    }

    if (kRenderThroughSurfaceTexture &&
        mVirtualCameraClientCallback == nullptr) {
      // Since we don't have client API yet to pass Surface to, let's just
      // render something to the Surface ourselves.
      renderTestPatternYCbCr420(mEglSurfaceTexture->getSurface(),
                                request.frameNumber);
    }

    mEglDisplayContext->makeCurrent();
    framebuffer->beforeDraw();

    if (kRenderThroughSurfaceTexture) {
      mEglTextureProgram->draw(mEglSurfaceTexture->updateTexture());
    } else {
      mEglTestPatternProgram->draw(framebuffer->getWidth(),
                                   framebuffer->getHeight(),
                                   request.frameNumber);
    }
    framebuffer->afterDraw();
  } else {
    // Render test pattern on CPU.
    std::shared_ptr<AHardwareBuffer> hwBuffer =
        fetchHardwareBuffer(streamBuffer);
    if (hwBuffer == nullptr) {
      ALOGE("%s: Failed to get hardware buffer id %" PRId64 " for streamId %d",
            __func__, streamBuffer.bufferId, streamBuffer.streamId);
      return cameraStatus(Status::ILLEGAL_ARGUMENT);
    }
    renderTestPatternYCbCr420(hwBuffer, request.frameNumber, fence->get());
  }

  return ndk::ScopedAStatus::ok();
}

}  // namespace virtualcamera
}  // namespace companion
}  // namespace android

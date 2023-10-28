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

#include <cstdint>
#include <memory>

#include "VirtualCameraSession.h"
#include "aidl/android/companion/virtualcamera/BnVirtualCameraCallback.h"
#include "aidl/android/companion/virtualcamera/IVirtualCameraCallback.h"
#include "aidl/android/hardware/camera/device/BnCameraDeviceCallback.h"
#include "aidl/android/hardware/camera/device/StreamConfiguration.h"
#include "aidl/android/hardware/graphics/common/PixelFormat.h"
#include "android/binder_auto_utils.h"
#include "android/binder_interface_utils.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace android {
namespace companion {
namespace virtualcamera {
namespace {

const std::string kCameraName = "virtual_camera";

using ::aidl::android::companion::virtualcamera::BnVirtualCameraCallback;
using ::aidl::android::hardware::camera::device::BnCameraDeviceCallback;
using ::aidl::android::hardware::camera::device::BufferRequest;
using ::aidl::android::hardware::camera::device::BufferRequestStatus;
using ::aidl::android::hardware::camera::device::CaptureResult;
using ::aidl::android::hardware::camera::device::HalStream;
using ::aidl::android::hardware::camera::device::NotifyMsg;
using ::aidl::android::hardware::camera::device::Stream;
using ::aidl::android::hardware::camera::device::StreamBuffer;
using ::aidl::android::hardware::camera::device::StreamBufferRet;
using ::aidl::android::hardware::camera::device::StreamConfiguration;
using ::aidl::android::hardware::graphics::common::PixelFormat;
using ::aidl::android::view::Surface;
using ::testing::_;
using ::testing::ElementsAre;
using ::testing::Return;
using ::testing::SizeIs;

Stream createStream(int streamId, int width, int height, PixelFormat format) {
  Stream s;
  s.id = streamId;
  s.width = width;
  s.height = height;
  s.format = format;
  return s;
}

class MockCameraDeviceCallback : public BnCameraDeviceCallback {
 public:
  MOCK_METHOD(ndk::ScopedAStatus, notify, (const std::vector<NotifyMsg>&),
              (override));
  MOCK_METHOD(ndk::ScopedAStatus, processCaptureResult,
              (const std::vector<CaptureResult>&), (override));
  MOCK_METHOD(ndk::ScopedAStatus, requestStreamBuffers,
              (const std::vector<BufferRequest>&, std::vector<StreamBufferRet>*,
               BufferRequestStatus*),
              (override));
  MOCK_METHOD(ndk::ScopedAStatus, returnStreamBuffers,
              (const std::vector<StreamBuffer>&), (override));
};

class MockVirtualCameraCallback : public BnVirtualCameraCallback {
 public:
  MOCK_METHOD(ndk::ScopedAStatus, onStreamConfigured,
              (int, const Surface&, int32_t, int32_t, PixelFormat), (override));
  MOCK_METHOD(ndk::ScopedAStatus, onStreamClosed, (int), (override));
};

class VirtualCameraSessionTest : public ::testing::Test {
 public:
  void SetUp() override {
    mMockCameraDeviceCallback =
        ndk::SharedRefBase::make<MockCameraDeviceCallback>();
    mMockVirtualCameraClientCallback =
        ndk::SharedRefBase::make<MockVirtualCameraCallback>();
    mVirtualCameraSession = ndk::SharedRefBase::make<VirtualCameraSession>(
        kCameraName, mMockCameraDeviceCallback,
        mMockVirtualCameraClientCallback);

    ON_CALL(*mMockVirtualCameraClientCallback, onStreamConfigured)
        .WillByDefault(ndk::ScopedAStatus::ok);
  }

 protected:
  std::shared_ptr<MockCameraDeviceCallback> mMockCameraDeviceCallback;
  std::shared_ptr<MockVirtualCameraCallback> mMockVirtualCameraClientCallback;
  std::shared_ptr<VirtualCameraSession> mVirtualCameraSession;
};

TEST_F(VirtualCameraSessionTest, ConfigureTriggersClientConfigureCallback) {
  int streamId = 0;
  int width = 640;
  int height = 480;
  PixelFormat format = PixelFormat::YCBCR_420_888;
  StreamConfiguration streamConfiguration;
  streamConfiguration.streams = {createStream(streamId, width, height, format)};
  std::vector<HalStream> halStreams;
  EXPECT_CALL(*mMockVirtualCameraClientCallback,
              onStreamConfigured(streamId, _, width, height, format));

  ASSERT_TRUE(
      mVirtualCameraSession->configureStreams(streamConfiguration, &halStreams)
          .isOk());

  EXPECT_THAT(halStreams, SizeIs(streamConfiguration.streams.size()));
  EXPECT_THAT(mVirtualCameraSession->getStreamIds(), ElementsAre(0));
}

TEST_F(VirtualCameraSessionTest, SecondConfigureDropsUnreferencedStreams) {
  int width = 640;
  int height = 480;
  PixelFormat format = PixelFormat::YCBCR_420_888;
  StreamConfiguration streamConfiguration;
  std::vector<HalStream> halStreams;

  streamConfiguration.streams = {createStream(0, width, height, format),
                                 createStream(1, width, height, format),
                                 createStream(2, width, height, format)};
  ASSERT_TRUE(
      mVirtualCameraSession->configureStreams(streamConfiguration, &halStreams)
          .isOk());

  EXPECT_THAT(mVirtualCameraSession->getStreamIds(), ElementsAre(0, 1, 2));

  streamConfiguration.streams = {createStream(0, width, height, format),
                                 createStream(2, width, height, format),
                                 createStream(3, width, height, format)};
  ASSERT_TRUE(
      mVirtualCameraSession->configureStreams(streamConfiguration, &halStreams)
          .isOk());

  EXPECT_THAT(mVirtualCameraSession->getStreamIds(), ElementsAre(0, 2, 3));
}

TEST_F(VirtualCameraSessionTest, CloseTriggersClientTerminateCallback) {
  int streamId = 0;
  EXPECT_CALL(*mMockVirtualCameraClientCallback, onStreamClosed(streamId))
      .WillOnce(Return(ndk::ScopedAStatus::ok()));

  ASSERT_TRUE(mVirtualCameraSession->close().isOk());
}

}  // namespace
}  // namespace virtualcamera
}  // namespace companion
}  // namespace android

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

#include <cstdio>
#include <memory>

#include "VirtualCameraService.h"
#include "aidl/android/companion/virtualcamera/BnVirtualCameraCallback.h"
#include "aidl/android/companion/virtualcamera/VirtualCameraConfiguration.h"
#include "aidl/android/hardware/camera/provider/BnCameraProviderCallback.h"
#include "aidl/android/hardware/graphics/common/PixelFormat.h"
#include "android/binder_auto_utils.h"
#include "android/binder_interface_utils.h"
#include "android/binder_libbinder.h"
#include "binder/Binder.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "utils/Errors.h"

namespace android {
namespace companion {
namespace virtualcamera {
namespace {

using ::aidl::android::companion::virtualcamera::BnVirtualCameraCallback;
using ::aidl::android::companion::virtualcamera::VirtualCameraConfiguration;
using ::aidl::android::hardware::camera::common::CameraDeviceStatus;
using ::aidl::android::hardware::camera::common::TorchModeStatus;
using ::aidl::android::hardware::camera::provider::BnCameraProviderCallback;
using ::aidl::android::hardware::graphics::common::PixelFormat;
using ::aidl::android::view::Surface;
using ::testing::_;
using ::testing::Eq;
using ::testing::Ge;
using ::testing::IsNull;
using ::testing::Not;

const VirtualCameraConfiguration kEmptyVirtualCameraConfiguration;

class MockCameraProviderCallback : public BnCameraProviderCallback {
 public:
  MOCK_METHOD(ndk::ScopedAStatus, cameraDeviceStatusChange,
              (const std::string&, CameraDeviceStatus), (override));
  MOCK_METHOD(ndk::ScopedAStatus, torchModeStatusChange,
              (const std::string&, TorchModeStatus), (override));
  MOCK_METHOD(ndk::ScopedAStatus, physicalCameraDeviceStatusChange,
              (const std::string&, const std::string&, CameraDeviceStatus),
              (override));
};

class VirtualCameraServiceTest : public ::testing::Test {
 public:
  void SetUp() override {
    mCameraProvider = ndk::SharedRefBase::make<VirtualCameraProvider>();
    mMockCameraProviderCallback =
        ndk::SharedRefBase::make<MockCameraProviderCallback>();
    ON_CALL(*mMockCameraProviderCallback, cameraDeviceStatusChange)
        .WillByDefault([](const std::string&, CameraDeviceStatus) {
          return ndk::ScopedAStatus::ok();
        });
    mCameraProvider->setCallback(mMockCameraProviderCallback);
    mCameraService =
        ndk::SharedRefBase::make<VirtualCameraService>(mCameraProvider);

    mDevNullFd = open("/dev/null", O_RDWR);
    ASSERT_THAT(mDevNullFd, Ge(0));
  }

  void createCamera() {
    mOwnerToken = sp<BBinder>::make();
    mNdkOwnerToken.set(AIBinder_fromPlatformBinder(mOwnerToken));
    bool aidlRet;

    ASSERT_TRUE(mCameraService
                    ->registerCamera(mNdkOwnerToken,
                                     kEmptyVirtualCameraConfiguration, &aidlRet)
                    .isOk());
    ASSERT_TRUE(aidlRet);
  }

  void TearDown() override {
    close(mDevNullFd);
  }

  void execute_shell_command(const std::string cmd) {
    std::array<const char*, 1> args{cmd.data()};
    ASSERT_THAT(
        mCameraService->handleShellCommand(mDevNullFd, mDevNullFd, mDevNullFd,
                                           args.data(), args.size()),
        Eq(NO_ERROR));
  }

 protected:
  std::shared_ptr<VirtualCameraService> mCameraService;
  std::shared_ptr<VirtualCameraProvider> mCameraProvider;
  std::shared_ptr<MockCameraProviderCallback> mMockCameraProviderCallback =
      ndk::SharedRefBase::make<MockCameraProviderCallback>();

  sp<BBinder> mOwnerToken;
  ndk::SpAIBinder mNdkOwnerToken;

  int mDevNullFd;
};

TEST_F(VirtualCameraServiceTest, RegisterCameraSucceeds) {
  sp<BBinder> token = sp<BBinder>::make();
  ndk::SpAIBinder ndkToken(AIBinder_fromPlatformBinder(token));
  bool aidlRet;

  ASSERT_TRUE(
      mCameraService
          ->registerCamera(ndkToken, kEmptyVirtualCameraConfiguration, &aidlRet)
          .isOk());

  EXPECT_TRUE(aidlRet);
}

TEST_F(VirtualCameraServiceTest, RegisterCameraTwiceSecondReturnsFalse) {
  createCamera();
  bool aidlRet;

  ASSERT_TRUE(mCameraService
                  ->registerCamera(mNdkOwnerToken,
                                   kEmptyVirtualCameraConfiguration, &aidlRet)
                  .isOk());
  EXPECT_FALSE(aidlRet);
}

TEST_F(VirtualCameraServiceTest, GetCamera) {
  createCamera();

  EXPECT_THAT(mCameraService->getCamera(mNdkOwnerToken), Not(IsNull()));

  sp<BBinder> otherToken = sp<BBinder>::make();
  EXPECT_THAT(mCameraService->getCamera(
                  ndk::SpAIBinder(AIBinder_fromPlatformBinder(otherToken))),
              IsNull());
}

TEST_F(VirtualCameraServiceTest, UnregisterCamera) {
  createCamera();

  EXPECT_THAT(mCameraService->getCamera(mNdkOwnerToken), Not(IsNull()));

  mCameraService->unregisterCamera(mNdkOwnerToken);

  EXPECT_THAT(mCameraService->getCamera(mNdkOwnerToken), IsNull());
}

}  // namespace
}  // namespace virtualcamera
}  // namespace companion
}  // namespace android

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

#define LOG_TAG "android.hardware.pixel.camera.provider@2.4-service"

#include <android/hardware/camera/provider/2.4/ICameraProvider.h>
#include <binder/ProcessState.h>
#include <hidl/LegacySupport.h>

using android::hardware::defaultPassthroughServiceImplementation;
using android::hardware::camera::provider::V2_4::ICameraProvider;

int main() {
  ALOGI("Google camera provider service is starting.");
  // The camera HAL may communicate to other vendor components via
  // /dev/vndbinder
  android::ProcessState::initWithDriver("/dev/vndbinder");
  int res = defaultPassthroughServiceImplementation<ICameraProvider>(
      "internal/0", /*maxThreads*/ 6);

  ALOGE("Google camera provider service ending with res %d", res);
  return res;
}
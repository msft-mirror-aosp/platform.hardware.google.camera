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

#include "gtest/gtest.h"
#include "util/EglDisplayContext.h"
#include "util/EglProgram.h"

namespace android {
namespace companion {
namespace virtualcamera {
namespace {

TEST(EglDisplayContextTest, SuccessfulInitialization) {
  EglDisplayContext displayContext;

  EXPECT_TRUE(displayContext.isInitialized());
}

TEST(EglTestPatternProgram, SuccessfulInitialization) {
  EglDisplayContext displayContext;
  ASSERT_TRUE(displayContext.isInitialized());
  ASSERT_TRUE(displayContext.makeCurrent());

  EglTestPatternProgram eglTestPatternProgram;

  // Verify the shaders compiled and linked successfully.
  EXPECT_TRUE(eglTestPatternProgram.isInitialized());
}

TEST(EglTextureProgram, SuccessfulInitialization) {
  EglDisplayContext displayContext;
  ASSERT_TRUE(displayContext.isInitialized());
  ASSERT_TRUE(displayContext.makeCurrent());

  EglTextureProgram eglTextureProgram;

  // Verify the shaders compiled and linked successfully.
  EXPECT_TRUE(eglTextureProgram.isInitialized());
}

}  // namespace
}  // namespace virtualcamera
}  // namespace companion
}  // namespace android

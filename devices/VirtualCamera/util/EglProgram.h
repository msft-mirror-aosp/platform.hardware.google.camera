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

#ifndef ANDROID_SERVICES_VIRTUALCAMERA_EGLPROGRAM_H
#define ANDROID_SERVICES_VIRTUALCAMERA_EGLPROGRAM_H

#include <complex>

#include "GLES/gl.h"

namespace android {
namespace services {
namespace virtualcamera {

// Base class for EGL Shader programs representation.
class EglProgram {
 public:
  virtual ~EglProgram();

 protected:
  // Compile & link program from the vertex & fragment shader source.
  bool initialize(const char* vertexShaderSrc, const char* fragmentShaderSrc);
  GLuint mProgram;
};

// Shader program to draw Julia Set test pattern.
class EglTestPatternProgram : public EglProgram {
 public:
  EglTestPatternProgram();

  bool draw(int width, int height, int frameNumber);

 private:
};

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

#endif

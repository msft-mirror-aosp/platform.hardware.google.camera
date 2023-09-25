#include "Util.h"

#define LOG_TAG "VirtualCameraUtil"
#include <unistd.h>

#include "log/log_main.h"

namespace android {
namespace services {
namespace virtualcamera {

using ::aidl::android::hardware::common::NativeHandle;

FenceGuard::FenceGuard(const NativeHandle& aidlHandle) {
  if (aidlHandle.fds.size() != 1) {
    ALOGE(
        "Cannot import fence from aidlHandle containing %ld file descriptors.",
        aidlHandle.fds.size());
    return;
  }

  mFd = ::dup(aidlHandle.fds[0].get());
}

FenceGuard::~FenceGuard() {
  if (mFd > 0) {
    ::close(mFd);
    mFd = -1;
  }
}

int FenceGuard::get() const {
  return mFd;
}

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

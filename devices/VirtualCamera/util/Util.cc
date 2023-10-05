#include "Util.h"

#define LOG_TAG "VirtualCameraUtil"

#include <unistd.h>

#include "log/log_main.h"

namespace android {
namespace services {
namespace virtualcamera {

using ::aidl::android::hardware::common::NativeHandle;

sp<Fence> importFence(const NativeHandle& aidlHandle) {
  if (aidlHandle.fds.size() != 1) {
    ALOGE(
        "%s: Cannot import fence from aidlHandle containing %d file "
        "descriptors.",
        __func__, static_cast<int>(aidlHandle.fds.size()));
    return sp<Fence>::make();
  }

  return sp<Fence>::make(::dup(aidlHandle.fds[0].get()));
}

}  // namespace virtualcamera
}  // namespace services
}  // namespace android

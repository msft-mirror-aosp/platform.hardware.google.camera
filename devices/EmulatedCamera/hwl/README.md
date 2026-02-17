# Emulated Camera HWL (Hardware Wrapper Layer)

This directory contains the implementation of the Hardware Wrapper Layer (HWL) for Google's Emulated Camera HAL. It allows for the creation and configuration of virtual camera devices with customizable characteristics and behaviors using JSON configuration files.

## Configuration Overview

The emulated camera provider uses a JSON-based configuration system to define the available camera devices and their static metadata.

### Configuration Files Location

Configuration files are typically located in:
*   `/vendor/etc/config/`
*   `/apex/com.google.emulated.camera.provider.hal/etc/config/` (if using APEX)

### Main Configuration File

The entry point is the main configuration file, usually named `emu_camera_main.json` (or `emu_camera_automotive_main.json` for automotive builds). This file lists all the camera devices that the provider should expose.

**Example `emu_camera_main.json`:**
```json
{
  "cameras": [
    {
      "type": "back",
      "filename": "emu_camera_back.json"
    },
    {
      "type": "front",
      "filename": "emu_camera_front.json",
      "source": "color_bar"
    }
  ]
}
```

**Fields:**
*   `type`: Used to identify the camera role (e.g., "back", "front", "external"). This matches against system properties (like `ro.vendor.camera.config`) to determine which cameras to enable in specific emulator configurations.
*   `filename`: The name of the detailed configuration file for this specific camera.
*   `source`: (Optional) The frame source type. Options:
    *   `emulated_scene` (default): Simulates a physical sensor capturing a 3D scene.
    *   `color_bar`: Generates static test pattern color bars.
    *   `video`: Plays a video file.
*   `file_path`: (Optional) Full path to the video file if `source` is set to `video`.

### Camera Characteristic Files

Each camera defined in the main config points to a separate JSON file (e.g., `emu_camera_back.json`) that defines its static metadata (CameraCharacteristics).

**Format:**
The file is a JSON object where keys are **Android Camera Metadata tag names** and values are **arrays of strings**.

**Example `emu_camera_back.json`:**
```json
{
  "android.lens.facing": [ "BACK" ],
  "android.sensor.orientation": [ "90" ],
  "android.sensor.info.activeArraySize": [ "0", "0", "640", "480" ],
  "android.sensor.info.pixelArraySize": [ "640", "480" ],
  "android.scaler.availableStreamConfigurations": [
    "33", "640", "480", "OUTPUT",
    "34", "640", "480", "OUTPUT"
  ]
}
```

**Notes:**
*   Even single values must be enclosed in an array (e.g., `["BACK"]`).
*   Numeric values are strings (e.g., `["90"]`).
*   Enums can be specified by name (e.g., `"BACK"`) or integer value.

### Logical Multi-Camera Configuration

To define a logical multi-camera, the characteristics file should contain a JSON **array** of objects instead of a single object.

1.  **First Object**: Characteristics for the **Logical** Camera.
2.  **Subsequent Objects**: Characteristics for each backing **Physical** Camera.

**Example Structure:**
```json
[
  {
    "android.request.availableCapabilities": [ "LOGICAL_MULTI_CAMERA", ... ],
    ...
  },
  {
    "android.lens.facing": [ "BACK" ],
    ...
  },
  {
    "android.lens.facing": [ "BACK" ],
    ...
  }
]
```

## How to Create and Use a New Config

1. **Create or Generate the Camera Config File**:

   You can either generate the configuration automatically from a real device bug report (recommended) or create it manually.

   *   **Option A: Automatic Generation (Recommended)**
       Use the `parse_bugreport.py` tool to generate a JSON configuration from an Android bug report. This is the intended way to emulate real-world devices.

       ```bash
       python3 hardware/google/camera/devices/EmulatedCamera/hwl/tools/parse_bugreport.py <path_to_bugreport> <camera_device_id> <output_json_path>
       ```

       **Note:** Some manual customization may still be required after generation, as the emulator may not support every feature reported by the physical device.

   *   **Option B: Manual Creation**
       *   Create a new JSON file (e.g., `my_new_camera.json`) in the `configs/` directory.
       *   Populate it with the desired camera characteristics (metadata tags). Use existing files like `emu_camera_back.json` as a template.


2.  **Register in Main Config**:
    *   Open `configs/emu_camera_main.json`.
    *   Add a new entry to the `cameras` array pointing to your new file:
        ```json
        {
          "type": "back",
          "filename": "my_new_camera.json"
        }
        ```

3.  **Update Build Rules**:
    *   Open `configs/Android.bp`.
    *   Add your new file (`my_new_camera.json`) to the `prebuilt_etc` modules to ensure it gets installed to the target device/emulator image.

    ```bp
    prebuilt_etc {
        name: "my_new_camera.json",
        src: "my_new_camera.json",
        defaults: ["emu_camera_config_defaults"],
    }
    ```
    *   Ensure the main config module (e.g., `emu_camera_main.json` in `Android.bp` or the APEX definition) depends on or includes this new file if necessary, though `prebuilt_etc` usually handles installation to `/vendor/etc`.

4.  **Rebuild and Run**:
    *   Rebuild the system image or the specific APEX module.
    *   Start the emulator/device. The new camera should be enumerated by the camera service.

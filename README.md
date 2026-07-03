# ESP32 OpenCV Projects

A collection of standalone computer-vision projects running **OpenCV directly on ESP32 microcontrollers** — no host PC, no cloud, no Linux SBC. Each subdirectory is an independent ESP-IDF project with its own hardware setup and README.

## Projects

### [`dice_dice/`](dice_dice/README.md) — Real-Time Dice Pip Counter (ESP32-S3)

Detects and counts dice pips from a live camera feed using a classical OpenCV pipeline (Otsu threshold, morphology, connected components) and shows the result as an animated overlay on an LCD.

<p align="center">
  <img src="dice_dice/out/1_gray.png" width="200" alt="Grayscale conversion"/>
  <img src="dice_dice/out/4_dark_mask.png" width="200" alt="Otsu threshold mask"/>
  <img src="dice_dice/out/3_die_solid.png" width="200" alt="Morphological cleanup"/>
  <img src="dice_dice/out/3_result.png" width="200" alt="Final pip-count result"/>
</p>

[Watch the demo on YouTube](https://youtu.be/1KdL57JlomE)

### [`legacy_project/color_code/`](legacy_project/color_code/README.md) — RGB Pixel Detector & Histogram (2022)

*No longer maintained, kept for reference.* Reads the center pixel's RGB value and draws a real-time color histogram on an ESP32 TTGO T-Camera.

<p align="center">
  <img src="legacy_project/color_code/demo/demo.gif" width="400" alt="RGB pixel detector and histogram demo"/>
</p>

[Watch the demo on YouTube](https://youtu.be/DNQuCkPtzYA)

## About OpenCV on ESP32

Running OpenCV on a microcontroller has historically required a manual cross-compilation of the library for the target toolchain. The projects in this repo reflect that evolution:

- **`legacy_project/`** is based on [joachimBurket/esp32-opencv](https://github.com/joachimBurket/esp32-opencv), a hand cross-compiled OpenCV port for the ESP32 (Xtensa LX6). It is preserved for reference but is no longer used or maintained.
- **`dice_dice/`** and current/future projects instead use [**`100312dog/opencv-mobile-prebuilt-newlib`**](https://components.espressif.com/components/100312dog/opencv-mobile-prebuilt-newlib), a prebuilt OpenCV component distributed through the ESP-IDF Component Manager. Since it ships already built for the newlib toolchain, it can simply be added as a one-line dependency in `idf_component.yml` — no manual cross-compilation needed.

See each project's own README for hardware requirements, build instructions, and implementation details.

## Limitations of OpenCV on ESP32

OpenCV on ESP32 is **not** the full desktop library — it only exposes a stripped-down subset that fits within the chip's limited flash and RAM:

- Only the **`core`** and **`imgproc`** modules are available (basic matrix ops, filtering, thresholding, morphology, contours/connected components, histograms). Both `legacy_project` and `dice_dice` are built against this same reduced module set.
- No `dnn`, `ml`, `video`, `videoio`, `calib3d`, `features2d`, or `objdetect` modules — no built-in face/object detection, optical flow, camera calibration, or neural-network inference.
- No hardware-accelerated backends (CUDA/OpenCL) — everything runs single-threaded on the Xtensa/RISC-V CPU core, so complex algorithms are either too slow or too large to run at usable frame rates.
- As a result, every project here relies on **classical, lightweight CV pipelines** (grayscale conversion, Otsu thresholding, morphological filtering, connected-component analysis) rather than heavier techniques like deep-learning-based detection.

## License

[MIT License](LICENSE)

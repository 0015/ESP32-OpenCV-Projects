/*
 * cube_app — LVGL UI + application state machine for the cube solver.
 *
 * Flow: START -> SCAN(6 faces, 3x3 grid guide, live HSV colors)
 *       -> REVIEW (tap cell to fix color manually) -> validate
 *       -> ERROR ("rescan face X") or GUIDE (step-by-step moves + 2D net)
 *       -> DONE
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Build all screens. LVGL display/touch must already be initialized and
 * img_camera (main.c) created. Locks LVGL internally. */
void cube_app_init(void);

/* Called from the detection task for every captured frame copy.
 * Performs HSV sampling while scanning. Never touches LVGL. */
void cube_app_on_frame(const uint8_t *rgb565, int w, int h);

/* True while the live camera preview should be shown/updated. */
bool cube_app_camera_active(void);

#ifdef __cplusplus
}
#endif

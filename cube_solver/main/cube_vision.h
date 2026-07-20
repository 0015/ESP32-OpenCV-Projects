/*
 * cube_vision — sample the 9 grid cells of a camera frame, convert to HSV
 * (OpenCV cvtColor) and classify each cell into one of the 6 cube colors.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CUBE_COL_WHITE = 0,
    CUBE_COL_RED,
    CUBE_COL_ORANGE,
    CUBE_COL_YELLOW,
    CUBE_COL_GREEN,
    CUBE_COL_BLUE,
    CUBE_COL_COUNT
} cube_color_t;

typedef struct {
    int cx[9];      /* cell-center X in camera-frame coordinates */
    int cy[9];      /* cell-center Y */
    int box;        /* sample box side length in pixels (e.g. 12) */
} cube_scan_grid_t;

/* rgb565: byte-swapped (big-endian) RGB565 as delivered by esp32-camera.
 * out_colors[9]: cube_color_t per cell, row-major. */
void cube_scan_cells(const uint8_t *rgb565, int w, int h,
                     const cube_scan_grid_t *grid, uint8_t out_colors[9]);

#ifdef __cplusplus
}
#endif

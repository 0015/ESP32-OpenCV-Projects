/*
 * cv_cube — HSV color sampling for cube face scanning.
 *
 * Pipeline (per grid cell):
 *   1) extract the box x box pixel patch around the cell center from the
 *      RGB565 frame (byte-swapped, as delivered by esp32-camera)
 *   2) cvtColor RGB -> HSV  (H 0..179, S/V 0..255)
 *   3) cv::mean over the patch
 *   4) nearest of the 6 cube colors by (S,V) whiteness test + hue bands
 *
 * HSV separates hue from brightness, so classification is far more robust
 * against lighting changes than raw RGB distances. Red/orange remain the
 * hardest pair — thresholds below are tunable, and the UI allows manual
 * per-cell correction as a fallback.
 */
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "cube_vision.h"

/* ---- tunable classification thresholds -------------------------------------*/
#define WHITE_SAT_MAX     65    /* mean S below this = white                  */
#define WHITE_VAL_MIN     90    /* ...as long as it is reasonably bright      */
#define HUE_RED_MAX        7    /* H <= this            -> red                */
#define HUE_ORANGE_MAX    20    /* red_max  < H <= this -> orange             */
#define HUE_YELLOW_MAX    38    /* orange_max < H <= .. -> yellow             */
#define HUE_GREEN_MAX     88    /*                      -> green              */
#define HUE_BLUE_MAX     135    /*                      -> blue               */
#define HUE_RED_WRAP     150    /* H >= this (magenta..red wrap) -> red       */

static uint8_t classify_hsv(double h, double s, double v)
{
    if (s < WHITE_SAT_MAX && v > WHITE_VAL_MIN) return CUBE_COL_WHITE;
    if (s < WHITE_SAT_MAX / 2) return CUBE_COL_WHITE;   /* dim but colorless   */

    if (h <= HUE_RED_MAX)    return CUBE_COL_RED;
    if (h <= HUE_ORANGE_MAX) return CUBE_COL_ORANGE;
    if (h <= HUE_YELLOW_MAX) return CUBE_COL_YELLOW;
    if (h <= HUE_GREEN_MAX)  return CUBE_COL_GREEN;
    if (h <= HUE_BLUE_MAX)   return CUBE_COL_BLUE;
    if (h >= HUE_RED_WRAP)   return CUBE_COL_RED;
    return CUBE_COL_RED;                                /* 136..149: magenta-ish */
}

extern "C" void cube_scan_cells(const uint8_t *rgb565, int w, int h,
                                const cube_scan_grid_t *grid, uint8_t out_colors[9])
{
    const uint16_t *src = (const uint16_t *)rgb565;
    const int box = grid->box;

    cv::Mat patch(box, box, CV_8UC3);

    for (int cell = 0; cell < 9; cell++) {
        int x0 = grid->cx[cell] - box / 2;
        int y0 = grid->cy[cell] - box / 2;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x0 > w - box) x0 = w - box;
        if (y0 > h - box) y0 = h - box;

        for (int y = 0; y < box; y++) {
            uint8_t *row = patch.ptr<uint8_t>(y);
            const uint16_t *s = src + (size_t)(y0 + y) * w + x0;
            for (int x = 0; x < box; x++) {
                uint16_t p = __builtin_bswap16(s[x]);   /* camera data is BE   */
                row[x * 3 + 0] = (uint8_t)(((p >> 11) & 0x1F) << 3);   /* R */
                row[x * 3 + 1] = (uint8_t)(((p >> 5)  & 0x3F) << 2);   /* G */
                row[x * 3 + 2] = (uint8_t)(( p        & 0x1F) << 3);   /* B */
            }
        }

        cv::Mat hsv;
        cv::cvtColor(patch, hsv, cv::COLOR_RGB2HSV);
        cv::Scalar m = cv::mean(hsv);
        out_colors[cell] = classify_hsv(m[0], m[1], m[2]);
    }
}

/*
 * Dice pip detection — direct detection pipeline
 *
 *   1) RGB565 -> Gray
 *   2) GaussianBlur(3x3)
 *   3) Otsu threshold + invert (THRESH_BINARY_INV) -> dark pips become white
 *   4) morphOpen(3x3) -> remove noise
 *   5) connectedComponentsWithStats -> per-blob statistics
 *   6) filter: area[min,max] + squareness (aspect) + fill ratio (extent)
 *   7) union-find distance clustering -> cluster = die, size = pip count
 *
 * Works robustly under uneven lighting because only "small dark circles = pips" survive.
 * Otsu computes the threshold automatically from the histogram, so it handles
 * exposure and lighting changes well.
 *
 * debug_step (cycles on screen touch):
 *   0 = no visualization (detection only)
 *   1 = Gray
 *   2 = Otsu binarization
 *   3 = after morphology cleanup
 *   4 = result overlay (accepted=red, rejected=blue, die bbox=green)
 * Visualization is skipped when debug_out is NULL or debug_step==0.
 *
 * DICE_DEBUG 1 = per-frame summary log | 2 = all blob stats (for area tuning)
 */
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string.h>
#include <stdint.h>
#include <algorithm>
#include <numeric>
#include <vector>
#include <cmath>
#include "esp_log.h"

#define DICE_DEBUG 1

// ---- Tuning parameters (for 480x320; adjust to match real-device framing) ---
#define PIP_AREA_MIN      12       // minimum pip pixel count
#define PIP_AREA_MAX_FRAC 0.02f    // maximum pip area = 2% of frame (removes shadows/background)
#define PIP_MIN_ASPECT    0.55f    // short/long side ratio (circle=1, removes elongated corners)
#define PIP_MIN_EXTENT    0.55f    // area/bbox area (circle≈0.785, removes crescents)
#define PIP_CLUSTER_FACT  2.8f     // cluster distance = pip diameter * this factor
#define PIP_MAX_TOTAL     12       // max pips for 2 dice = 6+6

static const char *TAG = "DICE";

// total_eyes: sum of all detected pip counts across all dice (no per-die distinction)
extern "C" void dice_detect(uint8_t* data, int width, int height,
                            int* total_eyes,
                            int debug_step, uint8_t* debug_out);

// ---- RGB565 -> 8-bit grayscale --------------------------------------------
// esp32-camera stores RGB565 with byte-swapped (big-endian) order.
// (Live video looks correct because the buffer is sent directly to the panel.)
// So bytes must be swapped back when reading as 16-bit to get correct R/G/B bits.
static cv::Mat rgb565_to_gray(const uint8_t* data, int width, int height)
{
    cv::Mat gray(height, width, CV_8UC1);
    const uint16_t* src = (const uint16_t*)data;
    for (int i = 0; i < width * height; i++) {
        uint16_t p = __builtin_bswap16(src[i]);   // byte swap
        int r = ((p >> 11) & 0x1F) << 3;
        int g = ((p >>  5) & 0x3F) << 2;
        int b = ( p        & 0x1F) << 3;
        gray.data[i] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
    }
    return gray;
}

// ---- Pack 8-bit value -> RGB565 (with same byte swap as the camera) ---------
static inline uint16_t pack565(int r, int g, int b)
{
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static void gray_to_565(const cv::Mat& gray, uint8_t* out)
{
    uint16_t* d = (uint16_t*)out;
    int n = gray.rows * gray.cols;
    for (int i = 0; i < n; i++) {
        int g = gray.data[i];
        d[i] = __builtin_bswap16(pack565(g, g, g));   // byte swap for panel
    }
}

static void bgr_to_565(const cv::Mat& bgr, uint8_t* out)
{
    uint16_t* d = (uint16_t*)out;
    int n = bgr.rows * bgr.cols;
    for (int i = 0; i < n; i++) {
        const uint8_t* p = bgr.data + i * 3;     // BGR
        d[i] = __builtin_bswap16(pack565(p[2], p[1], p[0]));
    }
}

// ---- union-find ----------------------------------------------------------
static int uf_find(std::vector<int>& parent, int a)
{
    while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
    return a;
}

struct Pip { float x, y; float area; };
struct Die { std::vector<int> members; float cx; };   // members = pip indices

void dice_detect(uint8_t* data, int width, int height,
                 int* total_eyes,
                 int debug_step, uint8_t* debug_out)
{
    *total_eyes = 0;
    const bool draw = (debug_step > 0 && debug_out != NULL);

    // 1) Gray
    cv::Mat gray = rgb565_to_gray(data, width, height);

    // 2) Blur
    cv::Mat blur;
    cv::GaussianBlur(gray, blur, cv::Size(3, 3), 0);

    // 3) Otsu + invert
    cv::Mat bin;
    cv::threshold(blur, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    if (draw && debug_step == 1) { gray_to_565(gray, debug_out); }
    if (draw && debug_step == 2) { gray_to_565(bin,  debug_out); }

    // 4) morphOpen
    cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, k);

    if (draw && debug_step == 3) { gray_to_565(bin, debug_out); }

    // 5) connected components labeling
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(bin, labels, stats, centroids, 8);

    // 6) area + shape filter
    const float max_area = PIP_AREA_MAX_FRAC * (float)(width * height);
    std::vector<Pip> pips;
    // BGR overlay image (only allocated when needed)
    cv::Mat vis;
    if (draw && debug_step == 4) cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);

    for (int i = 1; i < n; i++) {        // 0 = background
        float area = (float)stats.at<int>(i, cv::CC_STAT_AREA);
        int   w    = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int   h    = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        float aspect = (std::max(w, h) > 0)
                       ? (float)std::min(w, h) / (float)std::max(w, h) : 0.f;
        float extent = (w * h > 0) ? area / (float)(w * h) : 0.f;
        float cx = (float)centroids.at<double>(i, 0);
        float cy = (float)centroids.at<double>(i, 1);

        bool pass = (area >= PIP_AREA_MIN && area <= max_area &&
                     aspect >= PIP_MIN_ASPECT && extent >= PIP_MIN_EXTENT);

#if DICE_DEBUG >= 2
        if (area >= PIP_AREA_MIN)
            ESP_LOGI(TAG, "  blob area=%.0f asp=%.2f ext=%.2f [%s]",
                     area, aspect, extent, pass ? "PASS" : "fail");
#endif

        if (draw && debug_step == 4) {
            // Rejected blobs: skip tiny noise, draw rest in blue
            if (!pass && area >= PIP_AREA_MIN)
                cv::circle(vis, cv::Point((int)cx, (int)cy),
                           std::max(3, (w + h) / 4), cv::Scalar(255, 0, 0), 1);
        }
        if (pass) pips.push_back({cx, cy, area});
    }

    // 7) distance clustering (union-find)
    std::vector<Die> dice;
    if (!pips.empty() && (int)pips.size() <= PIP_MAX_TOTAL) {
        // cluster distance = pip diameter * PIP_CLUSTER_FACT (based on median area)
        std::vector<float> areas;
        areas.reserve(pips.size());
        for (auto& p : pips) areas.push_back(p.area);
        std::sort(areas.begin(), areas.end());
        float med = areas[areas.size() / 2];
        float diameter = std::sqrt(med / 3.14159265f) * 2.0f;
        float cdist = diameter * PIP_CLUSTER_FACT;
        float cd2 = cdist * cdist;

        int np = (int)pips.size();
        std::vector<int> parent(np);
        std::iota(parent.begin(), parent.end(), 0);
        for (int i = 0; i < np; i++)
            for (int j = i + 1; j < np; j++) {
                float dx = pips[i].x - pips[j].x;
                float dy = pips[i].y - pips[j].y;
                if (dx * dx + dy * dy <= cd2) {
                    int ra = uf_find(parent, i), rb = uf_find(parent, j);
                    if (ra != rb) parent[ra] = rb;
                }
            }

        // Accumulate per-cluster members (keep pip indices)
        std::vector<int> root_of;
        for (int i = 0; i < np; i++) {
            int r = uf_find(parent, i);
            int idx = -1;
            for (int t = 0; t < (int)root_of.size(); t++)
                if (root_of[t] == r) { idx = t; break; }
            if (idx < 0) { root_of.push_back(r); dice.push_back({}); idx = (int)dice.size() - 1; }
            dice[idx].members.push_back(i);
        }
        for (auto& d : dice) {
            float sx = 0.f;
            for (int m : d.members) sx += pips[m].x;
            d.cx = sx / d.members.size();
        }

        // Only clusters with 1-6 pips are valid (others treated as false detections)
        dice.erase(std::remove_if(dice.begin(), dice.end(),
                   [](const Die& d){ return d.members.size() < 1 || d.members.size() > 6; }),
                   dice.end());
    }

    // Total pip count = sum of all valid cluster sizes
    int total = 0;
    for (auto& d : dice) total += (int)d.members.size();
    *total_eyes = total;

#if DICE_DEBUG
    ESP_LOGI(TAG, "pips=%d  dice=%d  total_eyes=%d",
             (int)pips.size(), (int)dice.size(), total);
#endif

    // Draw result overlay
    if (draw && debug_step == 4) {
        // Accepted pips = red circles
        for (auto& p : pips)
            cv::circle(vis, cv::Point((int)p.x, (int)p.y),
                       std::max(4, (int)(std::sqrt(p.area / 3.14159265f) * 1.4f)),
                       cv::Scalar(0, 0, 255), 2);
        // Die bboxes = green rectangles (bounding box of cluster pips + padding)
        for (auto& d : dice) {
            float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
            for (int m : d.members) {
                x0 = std::min(x0, pips[m].x); y0 = std::min(y0, pips[m].y);
                x1 = std::max(x1, pips[m].x); y1 = std::max(y1, pips[m].y);
            }
            int pad = 14;
            cv::rectangle(vis,
                cv::Point((int)x0 - pad, (int)y0 - pad),
                cv::Point((int)x1 + pad, (int)y1 + pad),
                cv::Scalar(0, 255, 0), 2);
        }
        bgr_to_565(vis, debug_out);
    }
}

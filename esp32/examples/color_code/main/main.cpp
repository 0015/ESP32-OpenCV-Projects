/////////////////////////////////////////////////////////////////
/*
  RGB Pixel Detector & Histogram (Running OpenCV on ESP32)
  For More Information:
  Created by Eric N. (ThatProject)
*/
/////////////////////////////////////////////////////////////////

#undef EPS // specreg.h defines EPS which interfere with opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192

#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <iostream>
#include <map>

#include "system.h"
#include "app_screen.h"
#include "app_camera.h"

#include "iot_lvgl.h"

using namespace cv;

extern "C"
{
    void app_main(void);
}

#define TAG "main"

extern CEspLcd *tft;

static lv_obj_t *lvCameraImage; // Camera image object

static lv_obj_t *lv_container_RGB;
static lv_obj_t *lv_bar_red;
static lv_obj_t *lv_label_red;
static lv_obj_t *lv_bar_blue;
static lv_obj_t *lv_label_blue;
static lv_obj_t *lv_bar_green;
static lv_obj_t *lv_label_green;

const int histSize = 240;
const int hist_w = 240;
const int hist_h = 240;

void gui_screen()
{
    static lv_style_t style;
    lv_style_init(&style);

    lv_style_set_radius(&style, LV_STATE_DEFAULT, 2);
    lv_style_set_bg_opa(&style, LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_style_set_bg_color(&style, LV_STATE_DEFAULT, LV_COLOR_MAKE(190, 190, 190));
    lv_style_set_border_width(&style, LV_STATE_DEFAULT, 2);
    lv_style_set_border_color(&style, LV_STATE_DEFAULT, LV_COLOR_MAKE(142, 142, 142));

    lv_style_set_pad_top(&style, LV_STATE_DEFAULT, 60);
    lv_style_set_pad_bottom(&style, LV_STATE_DEFAULT, 60);
    lv_style_set_pad_left(&style, LV_STATE_DEFAULT, 60);
    lv_style_set_pad_right(&style, LV_STATE_DEFAULT, 60);

    lv_style_set_text_color(&style, LV_STATE_DEFAULT, LV_COLOR_MAKE(102, 102, 102));
    lv_style_set_text_letter_space(&style, LV_STATE_DEFAULT, 5);
    lv_style_set_text_line_space(&style, LV_STATE_DEFAULT, 20);

    /*Create an object with the new style*/
    lv_obj_t *obj = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(obj, LV_LABEL_PART_MAIN, &style);
    lv_label_set_text(obj, "Color Code\n"
                           "Histogram");
    lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 0, 0);
    wait_msec(2000);

    static lv_style_t style_container;
    lv_style_init(&style_container);
    lv_style_set_bg_color(&style_container, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_style_set_bg_opa(&style_container, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_style_set_border_width(&style_container, LV_STATE_DEFAULT, 0);
    lv_container_RGB = lv_cont_create(lv_scr_act(), NULL);
    lv_obj_add_style(lv_container_RGB, LV_CONT_PART_MAIN, &style_container);
    lv_obj_align(lv_container_RGB, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_obj_set_size(lv_container_RGB, 240, 46);
    static lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_opa(&label_style, LV_STATE_DEFAULT, LV_OPA_COVER);

    lv_label_red = lv_label_create(lv_container_RGB, NULL);
    lv_obj_add_style(lv_label_red, LV_LABEL_PART_MAIN, &label_style);
    lv_label_set_recolor(lv_label_red, true);
    lv_label_set_text(lv_label_red, "#ff0000 R: 128");
    lv_obj_set_size(lv_label_red, 40, 10);
    lv_obj_align(lv_label_red, NULL, LV_ALIGN_IN_TOP_LEFT, 2, 2);

    lv_label_green = lv_label_create(lv_container_RGB, NULL);
    lv_obj_add_style(lv_label_green, LV_LABEL_PART_MAIN, &label_style);
    lv_label_set_recolor(lv_label_green, true);
    lv_label_set_text(lv_label_green, "#00ff00 G: 128");
    lv_obj_set_size(lv_label_green, 40, 10);
    lv_obj_align(lv_label_green, NULL, LV_ALIGN_IN_TOP_LEFT, 2, 16);

    lv_label_blue = lv_label_create(lv_container_RGB, NULL);
    lv_obj_add_style(lv_label_blue, LV_LABEL_PART_MAIN, &label_style);
    lv_label_set_recolor(lv_label_blue, true);
    lv_label_set_text(lv_label_blue, "#0000ff B: 128");
    lv_obj_set_size(lv_label_blue, 40, 10);
    lv_obj_align(lv_label_blue, NULL, LV_ALIGN_IN_TOP_LEFT, 2, 30);

    static lv_style_t style_bar;
    lv_style_init(&style_bar);
    lv_style_set_radius(&style_bar, LV_STATE_DEFAULT, 4);
    lv_style_set_bg_color(&style_bar, LV_STATE_DEFAULT, LV_COLOR_RED);

    lv_bar_red = lv_bar_create(lv_container_RGB, NULL);
    lv_obj_add_style(lv_bar_red, LV_BAR_PART_INDIC, &style_bar);
    lv_obj_set_size(lv_bar_red, 160, 8);
    lv_obj_align(lv_bar_red, NULL, LV_ALIGN_IN_TOP_LEFT, 60, 6);
    lv_bar_set_anim_time(lv_bar_red, 100);
    lv_bar_set_value(lv_bar_red, 100, LV_ANIM_ON);
    lv_bar_set_range(lv_bar_red, 0, 255);
    lv_obj_set_style_local_bg_opa(lv_bar_red, LV_BAR_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);

    lv_bar_green = lv_bar_create(lv_container_RGB, NULL);
    lv_obj_add_style(lv_bar_green, LV_BAR_PART_INDIC, &style_bar);
    lv_obj_set_size(lv_bar_green, 160, 8);
    lv_obj_align(lv_bar_green, NULL, LV_ALIGN_IN_TOP_LEFT, 60, 20);
    lv_bar_set_anim_time(lv_bar_green, 100);
    lv_bar_set_value(lv_bar_green, 100, LV_ANIM_ON);
    lv_bar_set_range(lv_bar_green, 0, 255);
    lv_obj_set_style_local_bg_color(lv_bar_green, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, LV_COLOR_GREEN);
    lv_obj_set_style_local_bg_opa(lv_bar_green, LV_BAR_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);

    lv_bar_blue = lv_bar_create(lv_container_RGB, NULL);
    lv_obj_add_style(lv_bar_blue, LV_BAR_PART_INDIC, &style_bar);
    lv_obj_set_size(lv_bar_blue, 160, 8);
    lv_obj_align(lv_bar_blue, NULL, LV_ALIGN_IN_TOP_LEFT, 60, 34);
    lv_bar_set_anim_time(lv_bar_blue, 100);
    lv_bar_set_value(lv_bar_blue, 100, LV_ANIM_ON);
    lv_bar_set_range(lv_bar_blue, 0, 255);
    lv_obj_set_style_local_bg_color(lv_bar_blue, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_set_style_local_bg_opa(lv_bar_blue, LV_BAR_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
}

esp_err_t updateCameraImage(const Mat &img)
{
    // static variables because they must still be available when lv_task_handler() is called
    static Mat imgCopy;
    static lv_img_dsc_t my_img_dsc;

    if (img.empty())
    {
        ESP_LOGW(TAG, "Can't display empty image");
        return ESP_ERR_INVALID_ARG;
    }

    // convert image to bgr565 if needed
    if (img.type() == CV_8UC1)
    { // grayscale image
        cvtColor(img, imgCopy, COLOR_GRAY2BGR565, 1);
    }
    else if (img.type() == CV_8UC3)
    { // BGR888 image
        cvtColor(img, imgCopy, COLOR_BGR2BGR565, 1);
    }
    else if (img.type() == CV_8UC2)
    { // BGR565 image
        img.copyTo(imgCopy);
    }

    my_img_dsc.header.always_zero = 0;
    my_img_dsc.header.w = imgCopy.cols;
    my_img_dsc.header.h = imgCopy.rows;
    my_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    my_img_dsc.data_size = imgCopy.size().width * imgCopy.size().height;
    my_img_dsc.data = imgCopy.ptr<uchar>(0);

    lv_img_set_src(lvCameraImage, &my_img_dsc); /* Set the created file as image */
    lv_obj_set_pos(lvCameraImage, 0, 0);

    return ESP_OK;
}

void updateColorCode(int red, int green, int blue)
{

    lv_obj_move_foreground(lv_container_RGB);

    lv_bar_set_value(lv_bar_red, red, LV_ANIM_ON);
    lv_bar_set_value(lv_bar_green, green, LV_ANIM_ON);
    lv_bar_set_value(lv_bar_blue, blue, LV_ANIM_ON);

    std::string str = "#ff0000 R: " + std::to_string(red);
    lv_label_set_text(lv_label_red, str.c_str());

    str = "#00ff00 G: " + std::to_string(green);
    lv_label_set_text(lv_label_green, str.c_str());

    str = "#0000ff B: " + std::to_string(blue);
    lv_label_set_text(lv_label_blue, str.c_str());

    ESP_LOGI("COLOR", "R: %d,G: %d, B: %d", red, green, blue);
}

void drawCenterMark(Mat &src)
{
    Point pc1(120, 100), pc2(120, 140);
    line(src, pc1, pc2, Scalar(255, 255, 255), 2, LINE_8);

    Point pc3(100, 120), pc4(140, 120);
    line(src, pc3, pc4, Scalar(255, 255, 255), 2, LINE_8);
}

void drawHistogram(Mat &b_hist, Mat &g_hist, Mat &r_hist, Mat &src)
{

    int bin_w = cvRound((double)hist_w / histSize);

    normalize(b_hist, b_hist, 0, src.rows, NORM_MINMAX, -1,
              Mat());
    normalize(g_hist, g_hist, 0, src.rows, NORM_MINMAX, -1,
              Mat());
    normalize(r_hist, r_hist, 0, src.rows, NORM_MINMAX, -1,
              Mat());

    for (int i = 1; i < histSize; i++)
    {
        line(
            src,
            Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
            Scalar(255, 0, 0), 1, LINE_AA);
        line(
            src,
            Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
            Scalar(0, 255, 0), 1, LINE_AA);

        line(
            src,
            Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
            Scalar(0, 0, 255), 1, LINE_AA);
    }
}

void find_color(void *arg)
{
    ESP_LOGI(TAG, "Starting find_color");

    sensor_t *s = esp_camera_sensor_get();

    lvCameraImage = lv_img_create(lv_disp_get_scr_act(nullptr), nullptr);
    lv_obj_move_foreground(lvCameraImage);

    while (true)
    {
        auto start = esp_timer_get_time();

        camera_fb_t *fb = esp_camera_fb_get();

        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
        }
        else
        {
            if (s->pixformat == PIXFORMAT_JPEG)
            {
                TFT_jpg_image(CENTER, CENTER, 0, -1, NULL, fb->buf, fb->len);
                esp_camera_fb_return(fb);
                fb = NULL;
            }
            else
            {                                                            // RGB565 pixformat
                Mat inputImage(fb->height, fb->width, CV_8UC2, fb->buf); // rgb565 is 2 channels of 8-bit unsigned
                cvtColor(inputImage, inputImage, COLOR_BGR5652BGR);

                int pos_x = fb->width / 2;
                int pos_y = fb->height / 2;
                int blue = inputImage.at<Vec3b>(pos_x, pos_y)[0];  // getting the pixel values//
                int green = inputImage.at<Vec3b>(pos_x, pos_y)[1]; // getting the pixel values//
                int red = inputImage.at<Vec3b>(pos_x, pos_y)[2];   // getting the pixel values//

                updateColorCode(red, green, blue);

                std::vector<Mat> bgr_planes;
                split(inputImage, bgr_planes);

                float range[] = {0, 240};
                const float *histRange = {range};

                bool uniform = true;
                bool accumulate = false;

                Mat b_hist, g_hist, r_hist;

                calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize,
                         &histRange, uniform, accumulate);
                calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize,
                         &histRange, uniform, accumulate);
                calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize,
                         &histRange, uniform, accumulate);

                drawHistogram(b_hist, g_hist, r_hist, inputImage);
                drawCenterMark(inputImage);
                updateCameraImage(inputImage);
            }
        }

        ESP_LOGI(TAG, "Around %f fps", 1.0f / ((esp_timer_get_time() - start) / 1000000.0f));
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Starting main");

    app_camera_init();

    lvgl_init();

    gui_screen();

    xTaskCreatePinnedToCore(find_color, "find_color", 1024 * 9, nullptr, 24, nullptr, 0);
}
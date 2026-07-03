#include <stdio.h>
#include "esp_check.h"
#include "esp_log.h"

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lvgl_port.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include <string.h>

#define PWDN_GPIO_NUM 17  // power down is not used
#define RESET_GPIO_NUM -1 // software reset will be performed
#define XCLK_GPIO_NUM 8
#define SIOD_GPIO_NUM 21
#define SIOC_GPIO_NUM 16

#define Y9_GPIO_NUM 2
#define Y8_GPIO_NUM 7
#define Y7_GPIO_NUM 10
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 11
#define Y4_GPIO_NUM 15
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 12
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 4
#define PCLK_GPIO_NUM 9

#define EXAMPLE_PIN_NUM_SCLK 39
#define EXAMPLE_PIN_NUM_MOSI 38
#define EXAMPLE_PIN_NUM_MISO 40

#define EXAMPLE_SPI_HOST SPI2_HOST

#define EXAMPLE_I2C_NUM 0 // I2C number
#define EXAMPLE_PIN_NUM_I2C_SDA 48
#define EXAMPLE_PIN_NUM_I2C_SCL 47

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)

#define EXAMPLE_PIN_NUM_LCD_DC 42
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS 45

#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320

// ============================================================================
// Display orientation (LVGL software rotation)
//   0 = Portrait  (240 x 320)
//   1 = Landscape (320 x 240)
// The panel always runs at native 240x320 (same as when portrait mode worked).
// LVGL rotates the frame and touch events. -> Avoids screen corruption from HW swap_xy.
// If the landscape image is upside-down or mirrored, swap ROTATION_90 <-> ROTATION_270.
// If touch is misaligned, toggle TOUCH_MIRROR_X/Y below.
// ============================================================================
#define EXAMPLE_DISPLAY_LANDSCAPE 1

#if EXAMPLE_DISPLAY_LANDSCAPE
  #define EXAMPLE_DISP_ROTATION  LV_DISPLAY_ROTATION_90
  #define EXAMPLE_TOUCH_X_MAX    EXAMPLE_LCD_V_RES   // 320 (landscape width)
  #define EXAMPLE_TOUCH_Y_MAX    EXAMPLE_LCD_H_RES   // 240 (landscape height)
  #define EXAMPLE_TOUCH_SWAP_XY  1
  #define EXAMPLE_TOUCH_MIRROR_X 0
  #define EXAMPLE_TOUCH_MIRROR_Y 1
#else
  #define EXAMPLE_DISP_ROTATION  LV_DISPLAY_ROTATION_0
  #define EXAMPLE_TOUCH_X_MAX    EXAMPLE_LCD_H_RES   // 240
  #define EXAMPLE_TOUCH_Y_MAX    EXAMPLE_LCD_V_RES   // 320
  #define EXAMPLE_TOUCH_SWAP_XY  0
  #define EXAMPLE_TOUCH_MIRROR_X 0
  #define EXAMPLE_TOUCH_MIRROR_Y 0
#endif

// In landscape mode the raw 480x320 camera frame appears rotated 90 degrees.
// Rotate the frame 90 degrees before display to correct orientation.
//   EXAMPLE_CAMERA_ROTATE_CW : 1 = clockwise, 0 = counter-clockwise.
//   If the image appears upside-down (180°), toggle this value.
#define EXAMPLE_CAMERA_ROTATE     EXAMPLE_DISPLAY_LANDSCAPE
#define EXAMPLE_CAMERA_ROTATE_CW  1

#define EXAMPLE_PIN_NUM_BK_LIGHT 1

#define LCD_BL_LEDC_TIMER LEDC_TIMER_0
#define LCD_BL_LEDC_MODE LEDC_LOW_SPEED_MODE

#define LCD_BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_BL_LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LCD_BL_LEDC_DUTY (1024)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LCD_BL_LEDC_FREQUENCY (10000)          // Frequency in Hertz. Set frequency at 5 kHz

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1

static const char *TAG = "lcd_2_base";

esp_lcd_panel_io_handle_t io_handle;
esp_lcd_panel_handle_t panel_handle;
esp_lcd_touch_handle_t tp;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

lv_obj_t *img_camera;
static lv_obj_t *dice_label = NULL;
static int        g_last_eyes = -2;   // -2 = uninitialized, -1 = "no dice"

// Shared frame buffer: camera_task copies frame here, detect_task reads it
static uint8_t           *detect_buf = NULL;
static SemaphoreHandle_t  detect_sem = NULL;   // binary: signals "new frame ready"

// Rotation buffer for landscape display (rotated+cropped output, 320x240 RGB565)
static uint8_t           *rot_buf = NULL;

void dice_detect(uint8_t* data, int width, int height, int* total_eyes,
                 int debug_step, uint8_t* debug_out);

// Opacity animation callback (used by dice_label fade-in effect)
static void dice_opa_anim_cb(void *obj, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}

// ---- Debug step view (cycles on screen touch) ------------------------------
//  0 = live camera | 1 = Gray | 2 = Otsu | 3 = Morph | 4 = result overlay
#define DEBUG_STEP_COUNT 5
static volatile int g_debug_step = 0;
static const char *g_step_names[DEBUG_STEP_COUNT] = {
    "LIVE", "1:Gray", "2:Otsu", "3:Morph", "4:Result"
};
static uint8_t        *debug_buf  = NULL;   // 480*320*2, detection stage visualization output
static lv_obj_t       *step_label = NULL;   // top-center: current step name


void camera_init(void)
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_1;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_HVGA;
    config.pixel_format = PIXFORMAT_RGB565; // for streaming
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        printf("Camera init failed with error 0x%x", err);
        vTaskDelete(NULL);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_hmirror(s, 1);
}

// Runs on core 1: waits for a frame copy, runs OpenCV, updates label
static void detect_task(void *param)
{
    while (1)
    {
        // Block until camera_task signals a new frame is ready
        if (xSemaphoreTake(detect_sem, portMAX_DELAY) != pdTRUE) continue;

        int total = 0;
        int step = g_debug_step;
        dice_detect(detect_buf, 480, 320, &total, step, debug_buf);

        char buf[16];
        int  key = (total > 0) ? total : -1;
        if (total > 0) snprintf(buf, sizeof(buf), "%d", total);
        else           snprintf(buf, sizeof(buf), "-");

        // Color + glow coding: gray → cyan → green → yellow → orange
        lv_color_t text_col, shadow_col;
        if      (total <= 0)  { text_col = lv_color_make(160,160,160); shadow_col = lv_color_make( 40, 40, 40); }
        else if (total <= 4)  { text_col = lv_color_make( 80,220,255); shadow_col = lv_color_make(  0,140,200); }
        else if (total <= 8)  { text_col = lv_color_make(100,255,120); shadow_col = lv_color_make(  0,180, 60); }
        else if (total <= 12) { text_col = lv_color_make(255,220, 60); shadow_col = lv_color_make(200,140,  0); }
        else                  { text_col = lv_color_make(255,100, 50); shadow_col = lv_color_make(200, 50,  0); }

        lvgl_port_lock(0);
        lv_label_set_text(dice_label, buf);
        lv_obj_set_style_text_color(dice_label, text_col, 0);
        lv_obj_set_style_shadow_color(dice_label, shadow_col, 0);

        // Fade-in pop animation only when value changes
        if (key != g_last_eyes) {
            g_last_eyes = key;
            lv_obj_set_style_opa(dice_label, LV_OPA_TRANSP, 0);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, dice_label);
            lv_anim_set_exec_cb(&a, dice_opa_anim_cb);
            lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);
            lv_anim_set_time(&a, 350);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_start(&a);
        }
        lvgl_port_unlock();
    }
    vTaskDelete(NULL);
}

// Rotate the camera frame (480x320 RGB565) 90 degrees while producing only the
// visible center 320x240 region directly.
//  - Half the pixels vs. building a full 320x480 then cropping.
//  - Tile-based processing reuses PSRAM cache lines -> reduces strided-access cost.
//  - dst is written sequentially row by row.
// cw=1 clockwise, cw=0 counter-clockwise (toggle if image appears 180° flipped).
#define CAM_SRC_W   480
#define CAM_DST_W   320   // = EXAMPLE_LCD_V_RES
#define CAM_DST_H   240   // = EXAMPLE_LCD_H_RES
#define ROT_TILE    16    // 16px * 2B = 32B = cache line
static void rotate_crop_rgb565(const uint8_t *src8, uint8_t *dst8, int cw)
{
    const uint16_t *src = (const uint16_t *)src8;
    uint16_t       *dst = (uint16_t *)dst8;

    for (int oy0 = 0; oy0 < CAM_DST_H; oy0 += ROT_TILE)
    {
        for (int ox0 = 0; ox0 < CAM_DST_W; ox0 += ROT_TILE)
        {
            for (int oy = oy0; oy < oy0 + ROT_TILE; oy++)
            {
                uint16_t *drow = dst + (size_t)oy * CAM_DST_W;
                if (cw)
                {
                    const int sc = 120 + oy;                 // fixed src column
                    for (int ox = ox0; ox < ox0 + ROT_TILE; ox++)
                        drow[ox] = src[(size_t)(319 - ox) * CAM_SRC_W + sc];
                }
                else
                {
                    const int sc = 359 - oy;                 // fixed src column
                    for (int ox = ox0; ox < ox0 + ROT_TILE; ox++)
                        drow[ox] = src[(size_t)ox * CAM_SRC_W + sc];
                }
            }
        }
    }
}

// Runs on core 0: captures frames and updates the display without blocking
static void camera_task(void *param)
{
    camera_fb_t *pic;
    lv_img_dsc_t img_dsc;

    img_dsc.data      = NULL;
    // OV camera RGB565 output is byte-swapped (big-endian) relative to LVGL native.
    // Declare as SWAPPED so LVGL converts it correctly into the native framebuffer.
    // (Paired with display flags.swap_bytes=true to also normalize label/object colors.)
    img_dsc.header.cf = LV_COLOR_FORMAT_RGB565_SWAPPED;
#if EXAMPLE_CAMERA_ROTATE
    // Rotation + center crop produces exactly the display size: 320x240
    img_dsc.data_size = 320 * 240 * 2;
    img_dsc.header.w  = 320;
    img_dsc.header.h  = 240;
#else
    img_dsc.data_size = 480 * 320 * 2;
    img_dsc.header.w  = 480;
    img_dsc.header.h  = 320;
#endif

    TickType_t last_trigger = 0;

    while (1)
    {
        pic = esp_camera_fb_get();

        if (pic != NULL)
        {
            // Every 500 ms: copy frame for detection (non-blocking)
            TickType_t now = xTaskGetTickCount();
            if ((now - last_trigger) >= pdMS_TO_TICKS(500) && detect_buf != NULL)
            {
                last_trigger = now;
                memcpy(detect_buf, pic->buf, (size_t)pic->width * pic->height * 2);
                xSemaphoreGive(detect_sem);  // binary: safe to call even if already "1"
            }

            // Display update — always runs, never blocked by OpenCV.
            // Show detection stage visualization (debug_buf) when a debug step is active,
            // otherwise show the live camera feed (pic->buf).
            uint8_t *src = (g_debug_step != 0 && debug_buf != NULL)
                           ? debug_buf : pic->buf;
#if EXAMPLE_CAMERA_ROTATE
            // Landscape mode: rotate only the visible region 90° directly into 320x240.
            if (rot_buf != NULL)
            {
                rotate_crop_rgb565(src, rot_buf, EXAMPLE_CAMERA_ROTATE_CW);
                img_dsc.data = rot_buf;
            }
#else
            img_dsc.data = src;
#endif
            lvgl_port_lock(0);
            lv_img_set_src(img_camera, &img_dsc);
            lvgl_port_unlock();
        }
        esp_camera_fb_return(pic);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}



void display_init(void)
{
    ESP_LOGI(TAG, "SPI BUS init");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Install panel IO");

    

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        //.on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_SPI_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ST7789 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
}

void touch_init(void)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    i2c_master_bus_handle_t i2c_handle;


    ESP_LOGI(TAG, "Initialize I2C");
    esp_err_t ret = bsp_i2c_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Initialize I2C Fail");
        abort();
    }

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_TOUCH_X_MAX,
        .y_max = EXAMPLE_TOUCH_Y_MAX,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = EXAMPLE_TOUCH_SWAP_XY,
            .mirror_x = EXAMPLE_TOUCH_MIRROR_X,
            .mirror_y = EXAMPLE_TOUCH_MIRROR_Y,
        },
    };



    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();


    i2c_handle = bsp_i2c_get_handle();
    esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle);
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);


    // ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    // esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    // tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2

    // ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_I2C_NUM, &tp_io_config, &tp_io_handle));

    // esp_lcd_touch_config_t tp_cfg = {
    //     .x_max = EXAMPLE_LCD_V_RES,
    //     .y_max = EXAMPLE_LCD_H_RES,
    //     .rst_gpio_num = -1,
    //     .int_gpio_num = -1,
    //     .flags = {
    //         .swap_xy = 0,
    //         .mirror_x = 0,
    //         .mirror_y = 0,
    //     },
    // };

    // ESP_LOGI(TAG, "Initialize touch controller CST816");
    // ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp));
}

void bsp_brightness_init(void)
{
    gpio_set_direction(EXAMPLE_PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, 1);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LCD_BL_LEDC_MODE,
        .timer_num = LCD_BL_LEDC_TIMER,
        .duty_resolution = LCD_BL_LEDC_DUTY_RES,
        .freq_hz = LCD_BL_LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LCD_BL_LEDC_MODE,
        .channel = LCD_BL_LEDC_CHANNEL,
        .timer_sel = LCD_BL_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = EXAMPLE_PIN_NUM_BK_LIGHT,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void bsp_brightness_set_level(uint8_t level)
{
    if (level > 100)
    {
        ESP_LOGE(TAG, "Brightness value out of range");
        return;
    }

    uint32_t duty = (level * (LCD_BL_LEDC_DUTY - 1)) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL));

    ESP_LOGI(TAG, "LCD brightness set to %d%%", level);
}




// Screen touch -> cycle debug step (LIVE -> Gray -> Otsu -> Morph -> Result -> LIVE)
static void cycle_debug_step(void)
{
    g_debug_step = (g_debug_step + 1) % DEBUG_STEP_COUNT;
    if (step_label) {
        lvgl_port_lock(0);
        lv_label_set_text(step_label, g_step_names[g_debug_step]);
        lvgl_port_unlock();
    }
    ESP_LOGI(TAG, "debug step -> %d (%s)", g_debug_step, g_step_names[g_debug_step]);
}

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        cycle_debug_step();
    }
}

void app_main_display(void)
{
    lv_obj_t *scr = lv_screen_active();

    /* Task lock */
    lvgl_port_lock(0);
    img_camera = lv_img_create(scr);
    lv_obj_set_size(img_camera, lv_pct(100), lv_pct(100));
    // Center-align image -> frames larger than the screen are center-cropped
    lv_image_set_inner_align(img_camera, LV_IMAGE_ALIGN_CENTER);

    // Dice result — large game-style number display, bottom-center
    dice_label = lv_label_create(scr);
    lv_label_set_text(dice_label, "-");
    lv_obj_set_style_text_font(dice_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(dice_label, lv_color_make(160, 160, 160), 0);
    lv_obj_set_style_bg_color(dice_label, lv_color_make(10, 10, 10), 0);
    lv_obj_set_style_bg_opa(dice_label, LV_OPA_70, 0);
    lv_obj_set_style_pad_top(dice_label, 6, 0);
    lv_obj_set_style_pad_bottom(dice_label, 6, 0);
    lv_obj_set_style_pad_left(dice_label, 20, 0);
    lv_obj_set_style_pad_right(dice_label, 20, 0);
    lv_obj_set_style_radius(dice_label, 14, 0);
    lv_obj_set_style_shadow_color(dice_label, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_shadow_opa(dice_label, LV_OPA_80, 0);
    lv_obj_set_style_shadow_width(dice_label, 30, 0);
    lv_obj_set_style_shadow_spread(dice_label, 4, 0);
    lv_obj_align(dice_label, LV_ALIGN_BOTTOM_MID, 0, -12);

    // Debug step name — top-center. Updated on screen touch.
    step_label = lv_label_create(scr);
    lv_label_set_text(step_label, g_step_names[0]);
    lv_obj_set_style_text_color(step_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(step_label, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(step_label, LV_OPA_60, 0);
    lv_obj_set_style_pad_all(step_label, 6, 0);
    lv_obj_set_style_radius(step_label, 4, 0);
    lv_obj_align(step_label, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t * fullscreen_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(fullscreen_btn, lv_pct(100), lv_pct(100));
    lv_obj_center(fullscreen_btn);
    lv_obj_set_style_bg_opa(fullscreen_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(fullscreen_btn, 0, 0);            // Remove border
    lv_obj_set_style_border_opa(fullscreen_btn, LV_OPA_TRANSP, 0);  // Ensure border is transparent
    lv_obj_set_style_outline_opa(fullscreen_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(fullscreen_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    
    lv_obj_add_event_cb(fullscreen_btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    /* Task unlock */
    lvgl_port_unlock();
}



void app_main(void)
{

    ESP_LOGI(TAG, "app_main");

    
    display_init();
    touch_init();


     /* Add LCD screen */
     ESP_LOGI(TAG, "Add LCD screen");
     const lvgl_port_display_cfg_t disp_cfg = {
         .io_handle = io_handle,
         .panel_handle = panel_handle,
         .buffer_size = EXAMPLE_LCD_V_RES * EXAMPLE_LCD_H_RES,
         .double_buffer = true,
         // Drive at native panel resolution (rotation handled by lv_display_set_rotation)
         .hres = EXAMPLE_LCD_H_RES,
         .vres = EXAMPLE_LCD_V_RES,
         .monochrome = false,
         .rotation = {
             .swap_xy = false,
             .mirror_x = false,
             .mirror_y = false,
         },
         .flags = {
             .buff_dma = false,
             .buff_spiram = false,
             .sw_rotate = true,
             // ST7789 expects big-endian RGB565. The LVGL native (little-endian)
             // framebuffer must be byte-swapped at flush time so that labels and
             // other LVGL objects render with correct colors. (Camera image is
             // declared as _SWAPPED to match.)
             .swap_bytes = true,
             .full_refresh = true,
             .direct_mode = false,
         }};
 
     const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
     ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));
     lvgl_disp = lvgl_port_add_disp(&disp_cfg);
     if (!lvgl_disp)
     {
        ESP_LOGE(TAG, "[Err] LVGL Display is not setup properly");
        abort();
     }

     /* Apply display orientation (SW rotation). Touch is transformed separately via esp_lcd_touch flags. */
     lvgl_port_lock(0);
     lv_display_set_rotation(lvgl_disp, EXAMPLE_DISP_ROTATION);
     lvgl_port_unlock();

      /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = tp,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);
    if (!lvgl_touch_indev)
    {
       ESP_LOGE(TAG, "[Err] LVGL Touch is not setup properly");
       abort();
    }

    bsp_brightness_init();
    bsp_brightness_set_level(100);

    app_main_display();

    // Allocate shared detection buffer in PSRAM (480*320*2 = 307 200 bytes)
    detect_buf = (uint8_t*)heap_caps_malloc(480 * 320 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (detect_buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc detect_buf in PSRAM");
        abort();
    }
    detect_sem = xSemaphoreCreateBinary();

    // Detection stage visualization buffer (filled by detect_task, displayed by camera_task)
    debug_buf = (uint8_t*)heap_caps_calloc(480 * 320 * 2, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (debug_buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc debug_buf in PSRAM");
        abort();
    }

#if EXAMPLE_CAMERA_ROTATE
    // Rotation buffer for landscape display (320*240*2 = 153 600 bytes)
    rot_buf = (uint8_t*)heap_caps_malloc(320 * 240 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rot_buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc rot_buf in PSRAM");
        abort();
    }
#endif

    camera_init();
    xTaskCreatePinnedToCore(camera_task, "camera_task", 1024 * 10, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(detect_task, "detect_task", 1024 * 20, NULL, 1, NULL, 1);
}

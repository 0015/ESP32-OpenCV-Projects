/*
 * cube_app — cube solver UI state machine (240x320 portrait).
 *
 * Threading model:
 *  - All LVGL objects are created/updated in the LVGL task context
 *    (event callbacks + lv_timer). cube_app_init() runs under lvgl_port_lock.
 *  - cube_app_on_frame() runs in the detection task; it only samples colors
 *    into s_live[] and never calls LVGL.
 */
#include <string.h>
#include <stdio.h>
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "cube_app.h"
#include "cube_vision.h"
#include "cube_model.h"
#include "cube_solve.h"

static const char *TAG = "cube_app";

/* live camera image object owned by main.c */
extern lv_obj_t *img_camera;

/* ---- geometry ---------------------------------------------------------------
 * Screen: 240x320. Camera frame: 480x320 shown center-cropped by LVGL
 * (LV_IMAGE_ALIGN_CENTER) -> camera coord = screen coord + (120, 0).
 */
#define SCR_W        240
#define SCR_H        320
#define CAM_OFF_X    120
#define CAM_OFF_Y    0
#define GRID_CELL    58
#define GRID_X0      33            /* (240 - 3*58)/2 */
#define GRID_Y0      73            /* grid vertically centered around y=160 */
#define SAMPLE_BOX   12

/* Set to 1 if the preview is mirrored vs. reality (sensor hmirror): samples
 * the grid columns right-to-left so stored facelets match the real cube. */
#define SCAN_MIRROR_X 0

/* 2D net (guide screen): 15 px cells, faces laid out as a cross */
#define NET_CELL 15
#define NET_X    30
#define NET_Y    138
static const uint8_t net_fx[6] = { 1, 2, 1, 1, 0, 3 };   /* by face id U,R,F,D,L,B */
static const uint8_t net_fy[6] = { 0, 1, 1, 2, 1, 1 };

/* ---- state ------------------------------------------------------------------*/
typedef enum { APP_START, APP_SCAN, APP_REVIEW, APP_ERROR, APP_GUIDE, APP_DONE } app_state_t;

static volatile app_state_t s_state = APP_START;
static volatile uint8_t s_live[9];                 /* written by detect task   */

static const uint8_t scan_face[6] = { CF_F, CF_R, CF_B, CF_L, CF_U, CF_D };
static int      s_step;                            /* current scan-order index */
static bool     s_scanned[6];                      /* by face id               */
static uint8_t  s_colors[6][9];                    /* by face id, cube_color_t */
static uint8_t  s_face2col[6];                     /* face id -> center color  */
static int      s_pick_cell = -1;
static int      s_err_face  = -1;

static cube_t          g_cube;                     /* guide: current cube state */
static cube_solution_t g_sol;
static int             g_idx;

/* ---- widgets ----------------------------------------------------------------*/
static lv_obj_t *c_start, *c_scan, *c_picker, *c_error, *c_guide, *c_done;
static lv_obj_t *scan_label, *cells[9];
static lv_obj_t *btn_capture, *btn_retry, *btn_ok;
static lv_obj_t *err_label, *btn_err_face, *btn_err_all, *err_face_label;
static lv_obj_t *step_label, *stage_label, *move_label, *hint_label, *net_cells[54], *net_arrow;
static lv_obj_t *btn_prev, *btn_next;
static lv_obj_t *done_info;

/* ---- helpers ----------------------------------------------------------------*/
static const uint32_t pal_hex[CUBE_COL_COUNT] = {
    0xF2F2F2, 0xD32F2F, 0xFF7F00, 0xFFD600, 0x00A650, 0x1565FF,
};
static const char *col_names[CUBE_COL_COUNT] = {
    "WHITE", "RED", "ORANGE", "YELLOW", "GREEN", "BLUE",
};
static const char *face_names[6] = { "TOP", "RIGHT", "FRONT", "BOTTOM", "LEFT", "BACK" };

static const char *scan_text[6] = {
    "Face 1/6: FRONT\nPoint any face at the camera.\nRemember which face is UP!",
    "Face 2/6: RIGHT\nRotate cube 90 LEFT\n(same face stays on top)",
    "Face 3/6: BACK\nRotate cube 90 LEFT again",
    "Face 4/6: LEFT\nRotate cube 90 LEFT again",
    "Face 5/6: TOP\nRotate LEFT once more (start pos),\nthen tilt TOP face to camera",
    "Face 6/6: BOTTOM\nTilt back 180\n(bottom face to camera)",
};

static lv_color_t pal(int c) { return lv_color_hex(pal_hex[c]); }

static void finalize_scan(void);

static lv_obj_t *make_btn(lv_obj_t *parent, const char *txt, int x, int y, int w, int h,
                          lv_event_cb_t cb)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_center(l);
    return b;
}

static lv_obj_t *make_panel(lv_obj_t *parent)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_set_pos(o, 0, 0);
    lv_obj_set_size(o, SCR_W, SCR_H);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_radius(o, 0, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_bg_color(o, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    return o;
}

/* ---- state switching ---------------------------------------------------------*/
static void show_state(app_state_t st)
{
    s_state = st;
    bool cam = (st == APP_SCAN || st == APP_REVIEW);

    lv_obj_add_flag(c_start,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(c_scan,   LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(c_picker, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(c_error,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(c_guide,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(c_done,   LV_OBJ_FLAG_HIDDEN);

    if (cam) lv_obj_remove_flag(img_camera, LV_OBJ_FLAG_HIDDEN);
    else     lv_obj_add_flag(img_camera, LV_OBJ_FLAG_HIDDEN);

    switch (st) {
    case APP_START:  lv_obj_remove_flag(c_start, LV_OBJ_FLAG_HIDDEN); break;
    case APP_SCAN:
    case APP_REVIEW: lv_obj_remove_flag(c_scan,  LV_OBJ_FLAG_HIDDEN); break;
    case APP_ERROR:  lv_obj_remove_flag(c_error, LV_OBJ_FLAG_HIDDEN); break;
    case APP_GUIDE:  lv_obj_remove_flag(c_guide, LV_OBJ_FLAG_HIDDEN); break;
    case APP_DONE:   lv_obj_remove_flag(c_done,  LV_OBJ_FLAG_HIDDEN); break;
    }
}

/* ---- scan / review -----------------------------------------------------------*/
static void enter_scan_step(void)
{
    int face = scan_face[s_step];
    lv_label_set_text_fmt(scan_label, "%s\n(scanning %s face)",
                          scan_text[s_step], face_names[face]);
    for (int i = 0; i < 9; i++) {
        lv_obj_set_style_bg_opa(cells[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(cells[i], lv_color_white(), 0);
    }
    lv_obj_remove_flag(btn_capture, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_retry, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_ok, LV_OBJ_FLAG_HIDDEN);
    show_state(APP_SCAN);
}

static void goto_next_scan(void)
{
    for (int i = 0; i < 6; i++) {
        if (!s_scanned[scan_face[i]]) {
            s_step = i;
            enter_scan_step();
            return;
        }
    }
    /* all faces scanned -> validate & solve */
    finalize_scan();
}

static void begin_scan_all(lv_event_t *e)
{
    (void)e;
    memset(s_scanned, 0, sizeof(s_scanned));
    goto_next_scan();
}

static void show_review(void)
{
    int face = scan_face[s_step];
    lv_label_set_text_fmt(scan_label,
                          "Check the %s face colors.\nTap a wrong cell to fix it,\nthen press OK.",
                          face_names[face]);
    for (int i = 0; i < 9; i++) {
        lv_obj_set_style_bg_color(cells[i], pal(s_colors[face][i]), 0);
        lv_obj_set_style_bg_opa(cells[i], LV_OPA_80, 0);
    }
    lv_obj_add_flag(btn_capture, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(btn_retry, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(btn_ok, LV_OBJ_FLAG_HIDDEN);
    show_state(APP_REVIEW);
}

static void capture_cb(lv_event_t *e)
{
    (void)e;
    int face = scan_face[s_step];
    for (int i = 0; i < 9; i++) s_colors[face][i] = s_live[i];
    show_review();
}

static void retry_cb(lv_event_t *e) { (void)e; enter_scan_step(); }

static void review_ok_cb(lv_event_t *e)
{
    (void)e;
    s_scanned[scan_face[s_step]] = true;
    goto_next_scan();
}

/* ---- manual color picker ------------------------------------------------------*/
static void cell_cb(lv_event_t *e)
{
    if (s_state != APP_REVIEW) return;
    s_pick_cell = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_remove_flag(c_picker, LV_OBJ_FLAG_HIDDEN);
}

static void swatch_cb(lv_event_t *e)
{
    int col = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_pick_cell >= 0 && s_state == APP_REVIEW) {
        int face = scan_face[s_step];
        s_colors[face][s_pick_cell] = (uint8_t)col;
        lv_obj_set_style_bg_color(cells[s_pick_cell], pal(col), 0);
    }
    s_pick_cell = -1;
    lv_obj_add_flag(c_picker, LV_OBJ_FLAG_HIDDEN);
}

static void picker_cancel_cb(lv_event_t *e)
{
    (void)e;
    s_pick_cell = -1;
    lv_obj_add_flag(c_picker, LV_OBJ_FLAG_HIDDEN);
}

/* ---- error screen ---------------------------------------------------------------*/
static void show_error(const char *msg, int rescan_face)
{
    s_err_face = rescan_face;
    lv_label_set_text(err_label, msg);
    if (rescan_face >= 0) {
        lv_label_set_text_fmt(err_face_label, "RESCAN %s", face_names[rescan_face]);
        lv_obj_remove_flag(btn_err_face, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(btn_err_face, LV_OBJ_FLAG_HIDDEN);
    }
    show_state(APP_ERROR);
}

static void err_face_cb(lv_event_t *e)
{
    (void)e;
    if (s_err_face >= 0) {
        s_scanned[s_err_face] = false;
        goto_next_scan();
    }
}

static void err_all_cb(lv_event_t *e) { begin_scan_all(e); }

/* ---- guide screen ----------------------------------------------------------------*/
static const char *move_hint(uint8_t mv)
{
    static const char *tab[6][3] = {
        { "Top layer: turn LEFT",        "Top layer: 180",    "Top layer: turn RIGHT" },
        { "Right column: turn UP",       "Right column: 180", "Right column: turn DOWN" },
        { "Front face: clockwise",       "Front face: 180",   "Front face: counter-cw" },
        { "Bottom layer: turn RIGHT",    "Bottom layer: 180", "Bottom layer: turn LEFT" },
        { "Left column: turn DOWN",      "Left column: 180",  "Left column: turn UP" },
        { "Back face: cw (from behind)", "Back face: 180",    "Back face: ccw (from behind)" },
    };
    return tab[mv / 3][mv % 3];
}

static const char *move_arrow(uint8_t mv)
{
    static const char *tab[6][3] = {
        { LV_SYMBOL_LEFT,    "x2", LV_SYMBOL_RIGHT },   /* U */
        { LV_SYMBOL_UP,      "x2", LV_SYMBOL_DOWN  },   /* R */
        { LV_SYMBOL_REFRESH, "x2", LV_SYMBOL_LOOP  },   /* F */
        { LV_SYMBOL_RIGHT,   "x2", LV_SYMBOL_LEFT  },   /* D */
        { LV_SYMBOL_DOWN,    "x2", LV_SYMBOL_UP    },   /* L */
        { LV_SYMBOL_REFRESH, "x2", LV_SYMBOL_LOOP  },   /* B */
    };
    return tab[mv / 3][mv % 3];
}

static void update_guide(void)
{
    uint8_t mv = g_sol.moves[g_idx];
    int mface = mv / 3;
    int stg = g_sol.stage[g_idx];

    lv_label_set_text_fmt(step_label, "Step %d / %d  -  hold as scanned", g_idx + 1, g_sol.count);
    lv_label_set_text_fmt(stage_label, "%d/7  %s", stg, cube_stage_name(stg));
    lv_label_set_text(move_label, cube_move_name(mv));
    lv_label_set_text(hint_label, move_hint(mv));

    for (int i = 0; i < 54; i++) {
        lv_obj_set_style_bg_color(net_cells[i], pal(s_face2col[g_cube.f[i]]), 0);
        int f = i / 9;
        bool hl = (f == mface);
        lv_obj_set_style_border_color(net_cells[i], hl ? lv_color_white() : lv_color_hex(0x303030), 0);
        lv_obj_set_style_border_width(net_cells[i], hl ? 2 : 1, 0);
    }
    lv_label_set_text(net_arrow, move_arrow(mv));
    lv_obj_set_pos(net_arrow,
                   NET_X + net_fx[mface] * 3 * NET_CELL + 10,
                   NET_Y + net_fy[mface] * 3 * NET_CELL + 13);

    lv_obj_set_style_opa(btn_prev, g_idx > 0 ? LV_OPA_COVER : LV_OPA_40, 0);
}

static void enter_guide(void)
{
    show_state(APP_GUIDE);
    update_guide();
}

static void show_done(void)
{
    lv_label_set_text_fmt(done_info, "Cube solved in %d moves!", g_sol.count);
    show_state(APP_DONE);
}

static void next_cb(lv_event_t *e)
{
    (void)e;
    if (g_idx < g_sol.count) {
        cube_move(&g_cube, g_sol.moves[g_idx]);
        g_idx++;
    }
    if (g_idx >= g_sol.count) show_done();
    else update_guide();
}

static void prev_cb(lv_event_t *e)
{
    (void)e;
    if (g_idx > 0) {
        g_idx--;
        cube_move(&g_cube, cube_move_inv(g_sol.moves[g_idx]));
        update_guide();
    }
}

static void done_back_cb(lv_event_t *e)     /* step back into the guide */
{
    (void)e;
    if (g_sol.count > 0) {
        g_idx--;
        cube_move(&g_cube, cube_move_inv(g_sol.moves[g_idx]));
        enter_guide();
    }
}

static void done_again_cb(lv_event_t *e) { begin_scan_all(e); }

/* ---- validate + solve --------------------------------------------------------------*/
static void finalize_scan(void)
{
    /* center colors must be 6 distinct colors -> color->face mapping */
    int col2face[CUBE_COL_COUNT];
    for (int i = 0; i < CUBE_COL_COUNT; i++) col2face[i] = -1;
    for (int f = 0; f < 6; f++) {
        uint8_t cc = s_colors[f][4];
        if (col2face[cc] >= 0) {
            show_error("Error: two faces have the\nsame CENTER color.\nRescan the cube.", -1);
            return;
        }
        col2face[cc] = f;
        s_face2col[f] = cc;
    }

    cube_t c;
    for (int f = 0; f < 6; f++)
        for (int i = 0; i < 9; i++)
            c.f[f * 9 + i] = (uint8_t)col2face[s_colors[f][i]];

    int bad_color = -1;
    int rc = cube_validate(&c, &bad_color);
    if (rc == CUBE_ERR_COUNT && bad_color >= 0) {
        static char msg[96];
        int bf = col2face[bad_color];
        snprintf(msg, sizeof(msg), "Error detected:\n%s sticker count is not 9.\nRescan the %s face.",
                 col_names[bad_color], face_names[bf]);
        show_error(msg, bf);
        return;
    }
    if (rc != CUBE_OK) {
        ESP_LOGW(TAG, "validate rc=%d", rc);
        show_error("Error detected:\nimpossible cube state\n(a sticker is wrong).\nRescan the cube.", -1);
        return;
    }

    rc = cube_solve(&c, &g_sol);
    ESP_LOGI(TAG, "solve rc=%d moves=%d", rc, g_sol.count);
    if (rc != 0) {
        show_error("Solver failed.\nPlease rescan the cube.", -1);
        return;
    }
    g_cube = c;
    g_idx = 0;
    if (g_sol.count == 0) show_done();
    else enter_guide();
}

/* ---- live color timer (LVGL task) ---------------------------------------------------*/
static void live_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (s_state != APP_SCAN) return;
    for (int i = 0; i < 9; i++)
        lv_obj_set_style_border_color(cells[i], pal(s_live[i]), 0);
}

/* ---- frame hook (detect task) --------------------------------------------------------*/
void cube_app_on_frame(const uint8_t *rgb565, int w, int h)
{
    if (s_state != APP_SCAN) return;

    cube_scan_grid_t grid;
    grid.box = SAMPLE_BOX;
    for (int i = 0; i < 9; i++) {
        int row = i / 3, col = i % 3;
        int scol = SCAN_MIRROR_X ? 2 - col : col;
        grid.cx[i] = CAM_OFF_X + GRID_X0 + scol * GRID_CELL + GRID_CELL / 2;
        grid.cy[i] = CAM_OFF_Y + GRID_Y0 + row * GRID_CELL + GRID_CELL / 2;
    }
    uint8_t out[9];
    cube_scan_cells(rgb565, w, h, &grid, out);
    for (int i = 0; i < 9; i++) s_live[i] = out[i];
}

bool cube_app_camera_active(void)
{
    return s_state == APP_SCAN || s_state == APP_REVIEW;
}

/* ---- UI construction ------------------------------------------------------------------*/
static void build_start(lv_obj_t *scr)
{
    c_start = make_panel(scr);
    lv_obj_t *t = lv_label_create(c_start);
    lv_label_set_text(t, "CUBE\nSOLVER");
    lv_obj_set_style_text_font(t, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(t, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(t, lv_color_white(), 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 55);

    lv_obj_t *st = lv_label_create(c_start);
    lv_label_set_text(st, "Scan all 6 faces, then follow\nthe on-screen solution.");
    lv_obj_set_style_text_align(st, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(st, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(st, LV_ALIGN_TOP_MID, 0, 180);

    make_btn(c_start, "START", 40, 236, 160, 48, begin_scan_all);
}

static void build_scan(lv_obj_t *scr)
{
    c_scan = lv_obj_create(scr);
    lv_obj_set_pos(c_scan, 0, 0);
    lv_obj_set_size(c_scan, SCR_W, SCR_H);
    lv_obj_set_style_pad_all(c_scan, 0, 0);
    lv_obj_set_style_border_width(c_scan, 0, 0);
    lv_obj_set_style_bg_opa(c_scan, LV_OPA_TRANSP, 0);   /* camera visible below */
    lv_obj_remove_flag(c_scan, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(c_scan, LV_OBJ_FLAG_HIDDEN);

    scan_label = lv_label_create(c_scan);
    lv_obj_set_pos(scan_label, 4, 4);
    lv_obj_set_width(scan_label, SCR_W - 8);
    lv_obj_set_style_text_align(scan_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(scan_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(scan_label, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scan_label, LV_OPA_60, 0);
    lv_obj_set_style_pad_all(scan_label, 4, 0);
    lv_obj_set_style_radius(scan_label, 4, 0);

    for (int i = 0; i < 9; i++) {
        lv_obj_t *o = lv_obj_create(c_scan);
        lv_obj_set_pos(o, GRID_X0 + (i % 3) * GRID_CELL, GRID_Y0 + (i / 3) * GRID_CELL);
        lv_obj_set_size(o, GRID_CELL, GRID_CELL);
        lv_obj_set_style_pad_all(o, 0, 0);
        lv_obj_set_style_radius(o, 6, 0);
        lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(o, 3, 0);
        lv_obj_set_style_border_color(o, lv_color_white(), 0);
        lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(o, cell_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        cells[i] = o;
    }

    btn_capture = make_btn(c_scan, "CAPTURE", 20, 272, 200, 40, capture_cb);
    btn_retry   = make_btn(c_scan, "RETRY",   12, 272, 104, 40, retry_cb);
    btn_ok      = make_btn(c_scan, "OK",     124, 272, 104, 40, review_ok_cb);
}

static void build_picker(lv_obj_t *scr)
{
    c_picker = make_panel(scr);
    lv_obj_set_style_bg_opa(c_picker, LV_OPA_60, 0);
    lv_obj_add_flag(c_picker, LV_OBJ_FLAG_CLICKABLE);   /* swallow touches */

    lv_obj_t *p = lv_obj_create(c_picker);
    lv_obj_set_size(p, 204, 226);
    lv_obj_center(p);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_set_style_bg_color(p, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(p, 10, 0);
    lv_obj_remove_flag(p, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(p);
    lv_label_set_text(t, "Pick the cell color");
    lv_obj_set_style_text_color(t, lv_color_white(), 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 8);

    for (int k = 0; k < CUBE_COL_COUNT; k++) {
        lv_obj_t *sw = lv_obj_create(p);
        lv_obj_set_pos(sw, 12 + (k % 3) * 62, 36 + (k / 3) * 62);
        lv_obj_set_size(sw, 56, 56);
        lv_obj_set_style_pad_all(sw, 0, 0);
        lv_obj_set_style_radius(sw, 8, 0);
        lv_obj_set_style_bg_color(sw, pal(k), 0);
        lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(sw, 2, 0);
        lv_obj_set_style_border_color(sw, lv_color_hex(0x606060), 0);
        lv_obj_remove_flag(sw, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(sw, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(sw, swatch_cb, LV_EVENT_CLICKED, (void *)(intptr_t)k);
    }
    lv_obj_t *b = lv_btn_create(p);
    lv_obj_set_pos(b, 12, 172);
    lv_obj_set_size(b, 180, 40);
    lv_obj_add_event_cb(b, picker_cancel_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(b);
    lv_label_set_text(bl, "CANCEL");
    lv_obj_center(bl);
}

static void build_error(lv_obj_t *scr)
{
    c_error = make_panel(scr);

    err_label = lv_label_create(c_error);
    lv_obj_set_pos(err_label, 8, 60);
    lv_obj_set_width(err_label, SCR_W - 16);
    lv_obj_set_style_text_align(err_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(err_label, lv_color_hex(0xFF6060), 0);

    btn_err_face = lv_btn_create(c_error);
    lv_obj_set_pos(btn_err_face, 20, 190);
    lv_obj_set_size(btn_err_face, 200, 44);
    lv_obj_add_event_cb(btn_err_face, err_face_cb, LV_EVENT_CLICKED, NULL);
    err_face_label = lv_label_create(btn_err_face);
    lv_label_set_text(err_face_label, "RESCAN FACE");
    lv_obj_center(err_face_label);

    btn_err_all = make_btn(c_error, "RESCAN ALL", 20, 246, 200, 44, err_all_cb);
    (void)btn_err_all;
}

static void build_guide(lv_obj_t *scr)
{
    c_guide = make_panel(scr);

    step_label = lv_label_create(c_guide);
    lv_obj_set_pos(step_label, 0, 4);
    lv_obj_set_width(step_label, SCR_W);
    lv_obj_set_style_text_align(step_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(step_label, lv_color_hex(0xAAAAAA), 0);

    /* beginner-method stage, e.g. "3/7 Middle layer" */
    stage_label = lv_label_create(c_guide);
    lv_obj_set_pos(stage_label, 0, 21);
    lv_obj_set_width(stage_label, SCR_W);
    lv_obj_set_style_text_align(stage_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(stage_label, lv_color_hex(0xFFD54F), 0);

    move_label = lv_label_create(c_guide);
    lv_obj_set_style_text_font(move_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(move_label, lv_color_hex(0x40C4FF), 0);
    lv_obj_set_width(move_label, SCR_W);
    lv_obj_set_style_text_align(move_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(move_label, 0, 38);

    hint_label = lv_label_create(c_guide);
    lv_obj_set_pos(hint_label, 4, 96);
    lv_obj_set_width(hint_label, SCR_W - 8);
    lv_obj_set_style_text_align(hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(hint_label, lv_color_white(), 0);

    for (int i = 0; i < 54; i++) {
        int f = i / 9, r = (i % 9) / 3, cc = i % 3;
        lv_obj_t *o = lv_obj_create(c_guide);
        lv_obj_set_pos(o, NET_X + net_fx[f] * 3 * NET_CELL + cc * NET_CELL,
                          NET_Y + net_fy[f] * 3 * NET_CELL + r * NET_CELL);
        lv_obj_set_size(o, NET_CELL - 1, NET_CELL - 1);
        lv_obj_set_style_pad_all(o, 0, 0);
        lv_obj_set_style_radius(o, 2, 0);
        lv_obj_set_style_border_width(o, 1, 0);
        lv_obj_set_style_border_color(o, lv_color_hex(0x303030), 0);
        lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
        lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
        net_cells[i] = o;
    }

    net_arrow = lv_label_create(c_guide);
    lv_label_set_text(net_arrow, "");
    lv_obj_set_style_text_color(net_arrow, lv_color_white(), 0);
    lv_obj_set_style_bg_color(net_arrow, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(net_arrow, LV_OPA_70, 0);
    lv_obj_set_style_pad_all(net_arrow, 2, 0);
    lv_obj_set_style_radius(net_arrow, 3, 0);

    btn_prev = make_btn(c_guide, LV_SYMBOL_LEFT " PREV", 8, 280, 108, 34, prev_cb);
    btn_next = make_btn(c_guide, "NEXT " LV_SYMBOL_RIGHT, 124, 280, 108, 34, next_cb);
    (void)btn_next;
}

static void build_done(lv_obj_t *scr)
{
    c_done = make_panel(scr);

    lv_obj_t *t = lv_label_create(c_done);
    lv_label_set_text(t, "SOLVED!");
    lv_obj_set_style_text_font(t, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(0x69F0AE), 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 90);

    done_info = lv_label_create(c_done);
    lv_label_set_text(done_info, "");
    lv_obj_set_style_text_color(done_info, lv_color_white(), 0);
    lv_obj_set_style_text_align(done_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(done_info, SCR_W);
    lv_obj_set_pos(done_info, 0, 160);

    make_btn(c_done, LV_SYMBOL_LEFT " BACK", 20, 214, 200, 40, done_back_cb);
    make_btn(c_done, "SCAN AGAIN", 20, 264, 200, 40, done_again_cb);
}

void cube_app_init(void)
{
    lvgl_port_lock(0);
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    build_start(scr);
    build_scan(scr);
    build_error(scr);
    build_guide(scr);
    build_done(scr);
    build_picker(scr);       /* last -> always on top */

    lv_timer_create(live_timer_cb, 120, NULL);
    show_state(APP_START);
    lvgl_port_unlock();
    ESP_LOGI(TAG, "cube app UI ready");
}

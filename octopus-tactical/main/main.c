/* Octopus Tactical for the Waveshare ESP32-S3-Touch-AMOLED-2.06.
 * ESP-IDF port of the Arduino sketch: clock, gyro compass, screen light, SOS flash.
 *
 * Tap on FACE or COMPASS cycles views. Tap on TOOLS cycles the screen light.
 * Tap on SOS arms or disarms the on-screen Morse beacon.
 * BOOT always cycles to the next view.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "sunmoon.h"

static const char *TAG = "octopus";

#define PIN_BOOT GPIO_NUM_0
#define AXP2101_ADDR 0x34
#define PCF85063_ADDR 0x51
#define QMI8658_ADDR 0x6B
#define QMI8658_ADDR_ALT 0x6A

#define COL_BG 0x0A0E14
#define COL_PANEL 0x121826
#define COL_GREEN 0x00E5A0
#define COL_CYAN 0x4DA3FF
#define COL_AMBER 0xFFC857
#define COL_RED 0xFF4D6D
#define COL_WHITE 0xE8EDF5
#define COL_DIM 0x8B95A8
#define COL_NV 0xFF2038

#define BRIGHT_ON 82
#define BRIGHT_DIM 9
#define DIM_AFTER_MS 20000

/* Sketch asked for GYR_ODR_250Hz. SensorLib's nearest real setting is 224.2 Hz (value 5). */
#define QMI_CTRL3_64DPS_224HZ 0x25
#define QMI_GYRO_LSB_DPS (64.0f / 32768.0f)
#define GYRO_DEADBAND_DPS 1.5f

enum view {
    VIEW_FACE = 0,
    VIEW_COMPASS,
    VIEW_TOOLS,
    VIEW_SOS,
    VIEW_COUNT
};

enum flash_mode {
    FLASH_OFF = 0,
    FLASH_SOLID,
    FLASH_STROBE,
    FLASH_SOS,
    FLASH_RED,
    FLASH_COUNT
};

static const char *FLASH_LABELS[] = {
    "LIGHT OFF", "LIGHT SOLID", "LIGHT STROBE", "LIGHT SOS", "RED NVISION"
};

static i2c_master_dev_handle_t pmu, rtc, imu;
static bool imu_ok;

static enum view cur_view = VIEW_FACE;
static enum flash_mode flash = FLASH_OFF;
static bool morse_active;
static bool sos_armed;
static int sos_countdown = -1; /* -1 idle, 0 fired, 1..8 counting */
static float heading;
static uint16_t sun_az, moon_az;
static int battery = -1;
static double lat = 29.7633, lon = -95.3633; /* Houston, same default as the sketch */
static int hh = 10, mm = 8, ss;
static int year = 2026, mon = 11, day = 5;
static bool dimmed;
static volatile int tap_pending;

static lv_obj_t *views[VIEW_COUNT];
static lv_obj_t *hit;
static lv_obj_t *cover;
static lv_obj_t *lbl_time, *lbl_date, *lbl_batt;
static lv_obj_t *dot_sys, *dot_sun, *dot_moon, *dot_flash;
static lv_obj_t *compass;
static lv_obj_t *lbl_head, *lbl_astro;
static lv_obj_t *light;
static lv_obj_t *lbl_light, *lbl_morse;
static lv_obj_t *lbl_sos;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static float wrap360(float a)
{
    a = fmodf(a, 360.f);
    if (a < 0) {
        a += 360.f;
    }
    return a;
}

static uint8_t bcd2bin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0f)); }
static uint8_t bin2bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static int weekday(int y, int m, int d)
{
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) {
        y -= 1;
    }
    return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

static bool rd(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *buf, size_t n)
{
    return d && i2c_master_transmit_receive(d, &reg, 1, buf, n, 100) == ESP_OK;
}

static bool wr(i2c_master_dev_handle_t d, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return d && i2c_master_transmit(d, b, 2, 100) == ESP_OK;
}

static i2c_master_dev_handle_t add_dev(i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t h = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &h) != ESP_OK) {
        ESP_LOGW(TAG, "i2c 0x%02x not added", addr);
        return NULL;
    }
    return h;
}

static bool qmi_probe(i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_master_dev_handle_t dev = add_dev(bus, addr);
    uint8_t id = 0;
    if (!dev || !rd(dev, 0x00, &id, 1) || id != 0x05) {
        if (dev) {
            i2c_master_bus_rm_device(dev);
        }
        return false;
    }
    imu = dev;
    wr(imu, 0x60, 0xB0); /* soft reset */
    bool reset_ok = false;
    for (int i = 0; i < 50; i++) {
        uint8_t result = 0;
        vTaskDelay(pdMS_TO_TICKS(10));
        if (rd(imu, 0x4D, &result, 1) && result == 0x80) {
            reset_ok = true;
            break;
        }
    }
    uint8_t ctrl1 = 0;
    rd(imu, 0x02, &ctrl1, 1);
    wr(imu, 0x02, ctrl1 | 0x40);          /* address auto-increment, little endian */
    wr(imu, 0x04, QMI_CTRL3_64DPS_224HZ); /* CTRL3 gyro range + ODR */
    wr(imu, 0x06, 0x10);                  /* CTRL5 gyro LPF mode 0, enabled */
    wr(imu, 0x08, 0x02);                  /* CTRL7 gyroscope enable */
    ESP_LOGI(TAG, "QMI8658 @ 0x%02x reset %s", addr, reset_ok ? "ok" : "unconfirmed");
    return true;
}

static void hw_init(void)
{
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        ESP_LOGE(TAG, "no i2c bus");
        return;
    }
    pmu = add_dev(bus, AXP2101_ADDR);
    rtc = add_dev(bus, PCF85063_ADDR);
    imu_ok = qmi_probe(bus, QMI8658_ADDR) || qmi_probe(bus, QMI8658_ADDR_ALT);
    if (!imu_ok) {
        ESP_LOGW(TAG, "QMI8658 not found; heading drifts on its own");
    }

    uint8_t ctrl1 = 0;
    if (rd(rtc, 0x00, &ctrl1, 1)) {
        wr(rtc, 0x00, ctrl1 & (uint8_t) ~((1u << 5) | (1u << 1))); /* run, 24-hour */
    } else {
        ESP_LOGW(TAG, "PCF85063 not found");
        rtc = NULL;
    }
}

static bool rtc_read(int *y, int *mo, int *d, int *h, int *mi, int *s)
{
    uint8_t r[7];
    if (!rd(rtc, 0x04, r, 7) || (r[0] & 0x80)) {
        return false;
    }
    *s = bcd2bin(r[0] & 0x7f);
    *mi = bcd2bin(r[1] & 0x7f);
    *h = bcd2bin(r[2] & 0x3f);
    *d = bcd2bin(r[3] & 0x3f);
    *mo = bcd2bin(r[5] & 0x1f);
    *y = bcd2bin(r[6]) + 2000;
    return *y >= 2024 && *mo >= 1 && *mo <= 12 && *d >= 1 && *d <= 31;
}

static void rtc_write(int y, int mo, int d, int h, int mi, int s)
{
    if (!rtc) {
        return;
    }
    uint8_t buf[8] = {
        0x04,
        bin2bcd(s),
        bin2bcd(mi),
        bin2bcd(h),
        bin2bcd(d),
        (uint8_t)weekday(y, mo, d),
        bin2bcd(mo),
        bin2bcd(y % 100),
    };
    i2c_master_transmit(rtc, buf, sizeof(buf), 100);
}

static int battery_percent(void)
{
    uint8_t status = 0, pct = 0;
    if (!rd(pmu, 0x00, &status, 1) || !(status & 0x08) || !rd(pmu, 0xA4, &pct, 1)) {
        return -1;
    }
    return pct > 100 ? 100 : pct;
}

static bool gyro_z_dps(float *z_out)
{
    uint8_t status = 0, raw[6];
    if (!imu_ok || !rd(imu, 0x2E, &status, 1) || !(status & 0x02) || !rd(imu, 0x3B, raw, 6)) {
        return false;
    }
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);
    *z_out = (float)z * QMI_GYRO_LSB_DPS;
    return true;
}

static int64_t epoch_from_datetime(int y, int mo, int d, int h, int mi, int s)
{
    int64_t yy = y - (mo <= 2);
    int64_t era = (yy >= 0 ? yy : yy - 399) / 400;
    int64_t yoe = yy - era * 400;
    int64_t doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = era * 146097 + doe - 719468;
    return days * 86400 + h * 3600 + mi * 60 + s;
}

static void tick_clock(void)
{
    ss++;
    if (ss < 60) {
        return;
    }
    ss = 0;
    mm++;
    if (mm < 60) {
        return;
    }
    mm = 0;
    hh++;
    if (hh < 24) {
        return;
    }
    hh = 0;
}

static void style_label(lv_obj_t *lbl, uint32_t color, const lv_font_t *font)
{
    lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
    lv_obj_set_style_text_font(lbl, font, 0);
    lv_obj_remove_flag(lbl, LV_OBJ_FLAG_CLICKABLE);
}

static lv_obj_t *make_dot(lv_obj_t *parent, int x)
{
    lv_obj_t *d = lv_obj_create(parent);
    lv_obj_set_size(d, 14, 14);
    lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(d, 0, 0);
    lv_obj_set_style_pad_all(d, 0, 0);
    lv_obj_remove_flag(d, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(d, LV_ALIGN_BOTTOM_MID, x, -36);
    return d;
}

static void on_tap(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        tap_pending = 1;
    }
}

static void show_view(enum view v)
{
    if (v != VIEW_SOS) {
        sos_countdown = -1;
        sos_armed = false;
    }
    cur_view = v;
    for (int i = 0; i < VIEW_COUNT; i++) {
        if (i == (int)v) {
            lv_obj_remove_flag(views[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(views[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (v == VIEW_COMPASS) {
        lv_obj_invalidate(compass);
    }
    lv_obj_move_foreground(hit);
}

static void refresh_face(void)
{
    lv_label_set_text_fmt(lbl_time, "%02d:%02d", hh, mm);
    lv_label_set_text_fmt(lbl_date, "%04d-%02d-%02d", year, mon, day);
    if (battery < 0) {
        lv_label_set_text(lbl_batt, "BAT --");
    } else {
        lv_label_set_text_fmt(lbl_batt, "BAT %d%%", battery);
    }
    lv_obj_set_style_bg_color(dot_flash, lv_color_hex(flash ? COL_RED : COL_DIM), 0);
}

static void refresh_astro(void)
{
    lv_label_set_text_fmt(lbl_astro, "SUN %u    MOON %u", sun_az, moon_az);
    lv_label_set_text_fmt(lbl_head, "%d deg", (int)(heading + 0.5f));
}

static void refresh_tools(void)
{
    uint32_t color = COL_PANEL;
    uint32_t ms = now_ms();
    switch (flash) {
    case FLASH_SOLID:
        color = COL_WHITE;
        break;
    case FLASH_STROBE:
        color = ((ms / 100) % 2) ? COL_WHITE : COL_PANEL;
        break;
    case FLASH_SOS:
        color = ((ms / 2000) % 3 == 0) ? COL_RED : COL_PANEL;
        break;
    case FLASH_RED:
        color = COL_NV;
        break;
    default:
        break;
    }
    lv_obj_set_style_bg_color(light, lv_color_hex(color), 0);
    lv_label_set_text(lbl_light, FLASH_LABELS[flash]);
    lv_obj_set_style_text_color(lbl_morse, lv_color_hex(morse_active ? COL_CYAN : COL_DIM), 0);
    lv_label_set_text(lbl_morse, morse_active ? "MORSE: SOS" : "MORSE: idle");
}

static void refresh_sos(void)
{
    if (sos_countdown > 0) {
        lv_label_set_text_fmt(lbl_sos, "SOS %d", sos_countdown);
    } else if (sos_armed) {
        lv_label_set_text(lbl_sos, "BEACON FIRED");
    } else {
        lv_label_set_text(lbl_sos, "TAP TO ARM");
    }
}

static void refresh_current(void)
{
    switch (cur_view) {
    case VIEW_FACE:
        refresh_face();
        break;
    case VIEW_COMPASS:
        refresh_astro();
        lv_obj_invalidate(compass);
        break;
    case VIEW_TOOLS:
        refresh_tools();
        break;
    case VIEW_SOS:
        refresh_sos();
        break;
    default:
        break;
    }
}

static void cycle_view(void)
{
    show_view((enum view)((cur_view + 1) % VIEW_COUNT));
    refresh_current();
}

static void handle_tap(void)
{
    if (cur_view == VIEW_SOS) {
        if (sos_countdown > 0 || sos_armed) {
            sos_countdown = -1;
            sos_armed = false;
            refresh_sos();
            return;
        }
        sos_countdown = 8;
        refresh_sos();
        return;
    }
    if (cur_view == VIEW_TOOLS) {
        flash = (enum flash_mode)((flash + 1) % FLASH_COUNT);
        refresh_tools();
        refresh_face();
        return;
    }
    cycle_view();
}

static void flash_solid(uint32_t color, uint32_t hold_ms)
{
    if (!bsp_display_lock(200)) {
        return;
    }
    lv_obj_set_style_bg_color(cover, lv_color_hex(color), 0);
    lv_obj_remove_flag(cover, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(cover);
    bsp_display_unlock();
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
}

static void transmit_morse_sos(void)
{
    static const char *pattern = "...---...";
    const uint32_t dit = 120;
    morse_active = true;
    bsp_display_brightness_set(BRIGHT_ON);
    dimmed = false;
    for (int rep = 0; rep < 3; rep++) {
        for (const char *p = pattern; *p; p++) {
            flash_solid(COL_WHITE, *p == '.' ? dit : dit * 3);
            flash_solid(COL_BG, dit);
        }
        vTaskDelay(pdMS_TO_TICKS(700));
    }
    if (bsp_display_lock(200)) {
        lv_obj_add_flag(cover, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(hit);
        morse_active = false;
        refresh_current();
        bsp_display_unlock();
    } else {
        morse_active = false;
    }
}

static void polar(int cx, int cy, float deg_off, float radius, lv_point_precise_t *p)
{
    float a = deg_off * (float)M_PI / 180.f;
    p->x = cx + radius * sinf(a);
    p->y = cy - radius * cosf(a);
}

static void compass_draw(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) {
        return;
    }
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t box;
    lv_obj_get_coords(obj, &box);
    int cx = box.x1 + 205;
    int cy = box.y1 + 251;
    const float r = 180.f;

    lv_draw_rect_dsc_t ring;
    lv_draw_rect_dsc_init(&ring);
    ring.bg_opa = LV_OPA_TRANSP;
    ring.border_opa = LV_OPA_COVER;
    ring.border_width = 2;
    ring.radius = LV_RADIUS_CIRCLE;
    ring.border_color = lv_color_hex(COL_PANEL);
    lv_area_t outer = {cx - 180, cy - 180, cx + 180, cy + 180};
    lv_draw_rect(layer, &ring, &outer);
    ring.border_color = lv_color_hex(0x232D45);
    lv_area_t inner = {cx - 152, cy - 152, cx + 152, cy + 152};
    lv_draw_rect(layer, &ring, &inner);

    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.width = 2;
    line.round_start = 1;
    line.round_end = 1;
    for (int i = 0; i < 24; i++) {
        bool major = (i % 6) == 0;
        float a = (i * 15.f) - heading;
        line.color = lv_color_hex(major ? COL_GREEN : 0x2A3450);
        line.width = major ? 3 : 2;
        polar(cx, cy, a, major ? r - 12.f : r - 6.f, &line.p1);
        polar(cx, cy, a, r, &line.p2);
        lv_draw_line(layer, &line);
    }

    lv_draw_rect_dsc_t mark;
    lv_draw_rect_dsc_init(&mark);
    mark.bg_opa = LV_OPA_COVER;
    mark.radius = LV_RADIUS_CIRCLE;
    mark.border_width = 0;
    lv_point_precise_t sun, moon;
    polar(cx, cy, (float)sun_az - heading, r - 28.f, &sun);
    polar(cx, cy, (float)moon_az - heading, r - 28.f, &moon);
    mark.bg_color = lv_color_hex(COL_AMBER);
    lv_area_t sa = {(int)sun.x - 7, (int)sun.y - 7, (int)sun.x + 7, (int)sun.y + 7};
    lv_draw_rect(layer, &mark, &sa);
    mark.bg_color = lv_color_hex(COL_CYAN);
    lv_area_t ma = {(int)moon.x - 6, (int)moon.y - 6, (int)moon.x + 6, (int)moon.y + 6};
    lv_draw_rect(layer, &mark, &ma);

    line.color = lv_color_hex(COL_GREEN);
    line.width = 3;
    line.p1.x = cx;
    line.p1.y = cy;
    line.p2.x = cx;
    line.p2.y = cy - (int)r + 40;
    lv_draw_line(layer, &line);
    mark.bg_color = lv_color_hex(COL_GREEN);
    lv_area_t hub = {cx - 5, cy - 5, cx + 5, cy + 5};
    lv_draw_rect(layer, &mark, &hub);
}

static lv_obj_t *make_view(lv_obj_t *scr)
{
    lv_obj_t *v = lv_obj_create(scr);
    lv_obj_set_size(v, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(v, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(v, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(v, 0, 0);
    lv_obj_set_style_radius(v, 0, 0);
    lv_obj_set_style_pad_all(v, 0, 0);
    lv_obj_remove_flag(v, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(v, LV_OBJ_FLAG_HIDDEN);
    return v;
}

static lv_obj_t *hint(lv_obj_t *parent, const char *text)
{
    lv_obj_t *lbl = lv_label_create(parent);
    style_label(lbl, COL_DIM, &lv_font_montserrat_16);
    lv_label_set_text(lbl, text);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -12);
    return lbl;
}

static void ui_build(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    views[VIEW_FACE] = make_view(scr);
    lbl_time = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_time, COL_GREEN, &lv_font_montserrat_48);
    lv_obj_align(lbl_time, LV_ALIGN_CENTER, 0, -30);
    lbl_date = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_date, COL_DIM, &lv_font_montserrat_20);
    lv_obj_align(lbl_date, LV_ALIGN_CENTER, 0, 30);
    lbl_batt = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_batt, COL_CYAN, &lv_font_montserrat_20);
    lv_obj_align(lbl_batt, LV_ALIGN_CENTER, 0, 70);
    dot_sys = make_dot(views[VIEW_FACE], -45);
    dot_sun = make_dot(views[VIEW_FACE], -15);
    dot_moon = make_dot(views[VIEW_FACE], 15);
    dot_flash = make_dot(views[VIEW_FACE], 45);
    lv_obj_set_style_bg_color(dot_sys, lv_color_hex(COL_GREEN), 0);
    lv_obj_set_style_bg_color(dot_sun, lv_color_hex(COL_AMBER), 0);
    lv_obj_set_style_bg_color(dot_moon, lv_color_hex(COL_CYAN), 0);
    hint(views[VIEW_FACE], "tap: next");

    views[VIEW_COMPASS] = make_view(scr);
    compass = lv_obj_create(views[VIEW_COMPASS]);
    lv_obj_set_size(compass, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(compass, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(compass, 0, 0);
    lv_obj_remove_flag(compass, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(compass, compass_draw, LV_EVENT_DRAW_MAIN, NULL);
    lbl_astro = lv_label_create(views[VIEW_COMPASS]);
    style_label(lbl_astro, COL_AMBER, &lv_font_montserrat_16);
    lv_obj_align(lbl_astro, LV_ALIGN_TOP_MID, 0, 18);
    lbl_head = lv_label_create(views[VIEW_COMPASS]);
    style_label(lbl_head, COL_WHITE, &lv_font_montserrat_28);
    lv_obj_align(lbl_head, LV_ALIGN_BOTTOM_MID, 0, -48);
    hint(views[VIEW_COMPASS], "tap: next");

    views[VIEW_TOOLS] = make_view(scr);
    light = lv_obj_create(views[VIEW_TOOLS]);
    lv_obj_set_size(light, 160, 160);
    lv_obj_set_style_radius(light, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(light, 0, 0);
    lv_obj_set_style_bg_color(light, lv_color_hex(COL_PANEL), 0);
    lv_obj_set_style_bg_opa(light, LV_OPA_COVER, 0);
    lv_obj_remove_flag(light, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(light, LV_ALIGN_CENTER, 0, -70);
    lbl_light = lv_label_create(views[VIEW_TOOLS]);
    style_label(lbl_light, COL_WHITE, &lv_font_montserrat_20);
    lv_obj_align(lbl_light, LV_ALIGN_CENTER, 0, 60);
    lbl_morse = lv_label_create(views[VIEW_TOOLS]);
    style_label(lbl_morse, COL_DIM, &lv_font_montserrat_20);
    lv_obj_align(lbl_morse, LV_ALIGN_CENTER, 0, 100);
    hint(views[VIEW_TOOLS], "tap: light    boot: next");

    views[VIEW_SOS] = make_view(scr);
    lv_obj_t *beacon = lv_obj_create(views[VIEW_SOS]);
    lv_obj_set_size(beacon, 300, 300);
    lv_obj_set_style_radius(beacon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(beacon, 0, 0);
    lv_obj_set_style_bg_color(beacon, lv_color_hex(COL_RED), 0);
    lv_obj_set_style_bg_opa(beacon, LV_OPA_COVER, 0);
    lv_obj_remove_flag(beacon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(beacon, LV_ALIGN_CENTER, 0, -10);
    lbl_sos = lv_label_create(views[VIEW_SOS]);
    style_label(lbl_sos, COL_WHITE, &lv_font_montserrat_28);
    lv_obj_align(lbl_sos, LV_ALIGN_CENTER, 0, -10);
    hint(views[VIEW_SOS], "tap: arm    boot: next");

    cover = lv_obj_create(scr);
    lv_obj_set_size(cover, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(cover, lv_color_hex(COL_WHITE), 0);
    lv_obj_set_style_bg_opa(cover, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cover, 0, 0);
    lv_obj_set_style_radius(cover, 0, 0);
    lv_obj_remove_flag(cover, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(cover, LV_OBJ_FLAG_HIDDEN);

    hit = lv_obj_create(scr);
    lv_obj_set_size(hit, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(hit, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(hit, 0, 0);
    lv_obj_set_style_radius(hit, 0, 0);
    lv_obj_remove_flag(hit, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hit, on_tap, LV_EVENT_CLICKED, NULL);

    show_view(VIEW_FACE);
    refresh_face();
}

static void note_activity(void)
{
    if (bsp_display_lock(50)) {
        lv_display_trigger_activity(NULL);
        bsp_display_unlock();
    }
    if (dimmed) {
        bsp_display_brightness_set(BRIGHT_ON);
        dimmed = false;
    }
}

static void app_task(void *arg)
{
    uint32_t last_sec = now_ms();
    uint32_t last_ui = last_sec;
    uint32_t last_btn = 0;
    uint32_t last_heading_ms = last_sec;
    bool boot_was_down = false;

    while (1) {
        uint32_t t = now_ms();
        bool boot_down = gpio_get_level(PIN_BOOT) == 0;
        if (boot_down && !boot_was_down && (t - last_btn) > 400) {
            last_btn = t;
            note_activity();
            if (bsp_display_lock(100)) {
                cycle_view();
                bsp_display_unlock();
            }
        }
        boot_was_down = boot_down;

        if (tap_pending) {
            tap_pending = 0;
            note_activity();
            if (bsp_display_lock(100)) {
                handle_tap();
                bsp_display_unlock();
            }
        }

        if (t - last_ui >= 100) {
            float dt = (t - last_heading_ms) / 1000.f;
            last_heading_ms = t;
            last_ui = t;
            float gz = 0;
            if (gyro_z_dps(&gz)) {
                if (fabsf(gz) >= GYRO_DEADBAND_DPS) {
                    heading = wrap360(heading + gz * dt);
                }
            } else {
                heading = wrap360(heading + 1.6f * dt); /* sketch's demo drift, 0.4 deg / 250 ms */
            }
            if (cur_view == VIEW_COMPASS || cur_view == VIEW_TOOLS) {
                if (bsp_display_lock(30)) {
                    refresh_current();
                    bsp_display_unlock();
                }
            }
        }

        if (t - last_sec >= 1000) {
            last_sec = t;
            int y, mo, d, h, mi, s;
            if (rtc_read(&y, &mo, &d, &h, &mi, &s)) {
                year = y;
                mon = mo;
                day = d;
                hh = h;
                mm = mi;
                ss = s;
            } else {
                tick_clock();
            }
            battery = battery_percent();
            sunmoon_positions(lat, lon,
                              epoch_from_datetime(year, mon, day, hh, mm, ss) * 1000,
                              &sun_az, &moon_az);
            bool fire = false;
            if (sos_countdown > 0) {
                sos_countdown--;
                if (sos_countdown == 0) {
                    sos_countdown = -1;
                    sos_armed = true;
                    fire = true;
                }
            }
            if (bsp_display_lock(50)) {
                if (cur_view == VIEW_FACE || cur_view == VIEW_SOS || cur_view == VIEW_COMPASS) {
                    refresh_current();
                }
                uint32_t idle = lv_display_get_inactive_time(NULL);
                bsp_display_unlock();
                if (!dimmed && idle > DIM_AFTER_MS) {
                    bsp_display_brightness_set(BRIGHT_DIM);
                    dimmed = true;
                }
            }
            if (fire) {
                transmit_morse_sos();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "boot");
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_BOOT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    if (!bsp_display_start()) {
        ESP_LOGE(TAG, "display start failed");
        return;
    }
    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "display lock failed");
        return;
    }
    ui_build();
    bsp_display_unlock();
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_ERROR_CHECK(bsp_display_brightness_set(BRIGHT_ON));

    hw_init();
    if (!rtc_read(&year, &mon, &day, &hh, &mm, &ss)) {
        year = 2026;
        mon = 11;
        day = 5;
        hh = 10;
        mm = 8;
        ss = 0;
        rtc_write(year, mon, day, hh, mm, ss);
        ESP_LOGW(TAG, "RTC unset, wrote 2026-11-05 10:08:00");
    } else {
        ESP_LOGI(TAG, "RTC %04d-%02d-%02d %02d:%02d:%02d", year, mon, day, hh, mm, ss);
    }
    battery = battery_percent();
    ESP_LOGI(TAG, "battery %d", battery);
    sunmoon_positions(lat, lon, epoch_from_datetime(year, mon, day, hh, mm, ss) * 1000,
                      &sun_az, &moon_az);

    if (bsp_display_lock(200)) {
        refresh_face();
        bsp_display_unlock();
    }
    ESP_LOGI(TAG, "ready");
    xTaskCreate(app_task, "octopus", 8192, NULL, 5, NULL);
}

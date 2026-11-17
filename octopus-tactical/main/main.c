/* Octopus OS on the Waveshare ESP32-S3-Touch-AMOLED-2.06.
 * Face, app grid, texts, calls, AI, vitals, alerts, time set, nav, music,
 * compass, tools, SOS, companion link, and BLE mod injector.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "sunmoon.h"
#include "app.h"

static const char *TAG = "octopus";

#define PIN_BOOT GPIO_NUM_0
#define AXP2101_ADDR 0x34
#define PCF85063_ADDR 0x51
#define QMI8658_ADDR 0x6B
#define QMI8658_ADDR_ALT 0x6A

#define COL_BG 0x0A0E14
#define COL_PANEL 0x121826
#define COL_CARD 0x1A2234
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
#define MAX_MSGS 10
#define MAX_MODS 6
#define MSG_LEN 96

#define QMI_CTRL3_64DPS_224HZ 0x25
#define QMI_GYRO_LSB_DPS (64.0f / 32768.0f)
#define GYRO_DEADBAND_DPS 1.5f

enum view {
    VIEW_FACE = 0,
    VIEW_GRID,
    VIEW_MSGS,
    VIEW_CALL,
    VIEW_AI,
    VIEW_VITALS,
    VIEW_ALERTS,
    VIEW_TIME,
    VIEW_NAV,
    VIEW_MUSIC,
    VIEW_COMPASS,
    VIEW_TOOLS,
    VIEW_SOS,
    VIEW_LINK,
    VIEW_MOD,
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

static const char *QR[] = {"OK", "On my way", "Call you back", "No"};

static i2c_master_dev_handle_t pmu, rtc, imu;
static bool imu_ok;
static bool linked;
static bool face_long;
static bool time_edit;

static enum view cur_view = VIEW_FACE;
static enum flash_mode flash = FLASH_OFF;
static bool morse_active;
static bool sos_armed;
static int sos_countdown = -1;
static float heading;
static uint16_t sun_az, moon_az;
static int battery = -1;
static double lat = 29.7633, lon = -95.3633;
static int hh = 10, mm = 8, ss;
static int year = 2026, mon = 11, day = 5;
static bool dimmed;

static int e_hh, e_mm, e_day, e_mon, e_year, e_field;
static int vit_hr = -1, vit_spo2 = -1, vit_temp10 = -1, vit_stress = -1;
static int unread;
static bool call_ring;
static char call_name[32];
static bool nav_on;
static int nav_left;
static char nav_dist[24], nav_text[80];
static bool media_on;
static char media_title[40], media_artist[40];
static char msgs[MAX_MSGS][MSG_LEN];
static int msg_count;

typedef struct {
    bool used;
    uint8_t type;
    char name[24];
    char payload[96];
} mod_t;
static mod_t mods[MAX_MODS];
static int mod_n;
static int mod_open = -1;

static lv_obj_t *views[VIEW_COUNT];
static lv_obj_t *cover;
static lv_obj_t *card, *lbl_card;
static lv_obj_t *lbl_time, *lbl_date, *lbl_batt, *lbl_link, *lbl_unread;
static lv_obj_t *msg_list, *alert_list;
static lv_obj_t *lbl_call, *lbl_call_st;
static lv_obj_t *lbl_chat_state, *lbl_chat_you, *lbl_chat_ai;
static lv_obj_t *lbl_hr, *lbl_spo2, *lbl_temp, *lbl_stress, *lbl_sleep;
static lv_obj_t *lbl_clock, *lbl_field;
static lv_obj_t *lbl_nav, *lbl_nav2;
static lv_obj_t *lbl_media, *lbl_media2;
static lv_obj_t *lbl_linkst;
static lv_obj_t *lbl_mod_title, *lbl_mod_body;
static lv_obj_t *mod_tiles[MAX_MODS];
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

static int days_in_month(int y, int m)
{
    static const int d[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) {
        return 29;
    }
    return d[m - 1];
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
    wr(imu, 0x60, 0xB0);
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
    wr(imu, 0x02, ctrl1 | 0x40);
    wr(imu, 0x04, QMI_CTRL3_64DPS_224HZ);
    wr(imu, 0x06, 0x10);
    wr(imu, 0x08, 0x02);
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
    uint8_t ctrl1 = 0;
    if (rd(rtc, 0x00, &ctrl1, 1)) {
        wr(rtc, 0x00, ctrl1 & (uint8_t) ~((1u << 5) | (1u << 1)));
    } else {
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
        0x04, bin2bcd(s), bin2bcd(mi), bin2bcd(h), bin2bcd(d),
        (uint8_t)weekday(y, mo, d), bin2bcd(mo), bin2bcd(y % 100),
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

static lv_obj_t *header(lv_obj_t *parent, const char *text)
{
    lv_obj_t *lbl = lv_label_create(parent);
    style_label(lbl, COL_CYAN, &lv_font_montserrat_20);
    lv_label_set_text(lbl, text);
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 16);
    return lbl;
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

static void refresh_face(void);
static void refresh_msgs(void);
static void refresh_alerts(void);
static void refresh_time(void);
static void refresh_vitals(void);
static void refresh_call(void);
static void refresh_nav(void);
static void refresh_media(void);
static void refresh_link(void);
static void refresh_mod(void);
static void refresh_tools(void);
static void refresh_sos(void);
static void refresh_astro(void);
static void rebuild_mod_tiles(void);

static void show_view(enum view v)
{
    if (v != VIEW_SOS) {
        sos_countdown = -1;
        sos_armed = false;
    }
    if (v == VIEW_TIME) {
        if (cur_view != VIEW_TIME) {
            e_hh = hh;
            e_mm = mm;
            e_day = day;
            e_mon = mon;
            e_year = year;
            e_field = 0;
        }
        time_edit = true;
    } else {
        time_edit = false;
    }
    if (v == VIEW_MSGS) {
        unread = 0;
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
}

static void go_home(void)
{
    if (cur_view == VIEW_FACE) {
        show_view(VIEW_GRID);
    } else if (cur_view == VIEW_GRID) {
        show_view(VIEW_FACE);
        refresh_face();
    } else {
        show_view(VIEW_GRID);
    }
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
    lv_label_set_text(lbl_link, linked ? "PHONE LINK" : "NO PHONE");
    lv_obj_set_style_text_color(lbl_link, lv_color_hex(linked ? COL_CYAN : COL_DIM), 0);
    if (unread) {
        lv_label_set_text_fmt(lbl_unread, "%d unread", unread);
    } else {
        lv_label_set_text(lbl_unread, "");
    }
    lv_obj_set_style_bg_color(dot_flash, lv_color_hex(flash ? COL_RED : COL_DIM), 0);
}

static void fill_list(lv_obj_t *list)
{
    lv_obj_clean(list);
    if (!msg_count) {
        lv_obj_t *e = lv_label_create(list);
        style_label(e, COL_DIM, &lv_font_montserrat_16);
        lv_label_set_text(e, "Quiet. Pair the phone.");
        lv_label_set_long_mode(e, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(e, LV_PCT(100));
        return;
    }
    for (int i = 0; i < msg_count; i++) {
        lv_obj_t *item = lv_label_create(list);
        lv_label_set_text(item, msgs[i]);
        lv_label_set_long_mode(item, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(item, LV_PCT(100));
        lv_obj_set_style_text_font(item, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(item, lv_color_hex(COL_WHITE), 0);
        lv_obj_set_style_bg_color(item, lv_color_hex(COL_CARD), 0);
        lv_obj_set_style_bg_opa(item, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(item, 8, 0);
        lv_obj_set_style_radius(item, 10, 0);
    }
}

static void refresh_msgs(void) { fill_list(msg_list); }
static void refresh_alerts(void) { fill_list(alert_list); }

static const char *field_name(int f)
{
    static const char *n[] = {"HOUR", "MINUTE", "DAY", "MONTH", "YEAR"};
    return n[f];
}

static void refresh_time(void)
{
    lv_label_set_text_fmt(lbl_clock, "%02d:%02d   %04d-%02d-%02d", e_hh, e_mm, e_year, e_mon, e_day);
    lv_label_set_text_fmt(lbl_field, "edit %s   tap + / -", field_name(e_field));
}

static void refresh_vitals(void)
{
    if (vit_hr < 0) {
        lv_label_set_text(lbl_hr, "-- bpm");
    } else {
        lv_label_set_text_fmt(lbl_hr, "%d bpm", vit_hr);
    }
    lv_label_set_text_fmt(lbl_spo2, "SpO2  %s", vit_spo2 < 0 ? "--" : "");
    if (vit_spo2 >= 0) {
        lv_label_set_text_fmt(lbl_spo2, "SpO2  %d%%", vit_spo2);
    }
    if (vit_temp10 < 0) {
        lv_label_set_text(lbl_temp, "Skin  --");
    } else {
        lv_label_set_text_fmt(lbl_temp, "Skin  %d.%d C", vit_temp10 / 10, vit_temp10 % 10);
    }
    if (vit_stress < 0) {
        lv_label_set_text(lbl_stress, "Stress  --");
    } else {
        lv_label_set_text_fmt(lbl_stress, "Stress  %d", vit_stress);
    }
    lv_label_set_text(lbl_sleep, "No PPG on this board.\nVitals come from the phone.");
}

static void refresh_call(void)
{
    if (call_ring) {
        lv_label_set_text(lbl_call, call_name[0] ? call_name : "Incoming");
        lv_label_set_text(lbl_call_st, "RINGING");
    } else {
        lv_label_set_text(lbl_call, "No call");
        lv_label_set_text(lbl_call_st, "idle");
    }
}

static void refresh_nav(void)
{
    if (!nav_on) {
        lv_label_set_text(lbl_nav, "No active route");
        lv_label_set_text(lbl_nav2, "Start nav from the phone.");
        return;
    }
    lv_label_set_text(lbl_nav, nav_left ? "<< LEFT" : "RIGHT >>");
    lv_label_set_text_fmt(lbl_nav2, "%s\n%s", nav_dist, nav_text);
}

static void refresh_media(void)
{
    if (!media_title[0]) {
        lv_label_set_text(lbl_media, "No track");
        lv_label_set_text(lbl_media2, "Push now-playing from the phone.");
        return;
    }
    lv_label_set_text(lbl_media, media_title);
    lv_label_set_text_fmt(lbl_media2, "%s\n%s", media_artist, media_on ? "playing" : "paused");
}

static void refresh_link(void)
{
    lv_label_set_text_fmt(lbl_linkst,
                          "BLE: %s\nName: InfiniTime\nMods: %d / %d\nBattery: %d%%",
                          linked ? "UP" : "DOWN", mod_n, MAX_MODS, battery);
}

static void refresh_mod(void)
{
    if (mod_open < 0 || mod_open >= MAX_MODS || !mods[mod_open].used) {
        lv_label_set_text(lbl_mod_title, "Mod");
        lv_label_set_text(lbl_mod_body, "No module.");
        return;
    }
    lv_label_set_text(lbl_mod_title, mods[mod_open].name);
    lv_label_set_text(lbl_mod_body, mods[mod_open].payload);
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
    case VIEW_MSGS:
        refresh_msgs();
        break;
    case VIEW_ALERTS:
        refresh_alerts();
        break;
    case VIEW_TIME:
        refresh_time();
        break;
    case VIEW_VITALS:
        refresh_vitals();
        break;
    case VIEW_CALL:
        refresh_call();
        break;
    case VIEW_NAV:
        refresh_nav();
        break;
    case VIEW_MUSIC:
        refresh_media();
        break;
    case VIEW_LINK:
        refresh_link();
        break;
    case VIEW_MOD:
        refresh_mod();
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

static void wake_ui(void)
{
    note_activity();
}

static void push_msg(const char *text)
{
    if (msg_count >= MAX_MSGS) {
        for (int i = MAX_MSGS - 1; i > 0; i--) {
            memcpy(msgs[i], msgs[i - 1], MSG_LEN);
        }
    } else {
        for (int i = msg_count; i > 0; i--) {
            memcpy(msgs[i], msgs[i - 1], MSG_LEN);
        }
        msg_count++;
    }
    strlcpy(msgs[0], text, MSG_LEN);
    unread++;
}

static lv_obj_t *make_btn(lv_obj_t *parent, const char *title, int w, int h, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, lv_color_hex(COL_CARD), 0);
    lv_obj_set_style_radius(b, 14, 0);
    if (cb) {
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    }
    lv_obj_t *l = lv_label_create(b);
    style_label(l, COL_WHITE, &lv_font_montserrat_16);
    lv_label_set_text(l, title);
    lv_obj_center(l);
    return b;
}

static void tile_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    show_view((enum view)(uintptr_t)lv_event_get_user_data(e));
    refresh_current();
}

static void face_cb(lv_event_t *e)
{
    lv_event_code_t c = lv_event_get_code(e);
    if (c == LV_EVENT_LONG_PRESSED) {
        face_long = true;
        e_hh = hh;
        e_mm = mm;
        e_day = day;
        e_mon = mon;
        e_year = year;
        e_field = 0;
        time_edit = true;
        show_view(VIEW_TIME);
        refresh_time();
        return;
    }
    if (c == LV_EVENT_CLICKED) {
        if (face_long) {
            face_long = false;
            return;
        }
        show_view(VIEW_GRID);
    }
}

static void card_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        lv_obj_add_flag(card, LV_OBJ_FLAG_HIDDEN);
    }
}

static void qr_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    const char *q = (const char *)lv_event_get_user_data(e);
    uint8_t pkt[80];
    size_t n = strlen(q);
    if (n > 70) {
        n = 70;
    }
    pkt[0] = 1;
    memcpy(pkt + 1, q, n);
    ble_send(pkt, (uint16_t)(n + 1));
    char line[96];
    snprintf(line, sizeof(line), "Me\n%s", q);
    push_msg(line);
    unread = 0;
    refresh_msgs();
}

static void call_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    int ans = (int)(uintptr_t)lv_event_get_user_data(e);
    uint8_t pkt[2] = {2, (uint8_t)ans};
    ble_send(pkt, 2);
    call_ring = false;
    refresh_call();
}

static void talk_cb(lv_event_t *e)
{
    lv_event_code_t c = lv_event_get_code(e);
    if (c == LV_EVENT_PRESSED) {
        chat_talk_start();
    } else if (c == LV_EVENT_RELEASED || c == LV_EVENT_PRESS_LOST) {
        chat_talk_stop();
    }
}

static void time_adj(int dir)
{
    switch (e_field) {
    case 0:
        e_hh = (e_hh + dir + 24) % 24;
        break;
    case 1:
        e_mm = (e_mm + dir + 60) % 60;
        break;
    case 2: {
        int dim = days_in_month(e_year, e_mon);
        e_day = (e_day - 1 + dir + dim) % dim + 1;
        break;
    }
    case 3:
        e_mon = (e_mon - 1 + dir + 12) % 12 + 1;
        if (e_day > days_in_month(e_year, e_mon)) {
            e_day = days_in_month(e_year, e_mon);
        }
        break;
    case 4:
        e_year += dir;
        if (e_year < 2024) {
            e_year = 2024;
        }
        if (e_year > 2099) {
            e_year = 2099;
        }
        break;
    }
    refresh_time();
}

static void time_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    int cmd = (int)(uintptr_t)lv_event_get_user_data(e);
    if (cmd == 0) {
        time_adj(-1);
    } else if (cmd == 1) {
        time_adj(1);
    } else if (cmd == 2) {
        e_field = (e_field + 1) % 5;
        refresh_time();
    } else {
        hh = e_hh;
        mm = e_mm;
        ss = 0;
        day = e_day;
        mon = e_mon;
        year = e_year;
        rtc_write(year, mon, day, hh, mm, ss);
        time_edit = false;
        show_view(VIEW_FACE);
        refresh_face();
    }
}

static void media_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    uint8_t pkt[2] = {3, (uint8_t)(uintptr_t)lv_event_get_user_data(e)};
    ble_send(pkt, 2);
}

static void flash_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    flash = (enum flash_mode)(uintptr_t)lv_event_get_user_data(e);
    refresh_tools();
}

static void sos_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (sos_countdown > 0 || sos_armed) {
        sos_countdown = -1;
        sos_armed = false;
    } else {
        sos_countdown = 8;
    }
    refresh_sos();
}

static void run_mod(int i)
{
    if (i < 0 || i >= MAX_MODS || !mods[i].used) {
        return;
    }
    mod_open = i;
    if (mods[i].type == 2) {
        ui_chat_text(mods[i].payload, "prompt injected from mod");
        show_view(VIEW_AI);
        return;
    }
    show_view(VIEW_MOD);
    refresh_mod();
}

static void mod_tile_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    run_mod((int)(uintptr_t)lv_event_get_user_data(e));
}

static void rebuild_mod_tiles(void)
{
    for (int i = 0; i < MAX_MODS; i++) {
        lv_obj_t *lab = lv_obj_get_child(mod_tiles[i], 0);
        if (mods[i].used) {
            lv_label_set_text(lab, mods[i].name);
            lv_obj_remove_flag(mod_tiles[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(mod_tiles[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
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

void app_set_time(int y, int month, int d, int hour, int min, int sec)
{
    year = y;
    mon = month;
    day = d;
    hh = hour;
    mm = min;
    ss = sec;
    rtc_write(year, mon, day, hh, mm, ss);
    ESP_LOGI(TAG, "time from phone %04d-%02d-%02d %02d:%02d:%02d", y, month, d, hour, min, sec);
    if (bsp_display_lock(100)) {
        refresh_face();
        bsp_display_unlock();
    }
    wake_ui();
}

void app_on_notification(const char *text)
{
    ESP_LOGI(TAG, "notify %s", text);
    push_msg(text);
    if (bsp_display_lock(200)) {
        lv_label_set_text(lbl_card, text);
        lv_obj_remove_flag(card, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(card);
        refresh_msgs();
        refresh_alerts();
        refresh_face();
        bsp_display_unlock();
    }
    wake_ui();
}

void app_on_connection(bool connected)
{
    linked = connected;
    if (bsp_display_lock(100)) {
        refresh_face();
        refresh_link();
        bsp_display_unlock();
    }
    wake_ui();
}

void app_set_speed(int16_t speed_dkmh, uint16_t heading_deg)
{
    (void)speed_dkmh;
    if (heading_deg < 360) {
        heading = (float)heading_deg;
    }
}

void app_on_call(bool ringing, const char *name)
{
    call_ring = ringing;
    strlcpy(call_name, name ? name : "", sizeof(call_name));
    if (bsp_display_lock(200)) {
        if (ringing) {
            show_view(VIEW_CALL);
        }
        refresh_call();
        bsp_display_unlock();
    }
    wake_ui();
}

void app_on_nav(bool on, int dir_left, const char *dist, const char *text)
{
    nav_on = on;
    nav_left = dir_left;
    strlcpy(nav_dist, dist ? dist : "", sizeof(nav_dist));
    strlcpy(nav_text, text ? text : "", sizeof(nav_text));
    if (bsp_display_lock(150)) {
        if (on) {
            show_view(VIEW_NAV);
        }
        refresh_nav();
        bsp_display_unlock();
    }
    wake_ui();
}

void app_on_media(bool playing, const char *title, const char *artist)
{
    media_on = playing;
    strlcpy(media_title, title ? title : "", sizeof(media_title));
    strlcpy(media_artist, artist ? artist : "", sizeof(media_artist));
    if (bsp_display_lock(100)) {
        refresh_media();
        bsp_display_unlock();
    }
}

void app_on_vitals(int hr, int spo2, int temp10, int stress)
{
    vit_hr = hr;
    vit_spo2 = spo2;
    vit_temp10 = temp10;
    vit_stress = stress;
    if (bsp_display_lock(100)) {
        refresh_face();
        refresh_vitals();
        bsp_display_unlock();
    }
}

void app_on_ai_text(const char *you, const char *ai)
{
    ui_chat_text(you, ai);
    if (bsp_display_lock(150)) {
        show_view(VIEW_AI);
        bsp_display_unlock();
    }
    wake_ui();
}

void ui_chat_status(const char *status)
{
    if (bsp_display_lock(150)) {
        lv_label_set_text(lbl_chat_state, status);
        bsp_display_unlock();
    }
}

void ui_chat_text(const char *you, const char *ai)
{
    if (bsp_display_lock(200)) {
        lv_label_set_text_fmt(lbl_chat_you, "You: %s", you ? you : "");
        lv_label_set_text(lbl_chat_ai, ai ? ai : "");
        bsp_display_unlock();
    }
    wake_ui();
}

void app_on_mod_inject(uint8_t type, const char *name, const char *payload)
{
    int slot = -1;
    for (int i = 0; i < MAX_MODS; i++) {
        if (!mods[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "mod slots full");
        return;
    }
    mods[slot].used = true;
    mods[slot].type = type;
    strlcpy(mods[slot].name, name && name[0] ? name : "Mod", sizeof(mods[slot].name));
    strlcpy(mods[slot].payload, payload ? payload : "", sizeof(mods[slot].payload));
    mod_n++;
    ESP_LOGI(TAG, "mod inject '%s' type %u slot %d", mods[slot].name, type, slot);
    if (bsp_display_lock(200)) {
        rebuild_mod_tiles();
        show_view(VIEW_GRID);
        bsp_display_unlock();
    }
    wake_ui();
}

void app_on_mod_purge(int index)
{
    if (index < 0 || index >= MAX_MODS || !mods[index].used) {
        return;
    }
    mods[index].used = false;
    mod_n--;
    if (mod_n < 0) {
        mod_n = 0;
    }
    if (bsp_display_lock(150)) {
        rebuild_mod_tiles();
        bsp_display_unlock();
    }
}

void app_on_mod_purge_all(void)
{
    memset(mods, 0, sizeof(mods));
    mod_n = 0;
    if (bsp_display_lock(150)) {
        rebuild_mod_tiles();
        bsp_display_unlock();
    }
}

static void ui_build(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    views[VIEW_FACE] = make_view(scr);
    lv_obj_add_flag(views[VIEW_FACE], LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(views[VIEW_FACE], face_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(views[VIEW_FACE], face_cb, LV_EVENT_LONG_PRESSED, NULL);
    lbl_time = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_time, COL_GREEN, &lv_font_montserrat_48);
    lv_obj_align(lbl_time, LV_ALIGN_CENTER, 0, -50);
    lbl_date = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_date, COL_DIM, &lv_font_montserrat_20);
    lv_obj_align(lbl_date, LV_ALIGN_CENTER, 0, 10);
    lbl_batt = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_batt, COL_CYAN, &lv_font_montserrat_20);
    lv_obj_align(lbl_batt, LV_ALIGN_CENTER, 0, 46);
    lbl_link = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_link, COL_DIM, &lv_font_montserrat_16);
    lv_obj_align(lbl_link, LV_ALIGN_CENTER, 0, 78);
    lbl_unread = lv_label_create(views[VIEW_FACE]);
    style_label(lbl_unread, COL_AMBER, &lv_font_montserrat_16);
    lv_obj_align(lbl_unread, LV_ALIGN_CENTER, 0, 104);
    dot_sys = make_dot(views[VIEW_FACE], -45);
    dot_sun = make_dot(views[VIEW_FACE], -15);
    dot_moon = make_dot(views[VIEW_FACE], 15);
    dot_flash = make_dot(views[VIEW_FACE], 45);
    lv_obj_set_style_bg_color(dot_sys, lv_color_hex(COL_GREEN), 0);
    lv_obj_set_style_bg_color(dot_sun, lv_color_hex(COL_AMBER), 0);
    lv_obj_set_style_bg_color(dot_moon, lv_color_hex(COL_CYAN), 0);
    hint(views[VIEW_FACE], "tap apps    hold set time");

    views[VIEW_GRID] = make_view(scr);
    lv_obj_add_flag(views[VIEW_GRID], LV_OBJ_FLAG_SCROLLABLE);
    header(views[VIEW_GRID], "Apps");
    static const struct {
        const char *name;
        enum view v;
    } tiles[] = {
        {"Texts", VIEW_MSGS},   {"Call", VIEW_CALL},     {"AI", VIEW_AI},
        {"Vitals", VIEW_VITALS}, {"Alerts", VIEW_ALERTS}, {"Time", VIEW_TIME},
        {"Nav", VIEW_NAV},       {"Music", VIEW_MUSIC},   {"Compass", VIEW_COMPASS},
        {"Tools", VIEW_TOOLS},   {"SOS", VIEW_SOS},       {"Link", VIEW_LINK},
    };
    for (int i = 0; i < 12; i++) {
        int col = i % 3, row = i / 3;
        lv_obj_t *b = make_btn(views[VIEW_GRID], tiles[i].name, 118, 82, tile_cb, (void *)(uintptr_t)tiles[i].v);
        lv_obj_set_pos(b, 16 + col * 128, 52 + row * 92);
    }
    for (int i = 0; i < MAX_MODS; i++) {
        int idx = 12 + i;
        int col = idx % 3, row = idx / 3;
        mod_tiles[i] = make_btn(views[VIEW_GRID], "Mod", 118, 82, mod_tile_cb, (void *)(uintptr_t)i);
        lv_obj_set_pos(mod_tiles[i], 16 + col * 128, 52 + row * 92);
        lv_obj_set_style_bg_color(mod_tiles[i], lv_color_hex(0x163024), 0);
        lv_obj_add_flag(mod_tiles[i], LV_OBJ_FLAG_HIDDEN);
    }
    hint(views[VIEW_GRID], "boot: face    mods land here");

    views[VIEW_MSGS] = make_view(scr);
    header(views[VIEW_MSGS], "Messages");
    msg_list = lv_obj_create(views[VIEW_MSGS]);
    lv_obj_set_size(msg_list, 378, 300);
    lv_obj_align(msg_list, LV_ALIGN_TOP_MID, 0, 48);
    lv_obj_set_flex_flow(msg_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_bg_opa(msg_list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(msg_list, 0, 0);
    lv_obj_set_style_pad_row(msg_list, 8, 0);
    for (int i = 0; i < 4; i++) {
        lv_obj_t *b = make_btn(views[VIEW_MSGS], QR[i], 88, 44, qr_cb, (void *)QR[i]);
        lv_obj_set_pos(b, 16 + i * 96, 360);
    }
    hint(views[VIEW_MSGS], "quick reply sends to phone");

    views[VIEW_ALERTS] = make_view(scr);
    header(views[VIEW_ALERTS], "Alerts");
    alert_list = lv_obj_create(views[VIEW_ALERTS]);
    lv_obj_set_size(alert_list, 378, 380);
    lv_obj_align(alert_list, LV_ALIGN_TOP_MID, 0, 48);
    lv_obj_set_flex_flow(alert_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_bg_opa(alert_list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(alert_list, 0, 0);
    hint(views[VIEW_ALERTS], "Gadgetbridge or companion");

    views[VIEW_CALL] = make_view(scr);
    header(views[VIEW_CALL], "Call");
    lbl_call = lv_label_create(views[VIEW_CALL]);
    style_label(lbl_call, COL_WHITE, &lv_font_montserrat_28);
    lv_obj_align(lbl_call, LV_ALIGN_CENTER, 0, -40);
    lbl_call_st = lv_label_create(views[VIEW_CALL]);
    style_label(lbl_call_st, COL_DIM, &lv_font_montserrat_16);
    lv_obj_align(lbl_call_st, LV_ALIGN_CENTER, 0, 8);
    make_btn(views[VIEW_CALL], "Answer", 150, 56, call_btn_cb, (void *)(uintptr_t)1);
    lv_obj_align(lv_obj_get_child(views[VIEW_CALL], -1), LV_ALIGN_BOTTOM_LEFT, 24, -70);
    lv_obj_t *dec = make_btn(views[VIEW_CALL], "Decline", 150, 56, call_btn_cb, (void *)(uintptr_t)0);
    lv_obj_set_style_bg_color(dec, lv_color_hex(COL_RED), 0);
    lv_obj_align(dec, LV_ALIGN_BOTTOM_RIGHT, -24, -70);
    hint(views[VIEW_CALL], "rings from the phone");

    views[VIEW_AI] = make_view(scr);
    header(views[VIEW_AI], "Assistant");
    lbl_chat_state = lv_label_create(views[VIEW_AI]);
    style_label(lbl_chat_state, COL_DIM, &lv_font_montserrat_16);
    lv_label_set_text(lbl_chat_state, "not set up - BLE config");
    lv_obj_align(lbl_chat_state, LV_ALIGN_TOP_MID, 0, 50);
    lbl_chat_you = lv_label_create(views[VIEW_AI]);
    style_label(lbl_chat_you, COL_DIM, &lv_font_montserrat_16);
    lv_label_set_long_mode(lbl_chat_you, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_chat_you, 360);
    lv_obj_align(lbl_chat_you, LV_ALIGN_TOP_MID, 0, 90);
    lbl_chat_ai = lv_label_create(views[VIEW_AI]);
    style_label(lbl_chat_ai, COL_WHITE, &lv_font_montserrat_20);
    lv_label_set_long_mode(lbl_chat_ai, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_chat_ai, 360);
    lv_obj_align(lbl_chat_ai, LV_ALIGN_TOP_MID, 0, 160);
    lv_obj_t *talk = make_btn(views[VIEW_AI], "Hold to talk", 260, 70, NULL, NULL);
    lv_obj_align(talk, LV_ALIGN_BOTTOM_MID, 0, -70);
    lv_obj_add_event_cb(talk, talk_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(talk, talk_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(talk, talk_cb, LV_EVENT_PRESS_LOST, NULL);
    hint(views[VIEW_AI], "WiFi + server from companion");

    views[VIEW_VITALS] = make_view(scr);
    header(views[VIEW_VITALS], "Vitals");
    lbl_hr = lv_label_create(views[VIEW_VITALS]);
    style_label(lbl_hr, COL_RED, &lv_font_montserrat_48);
    lv_obj_align(lbl_hr, LV_ALIGN_TOP_MID, 0, 60);
    lbl_spo2 = lv_label_create(views[VIEW_VITALS]);
    style_label(lbl_spo2, COL_CYAN, &lv_font_montserrat_20);
    lv_obj_align(lbl_spo2, LV_ALIGN_TOP_MID, 0, 140);
    lbl_temp = lv_label_create(views[VIEW_VITALS]);
    style_label(lbl_temp, COL_AMBER, &lv_font_montserrat_20);
    lv_obj_align(lbl_temp, LV_ALIGN_TOP_MID, 0, 176);
    lbl_stress = lv_label_create(views[VIEW_VITALS]);
    style_label(lbl_stress, COL_GREEN, &lv_font_montserrat_20);
    lv_obj_align(lbl_stress, LV_ALIGN_TOP_MID, 0, 212);
    lbl_sleep = lv_label_create(views[VIEW_VITALS]);
    style_label(lbl_sleep, COL_DIM, &lv_font_montserrat_16);
    lv_label_set_long_mode(lbl_sleep, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_sleep, 340);
    lv_obj_align(lbl_sleep, LV_ALIGN_TOP_MID, 0, 270);
    hint(views[VIEW_VITALS], "phone pushes HR / SpO2");

    views[VIEW_TIME] = make_view(scr);
    header(views[VIEW_TIME], "Set time");
    lbl_clock = lv_label_create(views[VIEW_TIME]);
    style_label(lbl_clock, COL_GREEN, &lv_font_montserrat_28);
    lv_obj_align(lbl_clock, LV_ALIGN_CENTER, 0, -80);
    lbl_field = lv_label_create(views[VIEW_TIME]);
    style_label(lbl_field, COL_DIM, &lv_font_montserrat_16);
    lv_obj_align(lbl_field, LV_ALIGN_CENTER, 0, -30);
    make_btn(views[VIEW_TIME], "-", 90, 60, time_btn_cb, (void *)(uintptr_t)0);
    lv_obj_align(lv_obj_get_child(views[VIEW_TIME], -1), LV_ALIGN_CENTER, -110, 50);
    make_btn(views[VIEW_TIME], "+", 90, 60, time_btn_cb, (void *)(uintptr_t)1);
    lv_obj_align(lv_obj_get_child(views[VIEW_TIME], -1), LV_ALIGN_CENTER, 110, 50);
    make_btn(views[VIEW_TIME], "Next field", 170, 50, time_btn_cb, (void *)(uintptr_t)2);
    lv_obj_align(lv_obj_get_child(views[VIEW_TIME], -1), LV_ALIGN_CENTER, 0, 120);
    make_btn(views[VIEW_TIME], "Save", 170, 50, time_btn_cb, (void *)(uintptr_t)3);
    lv_obj_align(lv_obj_get_child(views[VIEW_TIME], -1), LV_ALIGN_CENTER, 0, 180);
    hint(views[VIEW_TIME], "or sync from the phone");

    views[VIEW_NAV] = make_view(scr);
    header(views[VIEW_NAV], "Nav");
    lbl_nav = lv_label_create(views[VIEW_NAV]);
    style_label(lbl_nav, COL_GREEN, &lv_font_montserrat_28);
    lv_obj_align(lbl_nav, LV_ALIGN_CENTER, 0, -20);
    lbl_nav2 = lv_label_create(views[VIEW_NAV]);
    style_label(lbl_nav2, COL_WHITE, &lv_font_montserrat_16);
    lv_label_set_long_mode(lbl_nav2, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_nav2, 340);
    lv_obj_align(lbl_nav2, LV_ALIGN_CENTER, 0, 50);
    hint(views[VIEW_NAV], "one pulse right, two left");

    views[VIEW_MUSIC] = make_view(scr);
    header(views[VIEW_MUSIC], "Music");
    lbl_media = lv_label_create(views[VIEW_MUSIC]);
    style_label(lbl_media, COL_WHITE, &lv_font_montserrat_28);
    lv_label_set_long_mode(lbl_media, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_media, 340);
    lv_obj_align(lbl_media, LV_ALIGN_CENTER, 0, -40);
    lbl_media2 = lv_label_create(views[VIEW_MUSIC]);
    style_label(lbl_media2, COL_DIM, &lv_font_montserrat_16);
    lv_label_set_long_mode(lbl_media2, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_media2, 340);
    lv_obj_align(lbl_media2, LV_ALIGN_CENTER, 0, 30);
    make_btn(views[VIEW_MUSIC], "Prev", 90, 50, media_cb, (void *)(uintptr_t)3);
    lv_obj_align(lv_obj_get_child(views[VIEW_MUSIC], -1), LV_ALIGN_BOTTOM_MID, -120, -80);
    make_btn(views[VIEW_MUSIC], "Play", 90, 50, media_cb, (void *)(uintptr_t)1);
    lv_obj_align(lv_obj_get_child(views[VIEW_MUSIC], -1), LV_ALIGN_BOTTOM_MID, 0, -80);
    make_btn(views[VIEW_MUSIC], "Next", 90, 50, media_cb, (void *)(uintptr_t)2);
    lv_obj_align(lv_obj_get_child(views[VIEW_MUSIC], -1), LV_ALIGN_BOTTOM_MID, 120, -80);
    hint(views[VIEW_MUSIC], "controls the phone");

    views[VIEW_LINK] = make_view(scr);
    header(views[VIEW_LINK], "Companion");
    lbl_linkst = lv_label_create(views[VIEW_LINK]);
    style_label(lbl_linkst, COL_WHITE, &lv_font_montserrat_20);
    lv_label_set_long_mode(lbl_linkst, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_linkst, 340);
    lv_obj_align(lbl_linkst, LV_ALIGN_CENTER, 0, 0);
    hint(views[VIEW_LINK], "advertises as InfiniTime");

    views[VIEW_MOD] = make_view(scr);
    lbl_mod_title = header(views[VIEW_MOD], "Mod");
    lbl_mod_body = lv_label_create(views[VIEW_MOD]);
    style_label(lbl_mod_body, COL_WHITE, &lv_font_montserrat_20);
    lv_label_set_long_mode(lbl_mod_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_mod_body, 340);
    lv_obj_align(lbl_mod_body, LV_ALIGN_CENTER, 0, 0);
    hint(views[VIEW_MOD], "injected from Mod Injector");

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
    hint(views[VIEW_COMPASS], "boot: apps");

    views[VIEW_TOOLS] = make_view(scr);
    header(views[VIEW_TOOLS], "Tools");
    light = lv_obj_create(views[VIEW_TOOLS]);
    lv_obj_set_size(light, 120, 120);
    lv_obj_set_style_radius(light, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(light, 0, 0);
    lv_obj_set_style_bg_color(light, lv_color_hex(COL_PANEL), 0);
    lv_obj_set_style_bg_opa(light, LV_OPA_COVER, 0);
    lv_obj_remove_flag(light, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(light, LV_ALIGN_TOP_MID, 0, 50);
    lbl_light = lv_label_create(views[VIEW_TOOLS]);
    style_label(lbl_light, COL_WHITE, &lv_font_montserrat_20);
    lv_obj_align(lbl_light, LV_ALIGN_TOP_MID, 0, 184);
    lbl_morse = lv_label_create(views[VIEW_TOOLS]);
    style_label(lbl_morse, COL_DIM, &lv_font_montserrat_16);
    lv_obj_align(lbl_morse, LV_ALIGN_TOP_MID, 0, 216);
    make_btn(views[VIEW_TOOLS], "Solid", 118, 48, flash_cb, (void *)(uintptr_t)FLASH_SOLID);
    lv_obj_set_pos(lv_obj_get_child(views[VIEW_TOOLS], -1), 16, 260);
    make_btn(views[VIEW_TOOLS], "Strobe", 118, 48, flash_cb, (void *)(uintptr_t)FLASH_STROBE);
    lv_obj_set_pos(lv_obj_get_child(views[VIEW_TOOLS], -1), 146, 260);
    make_btn(views[VIEW_TOOLS], "SOS", 118, 48, flash_cb, (void *)(uintptr_t)FLASH_SOS);
    lv_obj_set_pos(lv_obj_get_child(views[VIEW_TOOLS], -1), 276, 260);
    make_btn(views[VIEW_TOOLS], "Red NV", 118, 48, flash_cb, (void *)(uintptr_t)FLASH_RED);
    lv_obj_set_pos(lv_obj_get_child(views[VIEW_TOOLS], -1), 16, 320);
    make_btn(views[VIEW_TOOLS], "Off", 248, 48, flash_cb, (void *)(uintptr_t)FLASH_OFF);
    lv_obj_set_pos(lv_obj_get_child(views[VIEW_TOOLS], -1), 146, 320);
    hint(views[VIEW_TOOLS], "screen light");

    views[VIEW_SOS] = make_view(scr);
    lv_obj_t *beacon = lv_obj_create(views[VIEW_SOS]);
    lv_obj_set_size(beacon, 280, 280);
    lv_obj_set_style_radius(beacon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(beacon, 0, 0);
    lv_obj_set_style_bg_color(beacon, lv_color_hex(COL_RED), 0);
    lv_obj_set_style_bg_opa(beacon, LV_OPA_COVER, 0);
    lv_obj_add_flag(beacon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(beacon, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(beacon, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_event_cb(beacon, sos_cb, LV_EVENT_CLICKED, NULL);
    lbl_sos = lv_label_create(views[VIEW_SOS]);
    style_label(lbl_sos, COL_WHITE, &lv_font_montserrat_28);
    lv_obj_align(lbl_sos, LV_ALIGN_CENTER, 0, -10);
    hint(views[VIEW_SOS], "tap: 8s then Morse flash");

    cover = lv_obj_create(scr);
    lv_obj_set_size(cover, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(cover, lv_color_hex(COL_WHITE), 0);
    lv_obj_set_style_border_width(cover, 0, 0);
    lv_obj_add_flag(cover, LV_OBJ_FLAG_HIDDEN);

    card = lv_obj_create(lv_layer_top());
    lv_obj_set_size(card, LV_PCT(88), LV_PCT(50));
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(COL_CARD), 0);
    lv_obj_set_style_radius(card, 24, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_add_flag(card, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(card, card_cb, LV_EVENT_CLICKED, NULL);
    lbl_card = lv_label_create(card);
    style_label(lbl_card, COL_WHITE, &lv_font_montserrat_20);
    lv_label_set_long_mode(lbl_card, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_card, LV_PCT(100));

    show_view(VIEW_FACE);
    refresh_face();
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
                go_home();
                bsp_display_unlock();
            }
        }
        boot_was_down = boot_down;

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
                heading = wrap360(heading + 1.6f * dt);
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
            if (!time_edit) {
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
            }
            battery = battery_percent();
            if (battery >= 0) {
                ble_set_battery((uint8_t)battery);
            }
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
                if (cur_view == VIEW_FACE || cur_view == VIEW_SOS || cur_view == VIEW_COMPASS ||
                    cur_view == VIEW_TIME) {
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
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

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
    }
    battery = battery_percent();
    sunmoon_positions(lat, lon, epoch_from_datetime(year, mon, day, hh, mm, ss) * 1000,
                      &sun_az, &moon_az);
    if (bsp_display_lock(200)) {
        refresh_face();
        bsp_display_unlock();
    }

    ble_start();
    chat_init();
    ESP_LOGI(TAG, "ready");
    xTaskCreate(app_task, "octopus", 8192, NULL, 5, NULL);
}

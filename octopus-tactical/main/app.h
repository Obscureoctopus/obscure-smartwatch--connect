#pragma once

#include <stdbool.h>
#include <stdint.h>

void app_set_time(int year, int month, int day, int hour, int min, int sec);
void app_on_notification(const char *text);
void app_on_connection(bool connected);
void app_set_speed(int16_t speed_dkmh, uint16_t heading_deg);
void app_set_chat_config(const char *cfg);
void app_on_call(bool ringing, const char *name);
void app_on_nav(bool on, int dir_left, const char *dist, const char *text);
void app_on_media(bool playing, const char *title, const char *artist);
void app_on_vitals(int hr, int spo2, int temp10, int stress);
void app_on_ai_text(const char *you, const char *ai);
void app_on_mod_inject(uint8_t type, const char *name, const char *payload);
void app_on_mod_purge(int index);
void app_on_mod_purge_all(void);

void ble_start(void);
void ble_set_battery(uint8_t percent);
void ble_send(const uint8_t *data, uint16_t len);

void chat_init(void);
void chat_talk_start(void);
void chat_talk_stop(void);

void ui_chat_status(const char *status);
void ui_chat_text(const char *you, const char *ai);

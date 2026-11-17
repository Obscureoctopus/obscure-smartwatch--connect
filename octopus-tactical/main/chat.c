#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_websocket_client.h"
#include "esp_wifi.h"
#include "esp_codec_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs.h"
#include "bsp/esp-bsp.h"
#include "app.h"

static const char *TAG = "chat";

#define MIC_RATE 16000
#define MAX_REPLY_BYTES (16000 * 2 * 20)
#define MAX_RECORD_MS 15000
#define SPK_VOLUME 85
#define MIC_GAIN_DB 30.0f

static char ssid[33], pass[65], uri[128];
static bool wifi_up, wifi_inited;
static esp_websocket_client_handle_t ws;
static esp_codec_dev_handle_t spk, mic;
static volatile bool talking;
static TaskHandle_t rec_task;

static uint8_t *reply;
static size_t reply_len;
static int reply_rate = 16000;

static void save_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open("chat", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "ssid", ssid);
        nvs_set_str(h, "pass", pass);
        nvs_set_str(h, "uri", uri);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void load_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open("chat", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t n = sizeof(ssid);
    nvs_get_str(h, "ssid", ssid, &n);
    n = sizeof(pass);
    nvs_get_str(h, "pass", pass, &n);
    n = sizeof(uri);
    nvs_get_str(h, "uri", uri, &n);
    nvs_close(h);
}

static void play_reply(void)
{
    if (!spk || !reply_len) {
        return;
    }
    esp_codec_dev_sample_info_t fs = {.sample_rate = reply_rate, .channel = 1, .bits_per_sample = 16};
    if (esp_codec_dev_open(spk, &fs) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "speaker open failed");
        return;
    }
    esp_codec_dev_set_out_mute(spk, false);
    esp_codec_dev_set_out_vol(spk, SPK_VOLUME);
    ui_chat_status("speaking");
    esp_codec_dev_write(spk, reply, reply_len);
    esp_codec_dev_close(spk);
    reply_len = 0;
    ui_chat_status("ready - hold to talk");
}

static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_websocket_event_data_t *e = data;
    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "server connected");
        ui_chat_status("ready - hold to talk");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ui_chat_status("server offline");
        break;
    case WEBSOCKET_EVENT_DATA:
        if (e->op_code == 0x01) { // text
            if (e->data_len == 4 && !memcmp(e->data_ptr, "done", 4)) {
                play_reply();
            } else {
                cJSON *j = cJSON_ParseWithLength(e->data_ptr, e->data_len);
                if (j) {
                    cJSON *you = cJSON_GetObjectItem(j, "you"), *ai = cJSON_GetObjectItem(j, "ai");
                    cJSON *rate = cJSON_GetObjectItem(j, "rate");
                    reply_rate = cJSON_IsNumber(rate) ? rate->valueint : 16000;
                    reply_len = 0;
                    ui_chat_text(cJSON_IsString(you) ? you->valuestring : "", cJSON_IsString(ai) ? ai->valuestring : "");
                    cJSON_Delete(j);
                }
            }
        } else if (e->op_code == 0x02 && e->data_len > 0 && reply) { // binary audio
            size_t room = MAX_REPLY_BYTES - reply_len;
            size_t n = e->data_len < room ? e->data_len : room;
            memcpy(reply + reply_len, e->data_ptr, n);
            reply_len += n;
        }
        break;
    }
}

static void ws_connect(void)
{
    if (ws) {
        esp_websocket_client_destroy(ws);
        ws = NULL;
    }
    if (!uri[0]) {
        return;
    }
    esp_websocket_client_config_t cfg = {
        .uri = uri,
        .buffer_size = 4096,
        .reconnect_timeout_ms = 3000,
        .network_timeout_ms = 10000,
    };
    ws = esp_websocket_client_init(&cfg);
    esp_websocket_register_events(ws, WEBSOCKET_EVENT_ANY, ws_event, NULL);
    esp_websocket_client_start(ws);
    ui_chat_status("connecting to server");
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_up = false;
        ui_chat_status("wifi connecting");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        wifi_up = true;
        ESP_LOGI(TAG, "wifi up");
        ws_connect();
    }
}

static void wifi_apply(void)
{
    if (!ssid[0]) {
        ui_chat_status("not set up - see BLE config");
        return;
    }
    wifi_config_t c = {0};
    strlcpy((char *)c.sta.ssid, ssid, sizeof(c.sta.ssid));
    strlcpy((char *)c.sta.password, pass, sizeof(c.sta.password));
    c.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &c);
    esp_wifi_connect();
    ui_chat_status("wifi connecting");
}

void app_set_chat_config(const char *cfg)
{
    char buf[256];
    strlcpy(buf, cfg, sizeof(buf));
    char *a = buf, *b = strchr(a, '|');
    char *c = b ? strchr(b + 1, '|') : NULL;
    if (!b || !c) {
        ui_chat_status("bad config format");
        return;
    }
    *b++ = '\0';
    *c++ = '\0';
    strlcpy(ssid, a, sizeof(ssid));
    strlcpy(pass, b, sizeof(pass));
    strlcpy(uri, c, sizeof(uri));
    save_cfg();
    ESP_LOGI(TAG, "config saved: ssid=%s uri=%s", ssid, uri);
    if (wifi_up) {
        ws_connect();
    }
    wifi_apply();
}

static void record_task(void *arg)
{
    esp_codec_dev_sample_info_t fs = {.sample_rate = MIC_RATE, .channel = 1, .bits_per_sample = 16};
    if (esp_codec_dev_open(mic, &fs) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "mic open failed");
        ui_chat_status("mic error");
        rec_task = NULL;
        vTaskDelete(NULL);
    }
    esp_codec_dev_set_in_gain(mic, MIC_GAIN_DB);
    static int16_t buf[512];
    int64_t sum = 0;
    size_t samples = 0;
    int chunks = 0;
    while (talking && chunks * 512 * 1000 / MIC_RATE < MAX_RECORD_MS) {
        if (esp_codec_dev_read(mic, buf, sizeof(buf)) != ESP_CODEC_DEV_OK) {
            break;
        }
        for (int i = 0; i < 512; i++) {
            sum += (int32_t)buf[i] * buf[i];
        }
        samples += 512;
        chunks++;
        esp_websocket_client_send_bin(ws, (const char *)buf, sizeof(buf), pdMS_TO_TICKS(1000));
    }
    esp_codec_dev_close(mic);
    ESP_LOGI(TAG, "recorded %u samples, rms %d", (unsigned)samples, samples ? (int)__builtin_sqrt((double)sum / samples) : 0);
    esp_websocket_client_send_text(ws, "end", 3, pdMS_TO_TICKS(1000));
    ui_chat_status("thinking");
    rec_task = NULL;
    vTaskDelete(NULL);
}

void chat_talk_start(void)
{
    if (!ws || !esp_websocket_client_is_connected(ws) || rec_task || !mic) {
        ui_chat_status(!mic ? "mic unavailable" : "server not connected");
        return;
    }
    talking = true;
    ui_chat_status("listening...");
    xTaskCreate(record_task, "rec", 6144, NULL, 5, &rec_task);
}

void chat_talk_stop(void)
{
    talking = false;
}

void chat_init(void)
{
    reply = heap_caps_malloc(MAX_REPLY_BYTES, MALLOC_CAP_SPIRAM);
    spk = bsp_audio_codec_speaker_init();
    mic = bsp_audio_codec_microphone_init();
    ESP_LOGI(TAG, "audio: spk=%p mic=%p reply buf=%p", spk, mic, reply);

    load_cfg();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t w = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&w));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    wifi_inited = true;
    wifi_apply();
}

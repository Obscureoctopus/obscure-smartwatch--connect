#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_store.h"
void ble_store_config_init(void);
#include "app.h"

static const char *TAG = "ble";
/* Gadgetbridge treats this name as a PineTime and forwards notifications. */
#define DEVICE_NAME "InfiniTime"

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t batt_handle;
static uint16_t tx_handle;
static uint8_t own_addr_type;
static uint8_t batt_level = 100;

static void advertise(void);

static int info_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const char *s = arg;
    return os_mbuf_append(ctxt->om, s, strlen(s)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int batt_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return os_mbuf_append(ctxt->om, &batt_level, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int time_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t d[10];
        uint16_t len = 0;
        if (OS_MBUF_PKTLEN(ctxt->om) < 7 || ble_hs_mbuf_to_flat(ctxt->om, d, sizeof(d), &len) != 0 || len < 7) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        app_set_time(d[0] | (d[1] << 8), d[2], d[3], d[4], d[5], d[6]);
        return 0;
    }
    uint8_t d[10] = {0};
    return os_mbuf_append(ctxt->om, d, sizeof(d)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int alert_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    uint8_t d[200];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, d, sizeof(d) - 1, &len) != 0 || len < 3) {
        return 0;
    }
    char text[200];
    size_t n = 0;
    for (uint16_t i = 2; i < len && n < sizeof(text) - 1; i++) {
        text[n++] = d[i] == 0 ? '\n' : (char)d[i];
    }
    text[n] = '\0';
    app_on_notification(text);
    return 0;
}

static const ble_uuid128_t cfg_svc_uuid = BLE_UUID128_INIT(0x01, 0x00, 0x57, 0x53, 0x45, 0x48, 0x43, 0x54,
                                                           0x41, 0x57, 0x33, 0x53, 0x00, 0x00, 0x00, 0xa1);
static const ble_uuid128_t cfg_chr_uuid = BLE_UUID128_INIT(0x02, 0x00, 0x57, 0x53, 0x45, 0x48, 0x43, 0x54,
                                                           0x41, 0x57, 0x33, 0x53, 0x00, 0x00, 0x00, 0xa1);
static const ble_uuid128_t speed_chr_uuid = BLE_UUID128_INIT(0x03, 0x00, 0x57, 0x53, 0x45, 0x48, 0x43, 0x54,
                                                             0x41, 0x57, 0x33, 0x53, 0x00, 0x00, 0x00, 0xa1);
static const ble_uuid128_t phone_chr_uuid = BLE_UUID128_INIT(0x04, 0x00, 0x57, 0x53, 0x45, 0x48, 0x43, 0x54,
                                                             0x41, 0x57, 0x33, 0x53, 0x00, 0x00, 0x00, 0xa1);
static const ble_uuid128_t watch_chr_uuid = BLE_UUID128_INIT(0x05, 0x00, 0x57, 0x53, 0x45, 0x48, 0x43, 0x54,
                                                             0x41, 0x57, 0x33, 0x53, 0x00, 0x00, 0x00, 0xa1);

static int cfg_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char d[256];
    uint16_t len = 0;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || OS_MBUF_PKTLEN(ctxt->om) >= sizeof(d) ||
        ble_hs_mbuf_to_flat(ctxt->om, d, sizeof(d) - 1, &len) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    d[len] = '\0';
    app_set_chat_config(d);
    return 0;
}

static int speed_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t d[8];
    uint16_t len = 0;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || ble_hs_mbuf_to_flat(ctxt->om, d, sizeof(d), &len) != 0 ||
        len < 2) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    uint16_t heading = len >= 4 ? (uint16_t)(d[2] | (d[3] << 8)) : 0xFFFF;
    app_set_speed((int16_t)(d[0] | (d[1] << 8)), heading);
    return 0;
}

static void split2(const uint8_t *d, uint16_t len, char *a, size_t as, char *b, size_t bs)
{
    size_t i = 0, n = 0;
    while (i < len && d[i] && n + 1 < as) {
        a[n++] = (char)d[i++];
    }
    a[n] = '\0';
    if (i < len && d[i] == 0) {
        i++;
    }
    n = 0;
    while (i < len && n + 1 < bs) {
        b[n++] = (char)d[i++];
    }
    b[n] = '\0';
}

static int phone_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t d[220];
    uint16_t len = 0;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR ||
        ble_hs_mbuf_to_flat(ctxt->om, d, sizeof(d) - 1, &len) != 0 || len < 1) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    d[len] = 0;
    char a[96], b[140];
    switch (d[0]) {
    case 1: /* notify from\0text */
        split2(d + 1, (uint16_t)(len - 1), a, sizeof(a), b, sizeof(b));
        {
            char line[240];
            snprintf(line, sizeof(line), "%.90s\n%.140s", a[0] ? a : "Phone", b);
            app_on_notification(line);
        }
        break;
    case 2: /* call: 1=ring 0=end, then name */
        if (len >= 2) {
            split2(d + 2, (uint16_t)(len - 2), a, sizeof(a), b, sizeof(b));
            app_on_call(d[1] != 0, a);
        }
        break;
    case 3: /* nav on: dir, dist\0text */
        if (len >= 2) {
            split2(d + 2, (uint16_t)(len - 2), a, sizeof(a), b, sizeof(b));
            app_on_nav(true, d[1], a, b);
        }
        break;
    case 4: /* media: playing, title\0artist */
        if (len >= 2) {
            split2(d + 2, (uint16_t)(len - 2), a, sizeof(a), b, sizeof(b));
            app_on_media(d[1] != 0, a, b);
        }
        break;
    case 5: /* vitals: hr spo2 temp10le stress */
        if (len >= 6) {
            app_on_vitals(d[1], d[2], d[3] | (d[4] << 8), d[5]);
        }
        break;
    case 6:
        app_on_nav(false, 0, "", "");
        break;
    case 7: /* AI lines you\0ai */
        split2(d + 1, (uint16_t)(len - 1), a, sizeof(a), b, sizeof(b));
        app_on_ai_text(a, b);
        break;
    case 8: /* inject mod: type, name\0payload */
        if (len >= 2) {
            split2(d + 2, (uint16_t)(len - 2), a, sizeof(a), b, sizeof(b));
            app_on_mod_inject(d[1], a, b);
        }
        break;
    case 9:
        if (len >= 2) {
            app_on_mod_purge(d[1]);
        }
        break;
    case 10:
        app_on_mod_purge_all();
        break;
    default:
        break;
    }
    return 0;
}

static int watch_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t z = 0;
    return os_mbuf_append(ctxt->om, &z, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &cfg_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = &cfg_chr_uuid.u, .access_cb = cfg_access, .flags = BLE_GATT_CHR_F_WRITE},
            {.uuid = &speed_chr_uuid.u, .access_cb = speed_access,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
            {.uuid = &phone_chr_uuid.u, .access_cb = phone_access,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
            {.uuid = &watch_chr_uuid.u, .access_cb = watch_access, .val_handle = &tx_handle,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = BLE_UUID16_DECLARE(0x2A29), .access_cb = info_access, .arg = "Waveshare", .flags = BLE_GATT_CHR_F_READ},
            {.uuid = BLE_UUID16_DECLARE(0x2A24), .access_cb = info_access, .arg = "Octopus", .flags = BLE_GATT_CHR_F_READ},
            {.uuid = BLE_UUID16_DECLARE(0x2A26), .access_cb = info_access, .arg = "2.4.0", .flags = BLE_GATT_CHR_F_READ},
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1805),
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = BLE_UUID16_DECLARE(0x2A2B), .access_cb = time_access,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1811),
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = BLE_UUID16_DECLARE(0x2A46), .access_cb = alert_access,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = BLE_UUID16_DECLARE(0x2A19), .access_cb = batt_access, .val_handle = &batt_handle,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
            {0},
        },
    },
    {0},
};

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            app_on_connection(true);
        } else {
            advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        app_on_connection(false);
        advertise();
        break;
    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "encryption change status=%d", event->enc_change.status);
        break;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc d;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &d) == 0) {
            ble_store_util_delete_peer(&d.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;
    }
    return 0;
}

static void advertise(void)
{
    static const ble_uuid16_t uuids[] = {BLE_UUID16_INIT(0x1811)};
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.uuids16 = uuids;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv start failed: %d", rc);
    }
}

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    advertise();
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_set_battery(uint8_t percent)
{
    if (percent == batt_level) {
        return;
    }
    batt_level = percent;
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gatts_chr_updated(batt_handle);
    }
}

void ble_send(const uint8_t *data, uint16_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !data || !len) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) {
        ble_gatts_notify_custom(conn_handle, tx_handle, om);
    }
}

void ble_start(void)
{
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));
    ble_store_config_init();
    nimble_port_freertos_init(host_task);
}

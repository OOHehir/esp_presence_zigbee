#include "zigbee_node.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "zigbee_node";

/* ── Endpoint and cluster IDs ────────────────────────────────── */
#define ENDPOINT_ID          1
#define CUSTOM_CLUSTER_LD    0xFC00
#define CUSTOM_CLUSTER_VL    0xFC01
#define MANUF_CODE           0x1234   /* placeholder — replace before production */

/* ── Custom cluster attribute IDs ────────────────────────────── */
/* 0xFC00 — LD2410C */
#define ATTR_LD_MOVING_TARGET    0x0001
#define ATTR_LD_STATIONARY_TARGET 0x0002
#define ATTR_LD_MOVE_ENERGY      0x0003
#define ATTR_LD_STATIC_ENERGY    0x0004
#define ATTR_LD_TARGET_DIST      0x0005

/* 0xFC01 — VL53L0X */
#define ATTR_VL_RANGE_MM         0x0001
#define ATTR_VL_RANGE_STATUS     0x0002

/* ── Rejoin backoff ──────────────────────────────────────────── */
#define REJOIN_INITIAL_MS    5000
#define REJOIN_MAX_MS        300000  /* 5 minutes */
static uint32_t s_rejoin_delay_ms = REJOIN_INITIAL_MS;

/* ── Forward declarations ────────────────────────────────────── */
static void configure_reporting(void);

/* ── Signal handler (called by Zigbee stack) ─────────────────── */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *sig = signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *sig;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialised");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "Device %s — starting network steering",
                     sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? "first start" : "reboot");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Zigbee init signal failed (status %d)", status);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network — PAN ID: 0x%04x, short addr: 0x%04x, channel: %d",
                     esp_zb_get_pan_id(),
                     esp_zb_get_short_address(),
                     esp_zb_get_current_channel());
            s_rejoin_delay_ms = REJOIN_INITIAL_MS;
            configure_reporting();
        } else {
            ESP_LOGW(TAG, "Network steering failed (0x%x), retry in %lu ms",
                     status, (unsigned long)s_rejoin_delay_ms);
            vTaskDelay(pdMS_TO_TICKS(s_rejoin_delay_ms));
            if (s_rejoin_delay_ms < REJOIN_MAX_MS) {
                s_rejoin_delay_ms *= 2;
                if (s_rejoin_delay_ms > REJOIN_MAX_MS) {
                    s_rejoin_delay_ms = REJOIN_MAX_MS;
                }
            }
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "Left network — attempting rejoin");
        s_rejoin_delay_ms = REJOIN_INITIAL_MS;
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    default:
        ESP_LOGD(TAG, "ZDO signal: %d, status: %d", sig_type, status);
        break;
    }
}

/* ── Action handler ──────────────────────────────────────────── */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message)
{
    /* Handle factory reset via serial command check in a separate mechanism.
     * No attribute write handlers needed — this device is report-only. */
    return ESP_OK;
}

/* ── Reporting configuration ─────────────────────────────────── */
static void setup_report(uint16_t cluster_id, uint16_t attr_id, uint16_t manuf_code)
{
    esp_zb_zcl_attr_location_info_t attr_info = {
        .endpoint_id = ENDPOINT_ID,
        .cluster_id = cluster_id,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .manuf_code = manuf_code,
        .attr_id = attr_id,
    };

    esp_err_t ret = esp_zb_zcl_start_attr_reporting(attr_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start reporting for cluster 0x%04x attr 0x%04x: %s",
                 cluster_id, attr_id, esp_err_to_name(ret));
    }

    /* Update reporting intervals */
    esp_zb_zcl_reporting_info_t *report = esp_zb_zcl_find_reporting_info(attr_info);
    if (report) {
        report->u.send_info.min_interval = 5;
        report->u.send_info.max_interval = 5;
        esp_zb_zcl_update_reporting_info(report);
    }
}

static void configure_reporting(void)
{
    ESP_LOGI(TAG, "Configuring attribute reporting (5s interval)");

    /* Occupancy cluster — standard, no manufacturer code */
    setup_report(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                 ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, 0);

    /* Custom cluster 0xFC00 — LD2410C attributes */
    setup_report(CUSTOM_CLUSTER_LD, ATTR_LD_MOVING_TARGET, MANUF_CODE);
    setup_report(CUSTOM_CLUSTER_LD, ATTR_LD_STATIONARY_TARGET, MANUF_CODE);
    setup_report(CUSTOM_CLUSTER_LD, ATTR_LD_MOVE_ENERGY, MANUF_CODE);
    setup_report(CUSTOM_CLUSTER_LD, ATTR_LD_STATIC_ENERGY, MANUF_CODE);
    setup_report(CUSTOM_CLUSTER_LD, ATTR_LD_TARGET_DIST, MANUF_CODE);

    /* Custom cluster 0xFC01 — VL53L0X attributes */
    setup_report(CUSTOM_CLUSTER_VL, ATTR_VL_RANGE_MM, MANUF_CODE);
    setup_report(CUSTOM_CLUSTER_VL, ATTR_VL_RANGE_STATUS, MANUF_CODE);
}

/* ── Device registration ─────────────────────────────────────── */
static void register_device(void)
{
    /* --- Basic cluster --- */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .power_source = 0x01, /* mains */
    };
    esp_zb_attribute_list_t *basic_attrs = esp_zb_basic_cluster_create(&basic_cfg);

    char manufacturer[] = "Rufilla";
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    char model[] = "presence-node-v1";
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);

    /* --- Identify cluster --- */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *identify_attrs = esp_zb_identify_cluster_create(&identify_cfg);

    /* --- Occupancy Sensing cluster (0x0406) --- */
    esp_zb_occupancy_sensing_cluster_cfg_t occ_cfg = {
        .occupancy = 0,
        .sensor_type = 0x03, /* other/physical contact — closest for mmWave */
        .sensor_type_bitmap = (1 << 3), /* bit 3 = other */
    };
    esp_zb_attribute_list_t *occ_attrs = esp_zb_occupancy_sensing_cluster_create(&occ_cfg);

    /* --- Custom cluster 0xFC00: LD2410C rich data --- */
    esp_zb_attribute_list_t *ld_attrs = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_LD);
    bool     bool_val   = false;
    uint8_t  u8_val     = 0;
    uint16_t u16_val    = 0;

    esp_zb_custom_cluster_add_custom_attr(ld_attrs, ATTR_LD_MOVING_TARGET,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &bool_val);
    esp_zb_custom_cluster_add_custom_attr(ld_attrs, ATTR_LD_STATIONARY_TARGET,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &bool_val);
    esp_zb_custom_cluster_add_custom_attr(ld_attrs, ATTR_LD_MOVE_ENERGY,
        ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &u8_val);
    esp_zb_custom_cluster_add_custom_attr(ld_attrs, ATTR_LD_STATIC_ENERGY,
        ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &u8_val);
    esp_zb_custom_cluster_add_custom_attr(ld_attrs, ATTR_LD_TARGET_DIST,
        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &u16_val);

    /* --- Custom cluster 0xFC01: VL53L0X ranging data --- */
    esp_zb_attribute_list_t *vl_attrs = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_VL);

    esp_zb_custom_cluster_add_custom_attr(vl_attrs, ATTR_VL_RANGE_MM,
        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &u16_val);
    esp_zb_custom_cluster_add_custom_attr(vl_attrs, ATTR_VL_RANGE_STATUS,
        ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &u8_val);

    /* --- Build cluster list --- */
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, occ_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, ld_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, vl_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* --- Build endpoint --- */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint       = ENDPOINT_ID,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);

    /* --- Register --- */
    esp_zb_device_register(ep_list);
    ESP_LOGI(TAG, "Zigbee device registered (endpoint %d)", ENDPOINT_ID);
}

/* ── Public API ──────────────────────────────────────────────── */

esp_err_t zigbee_node_init(void)
{
    esp_zb_platform_config_t platform_cfg = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));
    ESP_LOGI(TAG, "Zigbee platform configured");
    return ESP_OK;
}

void zigbee_node_start(void)
{
    /* Router configuration */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {
            .max_children = 10,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    register_device();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Zigbee stack started — entering main loop");
    esp_zb_stack_main_loop();
}

esp_err_t zigbee_node_update_ld2410c(const ld2410c_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;

    esp_zb_lock_acquire(portMAX_DELAY);

    /* Update occupancy (0x0406) — occupied if any target detected */
    uint8_t occupancy = (data->moving_target || data->stationary_target) ? 1 : 0;
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID,
        ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
        &occupancy, false);

    /* Update custom cluster 0xFC00 attributes */
    bool moving = data->moving_target;
    bool stationary = data->stationary_target;
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, CUSTOM_CLUSTER_LD,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LD_MOVING_TARGET, &moving, false);
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, CUSTOM_CLUSTER_LD,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LD_STATIONARY_TARGET, &stationary, false);
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, CUSTOM_CLUSTER_LD,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LD_MOVE_ENERGY,
        (void *)&data->move_energy, false);
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, CUSTOM_CLUSTER_LD,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LD_STATIC_ENERGY,
        (void *)&data->static_energy, false);
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, CUSTOM_CLUSTER_LD,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LD_TARGET_DIST,
        (void *)&data->target_distance_cm, false);

    esp_zb_lock_release();
    return ESP_OK;
}

esp_err_t zigbee_node_update_vl53l0x(const vl53l0x_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;

    esp_zb_lock_acquire(portMAX_DELAY);

    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, CUSTOM_CLUSTER_VL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_VL_RANGE_MM,
        (void *)&data->range_mm, false);
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, CUSTOM_CLUSTER_VL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_VL_RANGE_STATUS,
        (void *)&data->status, false);

    esp_zb_lock_release();
    return ESP_OK;
}

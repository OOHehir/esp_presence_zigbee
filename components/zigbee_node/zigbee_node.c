#include "zigbee_node.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zigbee_node";

#define ENDPOINT_ID           1
#define ANALOG_IN_CLUSTER     0x000C
#define ATTR_PRESENT_VALUE    0x0055

/* ── Rejoin backoff ──────────────────────────────────────────── */
#define REJOIN_INITIAL_MS    5000
#define REJOIN_MAX_MS        300000
static uint32_t s_rejoin_delay_ms = REJOIN_INITIAL_MS;
static TaskFunction_t s_sensor_task_fn = NULL;
static bool s_sensor_task_started = false;

/* ── Signal handler ──────────────────────────────────────────── */
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
            ESP_LOGW(TAG, "Zigbee init failed (status %d)", status);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network — PAN 0x%04x, addr 0x%04x, ch %d",
                     esp_zb_get_pan_id(), esp_zb_get_short_address(),
                     esp_zb_get_current_channel());
            s_rejoin_delay_ms = REJOIN_INITIAL_MS;
            if (s_sensor_task_fn && !s_sensor_task_started) {
                xTaskCreate(s_sensor_task_fn, "sensor_report", 4096, NULL, 5, NULL);
                s_sensor_task_started = true;
            }
        } else {
            ESP_LOGW(TAG, "Steering failed (0x%x), retry in %lu ms",
                     status, (unsigned long)s_rejoin_delay_ms);
            vTaskDelay(pdMS_TO_TICKS(s_rejoin_delay_ms));
            if (s_rejoin_delay_ms < REJOIN_MAX_MS) {
                s_rejoin_delay_ms *= 2;
                if (s_rejoin_delay_ms > REJOIN_MAX_MS)
                    s_rejoin_delay_ms = REJOIN_MAX_MS;
            }
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "Left network — rejoining");
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
    return ESP_OK;
}

/* ── Public API ──────────────────────────────────────────────── */

esp_err_t zigbee_node_init(void)
{
    esp_zb_platform_config_t platform_cfg = {
        .radio_config  = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config   = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));
    ESP_LOGI(TAG, "Zigbee platform configured");
    return ESP_OK;
}

void zigbee_node_start(TaskFunction_t sensor_task_fn)
{
    s_sensor_task_fn = sensor_task_fn;

    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout  = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive  = 3000,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    /* Single Analog Input cluster — identical to working sound sensor */
    esp_zb_analog_input_cluster_cfg_t ai_cfg = { .present_value = 0.0f };
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_analog_input_cluster(
        cluster_list,
        esp_zb_analog_input_cluster_create(&ai_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Basic + Identify for device identification */
    esp_zb_basic_cluster_cfg_t basic_cfg = { .power_source = 0x01 };
    esp_zb_attribute_list_t *basic_attrs = esp_zb_basic_cluster_create(&basic_cfg);
    static char manufacturer[] = {7, 'R', 'u', 'f', 'i', 'l', 'l', 'a'};
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    static char model[] = {16, 'p', 'r', 'e', 's', 'e', 'n', 'c', 'e', '-', 'n', 'o', 'd', 'e', '-', 'v', '1'};
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cluster_list,
        esp_zb_identify_cluster_create(&identify_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint       = ENDPOINT_ID,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);
    esp_zb_device_register(ep_list);
    ESP_LOGI(TAG, "Device registered (endpoint %d, Analog Input)", ENDPOINT_ID);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(1 << 15);

    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Zigbee stack started — entering main loop");
    esp_zb_stack_main_loop();
}

esp_err_t zigbee_node_update_ld2410c(const ld2410c_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;

    /* Pack presence as a simple float: 1.0 = present, 0.0 = not present */
    float presence = (data->moving_target || data->stationary_target) ? 1.0f : 0.0f;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID,
        ANALOG_IN_CLUSTER, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ATTR_PRESENT_VALUE, &presence, false);
    esp_zb_lock_release();

    return ESP_OK;
}

esp_err_t zigbee_node_update_vl53l0x(const vl53l0x_data_t *data)
{
    /* Placeholder — will add second endpoint later */
    (void)data;
    return ESP_OK;
}

#include "zigbee_node.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zigbee_node";

#define EP_PRESENCE           1
#define EP_RANGE              2
#define EP_RANGE_STATUS       3
#define ANALOG_IN_CLUSTER     0x000C
#define ATTR_PRESENT_VALUE    0x0055

/* ── Rejoin backoff ──────────────────────────────────────────── */
#define REJOIN_INITIAL_MS    5000
#define REJOIN_MAX_MS        300000
static uint32_t s_rejoin_delay_ms = REJOIN_INITIAL_MS;
static TaskFunction_t s_sensor_task_fn = NULL;
static bool s_sensor_task_started = false;
static volatile bool s_network_joined = false;

/* ── Signal handler ──────────────────────────────────────────── */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *sig = signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *sig;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialised");
        if (s_sensor_task_fn && !s_sensor_task_started) {
            xTaskCreate(s_sensor_task_fn, "sensor_report", 4096, NULL, 5, NULL);
            s_sensor_task_started = true;
        }
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
            s_network_joined = true;
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
        s_network_joined = false;
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

    esp_zb_analog_input_cluster_cfg_t ai_cfg = { .present_value = 0.0f };

    /* ── Endpoint 1: LD2410C presence (0.0 / 1.0) ─────────────── */
    esp_zb_cluster_list_t *cl_presence = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_analog_input_cluster(
        cl_presence,
        esp_zb_analog_input_cluster_create(&ai_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Basic + Identify on endpoint 1 for device identification */
    esp_zb_basic_cluster_cfg_t basic_cfg = { .power_source = 0x01 };
    esp_zb_attribute_list_t *basic_attrs = esp_zb_basic_cluster_create(&basic_cfg);
    static char manufacturer[] = {7, 'R', 'u', 'f', 'i', 'l', 'l', 'a'};
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    static char model[] = {16, 'p', 'r', 'e', 's', 'e', 'n', 'c', 'e', '-', 'n', 'o', 'd', 'e', '-', 'v', '1'};
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
    esp_zb_cluster_list_add_basic_cluster(cl_presence, basic_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl_presence,
        esp_zb_identify_cluster_create(&identify_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* ── Endpoint 2: VL53L0X range (cm as float) ─────────────── */
    esp_zb_cluster_list_t *cl_range = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_analog_input_cluster(
        cl_range,
        esp_zb_analog_input_cluster_create(&ai_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* ── Endpoint 3: VL53L0X range status (0=valid, 255=no target) */
    esp_zb_cluster_list_t *cl_status = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_analog_input_cluster(
        cl_status,
        esp_zb_analog_input_cluster_create(&ai_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* ── Register all endpoints ────────────────────────────────── */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t ep1_cfg = {
        .endpoint       = EP_PRESENCE,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(ep_list, cl_presence, ep1_cfg);

    esp_zb_endpoint_config_t ep2_cfg = {
        .endpoint       = EP_RANGE,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(ep_list, cl_range, ep2_cfg);

    esp_zb_endpoint_config_t ep3_cfg = {
        .endpoint       = EP_RANGE_STATUS,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(ep_list, cl_status, ep3_cfg);

    esp_zb_device_register(ep_list);
    ESP_LOGI(TAG, "Device registered (ep %d=presence, ep %d=range, ep %d=range_status)",
             EP_PRESENCE, EP_RANGE, EP_RANGE_STATUS);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(1 << 15);

    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Zigbee stack started — entering main loop");
    esp_zb_stack_main_loop();
}

/* Ceiling baseline distance threshold (cm). Stationary targets at or beyond
 * this distance are assumed to be the ceiling/wall, not a person. Movement
 * detection is always trusted regardless of distance. */
#define CEILING_BASELINE_CM  185

esp_err_t zigbee_node_update_ld2410c(const ld2410c_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!s_network_joined) return ESP_OK;

    /* Movement always counts as presence. Stationary only counts if the
     * target distance is closer than the ceiling baseline — otherwise it's
     * just the ceiling/wall being detected as a permanent reflector. */
    bool present = data->moving_target ||
                   (data->stationary_target && data->target_distance_cm < CEILING_BASELINE_CM);

    float presence = present ? 1.0f : 0.0f;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(EP_PRESENCE,
        ANALOG_IN_CLUSTER, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ATTR_PRESENT_VALUE, &presence, false);
    esp_zb_lock_release();

    return ESP_OK;
}

esp_err_t zigbee_node_update_vl53l0x(const vl53l0x_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!s_network_joined) return ESP_OK;

    float range = (float)data->range_cm;
    float status = (float)data->status;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(EP_RANGE,
        ANALOG_IN_CLUSTER, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ATTR_PRESENT_VALUE, &range, false);
    esp_zb_zcl_set_attribute_val(EP_RANGE_STATUS,
        ANALOG_IN_CLUSTER, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ATTR_PRESENT_VALUE, &status, false);
    esp_zb_lock_release();

    return ESP_OK;
}

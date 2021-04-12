// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"
#include "mesh_mqtt_handle.h"
#include "mwifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "CJSON.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"

#define MEMORY_DEBUG

#define SSID_WIFI "MVP_GUEST"
#define PASS_WIFI ""
#define MESH_ID "123456"
#define MESH_PASS "0123456789"
#define MQTT_BROKER_URL "mqtt://mqtt.eclipseprojects.io"
#define NODE 1

#define FIRMWARE_VERSION "V0"
#define HARDWARE_VERSION "V2"
#define SERIAL_NUMBER "0123456789"
#define MANUFACTURER "MVP"

#define EX_UART_NUM UART_NUM_2
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define UART2_RX_PIN 16
#define UART2_TX_PIN 17

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

typedef struct {
	char id[32];
	char name[32];
	uint16_t layer;
	bool expired;
} manager_infor_t;

typedef struct {
	char firmware[32];
	char hardware[32];
	char serial[32];
	char company[32];
} device_infor_t;

typedef struct {
	char ssid[32];
	char password[32];
	char id_mesh[32];
	char pass_mesh[32];
	char mqtt_broker[32];
	char mqtt_topic[32];
	char mqtt_port[8];
} network_infor_t;

typedef enum {
	UPDATE_DATA_EVT = 1, SETTING_EVT = 2, PING_EVT = 3
} type_event_data_t;

typedef struct {
	type_event_data_t type;
	char *data;
	uint16_t lenght;
} type_data_communicate_t;

manager_infor_t manager_infor;
device_infor_t device_infor;
network_infor_t network_infor;
static QueueHandle_t queue_data_communicate;
mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
mwifi_config_t mesh_config;

char id_node_default[8] = "000000";
char name_node_default[16] = "CT-0000";
char mqtt_broker_default[32] = "mqtt.eclipseprojects.io";
char mqtt_topic_default[32] = "MESH";

/*Infor network */
char ssid_wifi_default[32] = "myLab";
char pass_wifi_default[32] = "12345678@Ab";
char id_mesh_network_default[8] = "IFS-CT";
char pass_mesh_network_default[32] = "0123456789";
char type[8]; // root or node
char mqtt_server[32];
char mqtt_port[8];
char mqtt_api[32];

/*Infor Manafacture Device */
char hardware_version_node[8] = "V1.0";
char firmware_version_node[8] = "V1.1";
char serial_number_node[26] = "0123456789";
char company[8] = "MVP";

static const char *TAG = "Node_mesh";

char*
generate_JSON(char *param);

type_data_communicate_t parse_data_uart(const char *data_uart) {
	static type_data_communicate_t value;
	value.data = calloc(strlen(data_uart), sizeof(char));
	sscanf(data_uart, "%d;%s", (int*) &value.type, value.data);
	value.lenght = strlen(value.data);
	return value;
}

void restart_to_configure_program(void) {

	const esp_partition_t *ota_partition = esp_partition_find_first(
			ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY,
			NULL);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	if (!ota_partition) {
		ESP_LOGE(TAG, "FATAL ERROR: No NVS partition found\n");
		return;
	}
	esp_err_t err = esp_ota_set_boot_partition(ota_partition);
	if (err != ESP_OK) {
	}
	vTaskDelay(100 / portTICK_PERIOD_MS);
	esp_restart();

}

void init_spiffs(void) {

	ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = { .base_path = "/spiffs", .partition_label =
	NULL, .max_files = 5, .format_if_mount_failed = true };

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",
					esp_err_to_name(ret));
		}
		return;
	}
	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)",
				esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
	}
}

void get_infor_configure(void) {
	// Init SPIFFS before get infor configure
	init_spiffs();
	FILE *f;
	// Open renamed file for reading
	ESP_LOGI(TAG, "Reading file");
	f = fopen("/spiffs/config.json", "r");
	if (f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for reading");
		return;
	}
	char data[512];
	fgets(data, sizeof(data), f);
	fclose(f);
	// strip newline
	char *pos = strchr(data, '\n');
	if (pos) {
		*pos = '\0';
	}
	ESP_LOGI(TAG, "Read from file: '%s'", data);

	// All done, unmount partition and disable SPIFFS

	cJSON *json_wifi_ssid;
	cJSON *json_wifi_password;
	cJSON *json_mesh_id;
	cJSON *json_mesh_password;
	cJSON *json_type_node;
	cJSON *json_mqtt_server;
	cJSON *json_mqtt_port;
	cJSON *json_mqtt_api;
	cJSON *json_device_name;
	cJSON *json_device_id;
	cJSON *monitor_json = cJSON_Parse(data);
	if (monitor_json == NULL) {
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL) {
			fprintf(stderr, "Error before: %s\n", error_ptr);
		}
		return;
	}
	// Configure WIFI
	json_wifi_ssid = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"wifi_ssid");
	json_wifi_password = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"wifi_password");

	// Configure MESH Network
	json_mesh_id = cJSON_GetObjectItemCaseSensitive(monitor_json, "network_id");
	json_mesh_password = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"network_password");
	json_type_node = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"work_mode");

	// Configure Infor Device
	json_device_name = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"device_name");
	json_device_id = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"device_id");

	// Configure MQTT
	json_mqtt_server = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"mqtt_server");
	json_mqtt_port = cJSON_GetObjectItemCaseSensitive(monitor_json,
			"mqtt_port");
	json_mqtt_api = cJSON_GetObjectItemCaseSensitive(monitor_json, "api_token");

	if (cJSON_IsString(json_wifi_ssid)
			&& (json_wifi_ssid->valuestring != NULL)) {
		ESP_LOGI(TAG, "wifi_ssid: \"%s\"\n", json_wifi_ssid->valuestring);
		strcpy(mesh_config.router_ssid, json_wifi_ssid->valuestring);
	} else {
		strcpy(mesh_config.router_ssid, ssid_wifi_default);
	}

	if (cJSON_IsString(json_wifi_password)
			&& (json_wifi_password->valuestring != NULL)) {
		ESP_LOGI(TAG, "wifi_pass: \"%s\"\n", json_wifi_password->valuestring);
		strcpy(mesh_config.router_password, json_wifi_password->valuestring);
	} else {
		strcpy(mesh_config.router_password, pass_wifi_default);
	}

	if (cJSON_IsString(json_mesh_id) && (json_mesh_id->valuestring != NULL)) {
		ESP_LOGI(TAG, "mesh_id: \"%s\"\n", json_mesh_id->valuestring);
		strcpy((char*) mesh_config.mesh_id, json_mesh_id->valuestring);
	} else {
		strcpy((char*) mesh_config.mesh_id, id_mesh_network_default);
	}

	if (cJSON_IsString(json_mesh_password)
			&& (json_mesh_password->valuestring != NULL)) {
		ESP_LOGI(TAG, "mesh_password: \"%s\"\n",
				json_mesh_password->valuestring);
		strcpy(mesh_config.mesh_password, json_mesh_password->valuestring);
	} else {
		strcpy(mesh_config.mesh_password, pass_mesh_network_default);
	}

	if (cJSON_IsString(json_type_node)
			&& (json_type_node->valuestring != NULL)) {
		ESP_LOGI(TAG, "work_mode: \"%s\"\n", json_type_node->valuestring);
		if (!memcmp(json_type_node->valuestring, "node", 4)) {
			mesh_config.mesh_type = MWIFI_MESH_NODE;
		} else if (!memcmp(json_type_node->valuestring, "root", 4)) {
			mesh_config.mesh_type = MWIFI_MESH_ROOT;
		} else {
			mesh_config.mesh_type = MWIFI_MESH_NODE;
		}

	}

	else {
		mesh_config.mesh_type = MWIFI_MESH_NODE;
	}

	if (cJSON_IsString(json_device_name)
			&& (json_device_name->valuestring != NULL)) {
		ESP_LOGI(TAG, "device_name: \"%s\"\n", json_device_name->valuestring);
		strcpy(manager_infor.name, json_device_name->valuestring);
	} else {
		strcpy(manager_infor.name, name_node_default);
	}

	if (cJSON_IsString(json_device_id)
			&& (json_device_id->valuestring != NULL)) {
		ESP_LOGI(TAG, "device_id: \"%s\"\n", json_device_id->valuestring);
		strcpy(manager_infor.id, json_device_id->valuestring);
	} else {
		strcpy(manager_infor.id, id_node_default);
	}

	if (cJSON_IsString(json_mqtt_server)
			&& (json_mqtt_server->valuestring != NULL)) {
		ESP_LOGI(TAG, "mqtt_server: \"%s\"\n", json_mqtt_server->valuestring);
		strcpy(network_infor.mqtt_broker, json_mqtt_server->valuestring);
	} else {
		strcpy(network_infor.mqtt_broker, mqtt_broker_default);
	}

	if (cJSON_IsString(json_mqtt_api) && (json_mqtt_api->valuestring != NULL)) {
		ESP_LOGI(TAG, "api_token: \"%s\"\n", json_mqtt_api->valuestring);
		strcpy(network_infor.mqtt_topic, json_mqtt_api->valuestring);
	} else {
		strcpy(network_infor.mqtt_topic, mqtt_broker_default);
	}

	esp_vfs_spiffs_unregister(NULL);
}

void root_write_task(void *arg) {
	mdf_err_t ret = MDF_OK;
	char *data = NULL;
	size_t size = MWIFI_PAYLOAD_LEN;
	uint8_t src_addr[MWIFI_ADDR_LEN] = { 0x0 };
	mwifi_data_type_t data_type = { 0x0 };

	MDF_LOGI("Root write task is running");

	while (esp_mesh_is_root()) {
		if (!mwifi_get_root_status()) {
			vTaskDelay(500 / portTICK_RATE_MS);
			continue;
		}

		/**
		 * @brief Recv data from node, and forward to mqtt server.
		 */
		ret = mwifi_root_read(src_addr, &data_type, &data, &size,
				portMAX_DELAY);
		MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_read",
				mdf_err_to_name(ret));

		ret = mesh_mqtt_write(src_addr, data, size, MESH_MQTT_DATA_JSON);

		MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mesh_mqtt_publish",
				mdf_err_to_name(ret));

		MEM_FREE: MDF_FREE(data);
	}

	MDF_LOGW("Root write task is exit");
	mesh_mqtt_stop();
	vTaskDelete(NULL);
}

void root_read_task(void *arg) {
	mdf_err_t ret = MDF_OK;

	MDF_LOGI("Root read task is running");

	while (esp_mesh_is_root()) {
		if (!mwifi_get_root_status()) {
			vTaskDelay(500 / portTICK_RATE_MS);
			continue;
		}

		mesh_mqtt_data_t *request = NULL;
		mwifi_data_type_t data_type = { 0x0 };

		/**
		 * @brief Recv data from mqtt data queue, and forward to special device.
		 */
		ret = mesh_mqtt_read(&request, pdMS_TO_TICKS(500));

		if (ret != MDF_OK) {
			continue;
		}

		ret = mwifi_root_write(request->addrs_list, request->addrs_num,
				&data_type, request->data, request->size, true);
		MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_write",
				mdf_err_to_name(ret));

		MEM_FREE: MDF_FREE(request->addrs_list);
		MDF_FREE(request->data);
		MDF_FREE(request);
	}

	MDF_LOGW("Root read task is exit");
	mesh_mqtt_stop();
	vTaskDelete(NULL);
}

static void node_read_task(void *arg) {
	mdf_err_t ret = MDF_OK;
	char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
	size_t size = MWIFI_PAYLOAD_LEN;
	mwifi_data_type_t data_type = { 0x0 };
	uint8_t src_addr[MWIFI_ADDR_LEN] = { 0x0 };

	MDF_LOGI("Node read task is running");

	for (;;) {
		if (!mwifi_is_connected()) {
			vTaskDelay(500 / portTICK_RATE_MS);
			continue;
		}

		size = MWIFI_PAYLOAD_LEN;
		memset(data, 0, MWIFI_PAYLOAD_LEN);
		ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
		MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read",
				mdf_err_to_name(ret));
	MDF_LOGI("Node receive: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
}

MDF_LOGW("Node read task is exit");
MDF_FREE(data);
vTaskDelete(NULL);
}

static void node_write_task(void *arg) {
mdf_err_t ret = MDF_OK;
size_t size = 0;
char *data = NULL;
mwifi_data_type_t data_type = { 0x0 };
uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
mesh_addr_t parent_mac = { 0 };

MDF_LOGI("Node task is running");

esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

for (;;) {
	if (!mwifi_is_connected() || !mwifi_get_root_status()) {
		vTaskDelay(500 / portTICK_RATE_MS);
		continue;
	}

	/**
	 * @brief Send device information to mqtt server throught root node.
	 */
	esp_mesh_get_parent_bssid(&parent_mac);
	size =
			asprintf(&data,
					"{\"type\":\"heartbeat\", \"self\": \"%02x%02x%02x%02x%02x%02x\", \"parent\":\"%02x%02x%02x%02x%02x%02x\",\"layer\":%d}",
					MAC2STR(sta_mac), MAC2STR(parent_mac.addr),
					esp_mesh_get_layer());

	MDF_LOGD("Node send, size: %d, data: %s", size, data);
	ret = mwifi_write(NULL, &data_type, data, size, true);
	MDF_FREE(data);
	MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));

	vTaskDelay(3000 / portTICK_RATE_MS);
}

MDF_LOGW("Node task is exit");
vTaskDelete(NULL);
}

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(void *timer) {
uint8_t primary = 0;
wifi_second_chan_t second = 0;
mesh_addr_t parent_bssid = { 0 };
uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
wifi_sta_list_t wifi_sta_list = { 0x0 };

esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
esp_wifi_ap_get_sta_list(&wifi_sta_list);
esp_wifi_get_channel(&primary, &second);
esp_mesh_get_parent_bssid(&parent_bssid);

MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
		", parent rssi: %d, node num: %d, free heap: %u",
		primary,
		esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
		mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

for (int i = 0; i < wifi_sta_list.num; i++) {
MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
}

#ifdef MEMORY_DEBUG

if (!heap_caps_check_integrity_all(true)) {
MDF_LOGE("At least one heap is corrupt");
}

//mdf_mem_print_heap();
mdf_mem_print_record();
//mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}

static mdf_err_t wifi_init() {
mdf_err_t ret = nvs_flash_init();
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
MDF_ERROR_ASSERT(nvs_flash_erase());
ret = nvs_flash_init();
}

MDF_ERROR_ASSERT(ret);

tcpip_adapter_init();
MDF_ERROR_ASSERT(esp_event_loop_init( NULL, NULL));
MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
MDF_ERROR_ASSERT(esp_wifi_start());

return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx) {
MDF_LOGI("event_loop_cb, event: %d", event);

switch (event) {
case MDF_EVENT_MWIFI_STARTED:
MDF_LOGI("MESH is started");
break;

case MDF_EVENT_MWIFI_PARENT_CONNECTED:
MDF_LOGI("Parent is connected on station interface");
break;

case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
MDF_LOGI("Parent is disconnected on station interface");

if (esp_mesh_is_root()) {
	mesh_mqtt_stop();
}

break;

case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
MDF_LOGI("MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE, total_num: %d",
		esp_mesh_get_total_node_num());

if (esp_mesh_is_root() && mwifi_get_root_status()) {
	mdf_err_t err = mesh_mqtt_update_topo();

	if (err != MDF_OK) {
		MDF_LOGE("Update topo failed");
	}
}

break;

case MDF_EVENT_MWIFI_ROOT_GOT_IP: {
MDF_LOGI(
		"Root obtains the IP address. It is posted by LwIP stack automatically");

mesh_mqtt_start_topic(network_infor.mqtt_broker, network_infor.mqtt_topic,
NULL);

xTaskCreate(root_write_task, "root_write", 4 * 1024,
NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
xTaskCreate(root_read_task, "root_read", 4 * 1024,
NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

break;
}

case MDF_EVENT_CUSTOM_MQTT_CONNECTED:
MDF_LOGI("MQTT connect");
mdf_err_t err = mesh_mqtt_subscribe();
if (err != MDF_OK) {
	MDF_LOGE("Subscribe failed");
}
err = mesh_mqtt_update_topo();
if (err != MDF_OK) {
	MDF_LOGE("Update topo failed");
}

mwifi_post_root_status(true);
break;

case MDF_EVENT_CUSTOM_MQTT_DISCONNECTED:
MDF_LOGI("MQTT disconnected");
mwifi_post_root_status(false);
break;

default:
break;
}

return MDF_OK;
}

bool send_mqtt_msg(char *param) {
mdf_err_t ret = MDF_OK;
size_t size = 0;
char *data = NULL;
mwifi_data_type_t data_type = { 0x0 };
uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
mesh_addr_t parent_mac = { 0 };

if (param == NULL || strlen(param) < 2) {
return false;
}

if (!mwifi_is_connected() || !mwifi_get_root_status()) {
vTaskDelay(500 / portTICK_RATE_MS);
return false;
}
data = generate_JSON(param);
size = strlen(data);
MDF_LOGI("Node send, size: %d, data: %s", size, data);
ret = mwifi_write(NULL, &data_type, data, size, true);
MDF_FREE(data);
printf("heap MDF_FREE %d \n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
return true;
}

static void uart_event_task(void *pvParameters) {
uart_event_t event;
size_t buffered_size;
uint8_t *dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
type_data_communicate_t communicate_msg;
for (;;) {
//Waiting for UART event.
if (xQueueReceive(uart0_queue, (void*) &event, (portTickType) portMAX_DELAY)) {
	bzero(dtmp, RD_BUF_SIZE);
	ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
	switch (event.type) {
	//Event of UART receving data
	/*We'd better handler data event fast, there would be much more data events than
	 other types of events. If we take too much time on data event, the queue might
	 be full.*/
	case UART_DATA:
		uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
		ESP_LOGI(TAG, "[DATA EVT]:");
		ESP_LOGI(TAG, "[UART DATA]: %s", (const char*) dtmp);
		// Parse data -> assign to queue -> send queue
		communicate_msg = parse_data_uart((const char*) dtmp);
		xQueueSend(queue_data_communicate, &communicate_msg, 0);
		uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
		break;
		//Event of HW FIFO overflow detected
	case UART_FIFO_OVF:
		ESP_LOGI(TAG, "hw fifo overflow");
		// If fifo overflow happened, you should consider adding flow control for your application.
		// The ISR has already reset the rx FIFO,
		// As an example, we directly flush the rx buffer here in order to read more data.
		uart_flush_input(EX_UART_NUM);
		xQueueReset(uart0_queue);
		break;
		//Event of UART ring buffer full
	case UART_BUFFER_FULL:
		ESP_LOGI(TAG, "ring buffer full");
		// If buffer full happened, you should consider encreasing your buffer size
		// As an example, we directly flush the rx buffer here in order to read more data.
		uart_flush_input(EX_UART_NUM);
		xQueueReset(uart0_queue);
		break;
		//Event of UART RX break detected
	case UART_BREAK:
		ESP_LOGI(TAG, "uart rx break");
		break;
		//Event of UART parity check error
	case UART_PARITY_ERR:
		ESP_LOGI(TAG, "uart parity error");
		break;
		//Event of UART frame error
	case UART_FRAME_ERR:
		ESP_LOGI(TAG, "uart frame error");
		break;
		//UART_PATTERN_DET
	case UART_PATTERN_DET:
		uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
		int pos = uart_pattern_pop_pos(EX_UART_NUM);
		ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos,
				buffered_size);
		if (pos == -1) {
			// There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
			// record the position. We should set a larger queue size.
			// As an example, we directly flush the rx buffer here.
			uart_flush_input(EX_UART_NUM);
		} else {
			uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
			uint8_t pat[PATTERN_CHR_NUM + 1];
			memset(pat, 0, sizeof(pat));
			uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM,
					100 / portTICK_PERIOD_MS);
			ESP_LOGI(TAG, "read data: %s", dtmp);
			ESP_LOGI(TAG, "read pat : %s", pat);
		}
		break;
		//Others
	default:
		ESP_LOGI(TAG, "uart event type: %d", event.type);
		break;
	}
}
}
free(dtmp);
dtmp = NULL;
vTaskDelete(NULL);
}

/**
 * Init UART2 to commnunicate with STM32
 */
mdf_err_t uart_start(void) {

esp_log_level_set(TAG, ESP_LOG_INFO);

/* Configure parameters of an UART driver,
 * communication pins and install the driver */
uart_config_t uart_config = { .baud_rate = 115200,
	.data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
uart_param_config(EX_UART_NUM, &uart_config);

			//Set UART log level
esp_log_level_set(TAG, ESP_LOG_INFO);
			//Set UART pins (using UART0 default pins ie no changes.)
uart_set_pin(EX_UART_NUM, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE,
	UART_PIN_NO_CHANGE);
			//Install UART driver, and get the queue.
uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue,
	0);

			//Set uart pattern detect function.
uart_enable_pattern_det_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 10000, 10, 10);
		//Reset the pattern queue length to record at most 20 pattern positions.
uart_pattern_queue_reset(EX_UART_NUM, 20);

			//Create a task to handler UART event from ISR
return MDF_OK;
}

/**
 * @brief Main task
 */

static void main_processing_task(void *param) {
type_data_communicate_t communicate_msg;
for (;;) {
xQueueReceive(queue_data_communicate, &communicate_msg,
		(portTickType) portMAX_DELAY);
switch (communicate_msg.type) {
case UPDATE_DATA_EVT:
	printf("evetn: UPDATE_DATA_EVT , data uart: %s \n", communicate_msg.data);
	// need to free pointer
	send_mqtt_msg(communicate_msg.data);
	free(communicate_msg.data);
	break;
case SETTING_EVT:
	// go to setting statement
	printf("setting cmd: %s \n", communicate_msg.data);
	restart_to_configure_program();

	break;
case PING_EVT:
	// send ACK back to STM32. If time out ,ESP32 will be restart
	uart_write_bytes(EX_UART_NUM, "3;1\n", 4);
	break;
default:
	break;
}
}

}

char*
generate_JSON(char *param) {
static int vts[9]; // value_tool_set
static int vtc[9]; // value_tool_current
char *json_string = NULL;
cJSON *json = NULL;
cJSON *json_manager_infor = NULL;
cJSON *json_device_infor = NULL;
cJSON *tool_status = NULL;
printf("heap before %d \n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

#define NUM_TOOL 9
json = cJSON_CreateObject();

json_device_infor = cJSON_CreateObject();
cJSON_AddStringToObject(json_device_infor, "firmware", device_infor.firmware);
cJSON_AddStringToObject(json_device_infor, "hardware", device_infor.hardware);
cJSON_AddStringToObject(json_device_infor, "serial", device_infor.serial);
cJSON_AddStringToObject(json_device_infor, "company", device_infor.company);

json_manager_infor = cJSON_CreateObject();
cJSON_AddStringToObject(json_manager_infor, "id", manager_infor.id);
cJSON_AddStringToObject(json_manager_infor, "name", manager_infor.name);

manager_infor.layer = esp_mesh_get_layer();
cJSON_AddNumberToObject(json_manager_infor, "layer", (int) manager_infor.layer);

sscanf(param, "%d,%d,%d,%d,%d,%d,%d,%d,%d:%d,%d,%d,%d,%d,%d,%d,%d,%d", &vts[0],
	&vts[1], &vts[2], &vts[3], &vts[4], &vts[5], &vts[6], &vts[7], &vts[8],
	&vtc[0], &vtc[1], &vtc[2], &vtc[3], &vtc[4], &vtc[5], &vtc[6], &vtc[7],
	&vtc[8]);
manager_infor.expired = false;
for (int i = 0; i < NUM_TOOL; i++) {
if (vtc[i] >= vts[i]) {
	manager_infor.expired = true;
}
}
cJSON_AddBoolToObject(json_manager_infor, "expired", manager_infor.expired);

//	cJSON_AddItemToObject(json, "toolStatus", tool_status);
tool_status = cJSON_AddArrayToObject(json, "toolStatus");

for (int i = 0; i < NUM_TOOL; i++) {

cJSON *component_array = cJSON_CreateObject();
if (cJSON_AddNumberToObject(component_array, "set", vts[i]) == NULL) {
	return NULL;
}
if (cJSON_AddNumberToObject(component_array, "count", vtc[i]) == NULL) {
	return NULL;
}
cJSON_AddItemToArray(tool_status, component_array);
}

cJSON_AddItemToObject(json, "deviceInfor", json_device_infor);
cJSON_AddItemToObject(json, "managerInfor", json_manager_infor);

json_string = cJSON_PrintUnformatted(json);
cJSON_Delete(json);
return json_string;
}

void get_infor_manager_device(void) {

strcpy(manager_infor.id, id_node_default);
strcpy(manager_infor.name, name_node_default);
}

void get_infor_hardware_device(void) {

strcpy(device_infor.firmware, firmware_version_node);
strcpy(device_infor.hardware, hardware_version_node);
strcpy(device_infor.serial, serial_number_node);
strcpy(device_infor.company, company);
}
void app_main() {

/**
 * @brief Set the log level for serial port printing.
 */
esp_err_t ret;

// Initialize NVS.
ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
ESP_ERROR_CHECK(nvs_flash_erase());
ret = nvs_flash_init();
}

esp_log_level_set("*", ESP_LOG_INFO);
esp_log_level_set(TAG, ESP_LOG_DEBUG);
esp_log_level_set("mesh_mqtt", ESP_LOG_DEBUG);

queue_data_communicate = xQueueCreate(2, sizeof(type_data_communicate_t));

/**
 * Get infor network
 */
get_infor_configure();
get_infor_hardware_device();

/**
 * @brief Initialize wifi mesh.
 */
MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
MDF_ERROR_ASSERT(wifi_init());
MDF_ERROR_ASSERT(mwifi_init(&cfg));
MDF_ERROR_ASSERT(mwifi_set_config(&mesh_config));
MDF_ERROR_ASSERT(mwifi_start());
MDF_ERROR_ASSERT(uart_start());
uart_write_bytes(EX_UART_NUM, "Start program\n", 14);
//    /**
//     * @brief Create node handler
//     */

TimerHandle_t timer = xTimerCreate("print_system_info",
	10000 / portTICK_RATE_MS, true, NULL, print_system_info_timercb);
xTimerStart(timer, 0);
xTaskCreate(main_processing_task, "process", 4096, NULL, 2, NULL);
xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 2, NULL);

}

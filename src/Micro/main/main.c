#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_tls.h"


#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10


static int s_retry_num = 0;
static const char *TAG = "HTTP_CLIENT";
static EventGroupHandle_t wifi_event_group;


static void wifi_event_handler(
    void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data
) {
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		ESP_LOGI(TAG, "Connecting to AP...");
		esp_wifi_connect();
	} else if (
        event_base == WIFI_EVENT && 
        event_id == WIFI_EVENT_STA_DISCONNECTED
    ) {
		if (s_retry_num < MAX_FAILURES) {
			ESP_LOGI(TAG, "Reconnecting to AP...");
			esp_wifi_connect();
			s_retry_num++;
		} else {
			xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
		}
	}
}

static void ip_event_handler(
    void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data
) {
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);
    }

}

esp_err_t connect_wifi(void) {
	int status = WIFI_FAILURE;

	//initialize the esp network interface
	ESP_ERROR_CHECK(esp_netif_init());

	//initialize default esp event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	//create wifi station in the wifi driver
	esp_netif_create_default_wifi_sta();

	//setup wifi station with the default wifi configuration
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t wifi_handler_event_instance;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler,
            NULL, &wifi_handler_event_instance
        )
    );

    esp_event_handler_instance_t got_ip_event_instance;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler,
            NULL, &got_ip_event_instance
        )
    );

    /** START THE WIFI DRIVER **/
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SSID", //temporary
            .password = "PASSWORD",
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // set the wifi controller to be a station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // set the wifi config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // start the wifi driver
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA initialization complete");

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group, WIFI_SUCCESS | WIFI_FAILURE,
        pdFALSE, pdFALSE, portMAX_DELAY
    );

    /* 
     * xEventGroupWaitBits() returns the bits before the call returned, hence
     * we can test which event actually happened. 
     */
    if (bits & WIFI_SUCCESS) {
        ESP_LOGI(TAG, "Connected to ap");
        status = WIFI_SUCCESS;
    } else if (bits & WIFI_FAILURE) {
        ESP_LOGI(TAG, "Failed to connect to ap");
        status = WIFI_FAILURE;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        status = WIFI_FAILURE;
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(
        esp_event_handler_instance_unregister(
            IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_event_instance
        )
    );
    ESP_ERROR_CHECK(
        esp_event_handler_instance_unregister(
            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance
        )
    );

    vEventGroupDelete(wifi_event_group);

    return status;
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    static char *output_buffer;
    static int output_len;
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(
                TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", 
                evt->header_key, evt->header_value
            );
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             * Check for chunked encoding is added as the URL for chunked
             * encoding used in this example returns binary data. However,
             * event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (evt->user_data) {
                    memcpy(
                        evt->user_data + output_len, 
                        evt->data, 
                        evt->data_len
                    );
                    output_len += evt->data_len;
                    break;
                }
                if (output_buffer == NULL) {
                    output_buffer = (char *) malloc(
                        esp_http_client_get_content_length(evt->client)
                    );
                    output_len = 0;
                    if (output_buffer == NULL) {
                        ESP_LOGE(
                            TAG, "Failed to allocate memory for output buffer"
                        );
                        return ESP_FAIL;
                    }
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len);

                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                /*
                 * Response is accumulated in output_buffer. 
                 * Uncomment the below line to print the accumulated response
                 */
                //ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(
                (esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL
            );

            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
    }
    return ESP_OK;
}

static void http_rest_with_url(void) {
    char local_response_buffer[1024] = {0};
    /*
     * NOTE: All the configuration parameters for http_client must be spefied
     * either in URL or as host and path parameters.
     * If host and path parameters are not set, query parameter will be
     * ignored. In such cases, query parameter should be specified in URL.
     *
     * If URL as well as host and path parameters are specified,
     * values of host and path will be considered.
     */
    esp_http_client_config_t config = {
        .host = "HOST",
        .path = "/",
        .port = 8080,
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "%s", local_response_buffer);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

void setupStorage(void){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void app_main(void){
    ESP_LOGI(TAG, "Entering Main");
    connect_wifi();
    while (1) {
        sys_delay_ms(1000);
        http_rest_with_url();
    }
}

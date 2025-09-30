// Temperature-Sensor.c  (main.c)
// ESP-IDF v5.x
// - HTTPS ingest to Render (with X-API-Key)
// - SNTP time sync (TLS needs correct clock)
// - Cert bundle trust (Let's Encrypt, etc.)
// - MAX31856 read + per-interval POST
// - SoftAP portal fallback if Wi-Fi not provisioned

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_sntp.h"

#include "wifi_mgr.h"     // your Wi-Fi manager (PSK + Enterprise)
#include "portal.h"       // your SoftAP provisioning portal
#include "nvs_kv.h"       // your NVS helpers (optional here)
#include "max31856.h"     // your MAX31856 driver

// -------------------- Settings --------------------
static const char *TAG = "APP";

#define WIFI_CONNECT_TIMEOUT_MS   40000   // Enterprise can take a while

#define ENABLE_HTTP_POST 1
#if ENABLE_HTTP_POST
  // Cloud ingest endpoint (HTTPS, no :3000)
  #define API_URL        "https://freezer-monitor-server.onrender.com/ingest"
  // MUST match Render → Environment → API_KEY
  #define API_KEY        "super_secret_key_here"
  // Post cadence
  #define POST_PERIOD_MS 15000
#endif

// SPI pins (ESP32-S3)
#define PIN_NUM_MISO 13 // SDO
#define PIN_NUM_MOSI 11 // SDI
#define PIN_NUM_CLK  12 // SCK
#define PIN_NUM_CS   10 // CS

// -------------------- Helpers --------------------
static void sntp_sync(void) {
    // Start SNTP and wait until time is sane (> 2021-01-01)
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    for (int i = 0; i < 200 && time(NULL) < 1609459200; ++i) { // ~20s max
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void https_health_check(void) {
    esp_http_client_config_t hc = {
        .url = "https://freezer-monitor-server.onrender.com/health",
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t h = esp_http_client_init(&hc);
    if (!h) { ESP_LOGW(TAG, "health: init failed"); return; }
    esp_err_t err = esp_http_client_perform(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "GET /health -> %d", esp_http_client_get_status_code(h));
    } else {
        ESP_LOGW(TAG, "GET /health failed: %s (errno=%d)",
                 esp_err_to_name(err), esp_http_client_get_errno(h));
    }
    esp_http_client_cleanup(h);
}

#if ENABLE_HTTP_POST
static esp_err_t http_post_reading(const char *device_id, float temp_c, uint8_t sr) {
    char body[192];
    int n = snprintf(body, sizeof(body),
                     "{\"device_id\":\"%s\",\"temp_c\":%.2f,\"sr\":%u}",
                     device_id, temp_c, (unsigned)sr);
    if (n < 0 || n >= (int)sizeof(body)) return ESP_FAIL;

    esp_http_client_config_t cfg = {
        .url = API_URL,
        .method = HTTP_METHOD_POST,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,   // HTTPS
        .crt_bundle_attach = esp_crt_bundle_attach,  // trust CA bundle
        .timeout_ms = 10000,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-API-Key",    API_KEY);
    esp_http_client_set_post_field(client, body, n);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int sc = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "POST /ingest -> %d", sc);
        if (sc != 200) {
            char buf[160];
            int rd = esp_http_client_read_response(client, buf, sizeof(buf)-1);
            if (rd > 0) { buf[rd]=0; ESP_LOGW(TAG, "resp: %s", buf); }
        }
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %s, errno=%d",
                 esp_err_to_name(err),
                 esp_http_client_get_errno(client));
    }
    esp_http_client_cleanup(client);
    return err;
}
#endif

static void get_device_id(char *out, size_t len) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out, len, "esp32-%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// -------------------- app_main --------------------
void app_main(void) {
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("eap",  ESP_LOG_INFO);
    esp_log_level_set("wpa",  ESP_LOG_INFO);

    // ------- SPI init -------
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 1,                         // MAX31856 uses SPI mode 1
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };
    spi_device_handle_t dev;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &dev));
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLDOWN_ONLY);
    ESP_LOGI(TAG, "SPI bus initialized");

    max31856_attach(dev);
    max31856_init();

    // ------- Wi-Fi -------
    wifi_netif_init_once();
    if (wifi_try_load_and_connect_auto(WIFI_CONNECT_TIMEOUT_MS) != WIFI_RES_OK) {
        char ap_ssid[32];
        uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
        snprintf(ap_ssid, sizeof(ap_ssid), "FreezerMonitor-%02X%02X", mac[4], mac[5]);
        wifi_start_softap(ap_ssid, "freezer123");
        portal_start();
        ESP_LOGI(TAG, "Provisioning: connect to SSID '%s', open http://192.168.4.1/", ap_ssid);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Wi-Fi connected.");

    // ------- TLS prerequisites -------
    sntp_sync();           // set time for certificate validation
    https_health_check();  // ensure DNS+TLS path works (expect 200)

    // ------- Device ID -------
    char device_id[32] = {0};
    get_device_id(device_id, sizeof(device_id));
    ESP_LOGI(TAG, "Device ID: %s", device_id);

    // ------- Main loop -------
    while (1) {
        float   temp_c = 0.0f;
        uint8_t sr     = 0;

        if (max31856_get_temp_c(&temp_c, &sr)) {
            ESP_LOGI(TAG, "Thermocouple Temp: %.2f°C (sr=0x%02X)", temp_c, sr);
            #if ENABLE_HTTP_POST
            (void)http_post_reading(device_id, temp_c, sr);
            #endif
        } else {
            ESP_LOGW(TAG, "MAX31856 read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(POST_PERIOD_MS));
    }
}

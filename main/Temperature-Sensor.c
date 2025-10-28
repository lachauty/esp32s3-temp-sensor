// Temperature-Sensor.c  (main.c)
// ESP-IDF v5.x
// - HTTPS ingest to Render (with X-API-Key)
// - SNTP time sync (TLS needs correct clock)
// - Cert bundle trust (Let's Encrypt, etc.)
// - MAX31856 read + per-interval POST with queue
// - Health checks + alert LED (GPIO1) if no successful ingest
// - SoftAP portal fallback if Wi-Fi not provisioned

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

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
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_pm.h"

#include "wifi_mgr.h"     // your Wi-Fi manager (PSK + Enterprise)
#include "portal.h"       // your SoftAP provisioning portal
#include "nvs_kv.h"       // your NVS helpers (optional here)
#include "max31856.h"     // your MAX31856 driver

// -------------------- Settings --------------------
static const char *TAG = "APP";

#define WIFI_CONNECT_TIMEOUT_MS   40000   // Enterprise can take a while

#define ENABLE_HTTP_POST 1
#if ENABLE_HTTP_POST
  static int http_post_reading(const char *device_id, float temp_c, uint8_t sr, int64_t ts_ms);
// Cloud ingest endpoint (HTTPS, no :3000)
  #define API_URL        "https://freezer-monitor-server.onrender.com/ingest"
  // MUST match Render → Environment → API_KEY
  #define API_KEY        "super_secret_key_here"
  // Default post cadence
  #define POST_PERIOD_MS 15000
  #else
  static inline int http_post_reading(const char *device_id, float temp_c, uint8_t sr, int64_t ts_ms)
  { (void)device_id; (void)temp_c; (void)sr; (void)ts_ms; return -1; }
#endif

// ---------- Health control ----------
static bool s_server_ok = false;
static const int64_t HEALTH_PERIOD_US = 60LL * 1000000LL; // every 60s

// ---------- Sample queue (lock-protected ring buffer) ----------
typedef struct {
    float    t_c;
    uint8_t  sr;
    int64_t  ts_ms_utc;
} reading_t;

#define RB_CAP 16
#define ALERT_LED_GPIO 1   // Alert LED on GPIO1

// Forward declarations used by tasks:
static bool https_health_check(void);

// Queue
static reading_t   s_rb[RB_CAP];
static volatile int s_rb_head = 0, s_rb_tail = 0;
static portMUX_TYPE s_rb_lock = portMUX_INITIALIZER_UNLOCKED;

static inline bool rb_push(reading_t r){
    taskENTER_CRITICAL(&s_rb_lock);
    int nhead = (s_rb_head + 1) % RB_CAP;
    if (nhead == s_rb_tail) { s_rb_tail = (s_rb_tail + 1) % RB_CAP; } // drop oldest
    s_rb[s_rb_head] = r;
    s_rb_head = nhead;
    taskEXIT_CRITICAL(&s_rb_lock);
    return true;
}

static inline bool rb_pop(reading_t *out){
    taskENTER_CRITICAL(&s_rb_lock);
    bool ok = (s_rb_tail != s_rb_head);
    if (ok) { *out = s_rb[s_rb_tail]; s_rb_tail = (s_rb_tail + 1) % RB_CAP; }
    taskEXIT_CRITICAL(&s_rb_lock);
    return ok;
}

// ----- Tasks & timers (“software interrupts”) -----
static TaskHandle_t s_task_sensor = NULL;
static TaskHandle_t s_task_net    = NULL;

static esp_timer_handle_t s_timer_sample = NULL;
static esp_timer_handle_t s_timer_health = NULL;

static int64_t s_last_ingest_ok_us = 0;

#define ALERT_WINDOW_MIN 2
#define ALERT_WINDOW_US  ((int64_t)ALERT_WINDOW_MIN * 60LL * 1000000LL)

static bool s_alert_active = false;

// Make device_id visible to tasks
static char s_device_id[32] = {0};

// SPI pins (ESP32-S3)
#define PIN_NUM_MISO 13 // SDO
#define PIN_NUM_MOSI 11 // SDI
#define PIN_NUM_CLK  12 // SCK
#define PIN_NUM_CS   10 // CS

// Timer callbacks (post a notify to tasks)
static void cb_sample(void *arg){
    (void)arg;
    if (s_task_sensor) xTaskNotifyGive(s_task_sensor);
    if (s_server_ok && s_task_net) xTaskNotifyGive(s_task_net); // only when healthy
}

static void cb_health(void *arg){
    (void)arg;
    if (s_task_net) xTaskNotifyGive(s_task_net);
}

// -------------------- Tasks --------------------
static void task_sensor(void *arg){
    for(;;){
        // wait for “software interrupt” from sample timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float t=0; uint8_t sr=0;
        if (max31856_get_temp_c(&t, &sr)) {
            struct timeval tv; gettimeofday(&tv, NULL);
            int64_t ts_ms = (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
        
            reading_t r = { .t_c = t, .sr = sr, .ts_ms_utc = ts_ms };
            rb_push(r);
            ESP_LOGI(TAG, "Sample queued: %.2f°C (sr=0x%02X) @ %lld",
                     t, sr, (long long)ts_ms);
        }else {
            ESP_LOGW(TAG, "MAX31856 read failed");
        }
    }
}

static void update_alert_led(bool on){
    static bool init = false;
    if (!init) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << ALERT_LED_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        init = true;
    }
    gpio_set_level(ALERT_LED_GPIO, on ? 1 : 0);
}

static void task_net(void *arg){

    int64_t last_health_us = 0;

    for(;;){
        // wait for “software interrupt” from health timer (or sample timer when healthy)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        // 1) periodic health check (max once per HEALTH_PERIOD_US)
        int64_t now = esp_timer_get_time();
        bool ok = s_server_ok;  // default to current view
        if (now - last_health_us >= HEALTH_PERIOD_US) {
            ok = https_health_check();
            last_health_us = now;
        }

        if (ok && !s_server_ok) {
            ESP_LOGI(TAG, "Server healthy; clearing alert");
            s_alert_active = false;
            update_alert_led(false);
        }
        s_server_ok = ok;

        // 2) If healthy, flush any queued samples
        if (s_server_ok){
            reading_t r;
            int sent = 0;
            while (rb_pop(&r)) {
                int sc = http_post_reading(s_device_id, r.t_c, r.sr, r.ts_ms_utc);
                if (sc == 200) {
                    s_last_ingest_ok_us = esp_timer_get_time();
                    sent++;
                } else {
                    rb_push(r);
                    break;
                }
            }
            if (sent) ESP_LOGI(TAG, "Flushed %d queued reading(s)", sent);
        }

        // 3) Alert if no successful ingest for too long
        now = esp_timer_get_time();
        if (s_last_ingest_ok_us == 0) s_last_ingest_ok_us = now; // baseline at boot
        bool overdue = (now - s_last_ingest_ok_us) > ALERT_WINDOW_US;
        if (overdue && !s_alert_active){
            s_alert_active = true;
            update_alert_led(true);
            ESP_LOGW(TAG, "ALERT: No successful ingest for > %d min",
                     (int)(ALERT_WINDOW_US/60000000LL));
        }
        if (!overdue && s_alert_active){
            s_alert_active = false;
            update_alert_led(false);
        }
        
    }
}

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

static bool https_health_check(void) {
    esp_http_client_config_t hc = {
        .url = "https://freezer-monitor-server.onrender.com/health",
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 8000,
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t h = esp_http_client_init(&hc);
    if (!h) { ESP_LOGW(TAG, "health: init failed"); return false; }

    bool ok = false;
    esp_err_t err = esp_http_client_perform(h);
    if (err == ESP_OK) {
        int sc = esp_http_client_get_status_code(h);
        ESP_LOGI(TAG, "GET /health -> %d", sc);
        ok = (sc == 200);
    } else {
        ESP_LOGW(TAG, "GET /health failed: %s (errno=%d)",
                 esp_err_to_name(err), esp_http_client_get_errno(h));
        ok = false;
    }
    esp_http_client_cleanup(h);
    return ok;
}

#if ENABLE_HTTP_POST
static int http_post_reading(const char *device_id, float temp_c, uint8_t sr, int64_t ts_ms) {
    char body[256];
    int n = snprintf(body, sizeof(body),
                     "{\"device_id\":\"%s\",\"temp_c\":%.2f,\"sr\":%u,\"ts_ms\":%lld}",
        device_id, temp_c, (unsigned)sr, (long long)ts_ms);
    if (n < 0 || n >= (int)sizeof(body)) return -1;

    esp_http_client_config_t cfg = {
        .url = API_URL,
        .method = HTTP_METHOD_POST,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
        .keep_alive_enable = true,
    };

    int status = -1;
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return -1;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-API-Key",    API_KEY);
    esp_http_client_set_post_field(client, body, n);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "POST /ingest -> %d", status);
        if (status != 200) {
            char buf[160];
            int rd = esp_http_client_read_response(client, buf, sizeof(buf)-1);
            if (rd > 0) { buf[rd]=0; ESP_LOGW(TAG, "resp: %s", buf); }
        }
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %s, errno=%d",
                 esp_err_to_name(err), esp_http_client_get_errno(client));
        status = -1;
    }
    esp_http_client_cleanup(client);
    return status;
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

    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MAX_MODEM) );
    
    #if CONFIG_PM_ENABLE
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
    #endif

    // ------- TLS prerequisites -------
    sntp_sync();
    s_server_ok = https_health_check();

    // ------- Device ID -------
    char device_id[32] = {0};
    get_device_id(device_id, sizeof(device_id));
    ESP_LOGI(TAG, "Device ID: %s", device_id);
    strncpy(s_device_id, device_id, sizeof(s_device_id)-1);

    // Re-tune the already-initialized Task WDT (IDF auto-started it)
    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms     = 30000,   // 30s
        .trigger_panic  = false,
        .idle_core_mask = (1<<0)|(1<<1) // watch IDLE0 & IDLE1
    };
    esp_task_wdt_reconfigure(&twdt_cfg);


    // Optional: quick LED blink to prove GPIO1 works
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << ALERT_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    for (int i = 0; i < 2; ++i) {
        gpio_set_level(ALERT_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(ALERT_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(150));
    }

    // --- Create tasks ---
    xTaskCreatePinnedToCore(task_sensor, "t_sensor", 4096, NULL, 8, &s_task_sensor, 1);
    xTaskCreatePinnedToCore(task_net,    "t_net",    6144, NULL, 8, &s_task_net,    1);

    // --- Create periodic timers (software “interrupts”) ---
    const esp_timer_create_args_t t_sample_args = {
        .callback = &cb_sample, .arg = NULL, .name = "sample"
    };
    ESP_ERROR_CHECK( esp_timer_create(&t_sample_args, &s_timer_sample) );
    ESP_ERROR_CHECK( esp_timer_start_periodic(s_timer_sample, (uint64_t)POST_PERIOD_MS * 1000ULL) );

    const esp_timer_create_args_t t_health_args = {
        .callback = &cb_health, .arg = NULL, .name = "health"
    };
    ESP_ERROR_CHECK( esp_timer_create(&t_health_args, &s_timer_health) );
    ESP_ERROR_CHECK( esp_timer_start_periodic(s_timer_health, HEALTH_PERIOD_US) );

    // Park main task
    vTaskDelete(NULL);
}

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

// Settings
static const char *TAG = "APP";
#define WIFI_CONNECT_TIMEOUT_MS   40000   // Enterprise can take a while

#define POST_PERIOD_MS 15000   // Default post cadence

const char* PRIMARY_BASE = "https://freezer-monitor-server.onrender.com";
const char* FALLBACK_BASE = "http://192.168.0.42:3000";  // your laptop / local server IP on the LAN

#define USE_SMOOTHING     1
#define SMOOTH_ALPHA      0.25f

static char s_base_url[128] = {0};
static bool s_use_tls = false;

//Local and cloud server website location
static const char *URL_LOCAL = "http://172.16.0.123:3000";
static const char *URL_CLOUD = "https://freezer-monitor-server.onrender.com";

#define ENABLE_HTTP_POST 1
#if ENABLE_HTTP_POST
  static int http_post_reading(const char *device_id, float temp_c, uint8_t sr, int64_t ts_ms);

  // MUST match Render → Environment → API_KEY
  #define API_KEY        "super_secret_key_here"
  #else
  static inline int http_post_reading(const char *device_id, float temp_c, uint8_t sr, int64_t ts_ms)
  { (void)device_id; (void)temp_c; (void)sr; (void)ts_ms; return -1; }
#endif

// Health control
static bool s_server_ok = false;
static const int64_t HEALTH_PERIOD_US = 60LL * 1000000LL; // every 60s

// Sample queue (lock-protected ring buffer)
// Fixed-size First in first out (FIFO) buffer (no malloc)
typedef struct {
    float    t_c;
    uint8_t  sr;
    int64_t  ts_ms_utc;
} reading_t;

//16 samples
#define RB_CAP 16
#define ALERT_LED_GPIO 1   // Alert LED on GPIO1

// Forward declarations used by tasks:
static bool https_health_check(void);

// Forward declarations for helpers used before their definitions
static bool try_health_once(const char *base, bool tls);
static void pick_base_url(void);
static void maybe_prefer_local_again(void);


// Sample Queue
static reading_t   s_rb[RB_CAP];
static volatile int s_rb_head = 0, s_rb_tail = 0;
static portMUX_TYPE s_rb_lock = portMUX_INITIALIZER_UNLOCKED;

// push or producer method
static inline bool rb_push(reading_t r){
    //enter critical section
    taskENTER_CRITICAL(&s_rb_lock);
    //move head to next
    int nhead = (s_rb_head + 1) % RB_CAP;
    //if full, drop the oldest by moving tail
    if (nhead == s_rb_tail) { s_rb_tail = (s_rb_tail + 1) % RB_CAP; } // drop oldest
    //store the new reading
    s_rb[s_rb_head] = r;
    s_rb_head = nhead;
    taskEXIT_CRITICAL(&s_rb_lock);
    return true;
}

//pop or consumer metho
static inline bool rb_pop(reading_t *out){
    //enter critical section
    taskENTER_CRITICAL(&s_rb_lock);
    bool ok = (s_rb_tail != s_rb_head);
    //returns false if empty and copies oldest item and advances tail
    if (ok) { *out = s_rb[s_rb_tail]; s_rb_tail = (s_rb_tail + 1) % RB_CAP; }
    taskEXIT_CRITICAL(&s_rb_lock);
    return ok;
}

// Tasks & timers & software interrupts
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
    //every 15 sec wakeup sensor task
    if (s_task_sensor) xTaskNotifyGive(s_task_sensor);
    //if server healthy, wake net task
    if (s_server_ok && s_task_net) xTaskNotifyGive(s_task_net); // only when healthy
}

// healthy timer callback
static void cb_health(void *arg){
    (void)arg;
    //every 60s wake up net task
    if (s_task_net) xTaskNotifyGive(s_task_net);
}

// Tasks
static void task_sensor(void *arg){

    static bool  s_have_filt = false;
    static float s_filt_c    = 0.0f;

    //loop
    for(;;){
        // wait for software interrupt to wake
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float t=0; uint8_t sr=0;
        //read sensor
        if (max31856_get_temp_c(&t, &sr)) {

            // Fault-aware smoothing: if sr!=0, treat as raw (don’t smooth faults)
            float use_c = t;
            //temperature reading smoothing
#if USE_SMOOTHING
            if (sr == 0) {
                if (!s_have_filt) { s_filt_c = t; s_have_filt = true; }
                else              { s_filt_c = SMOOTH_ALPHA * t + (1.0f - SMOOTH_ALPHA) * s_filt_c; }
                use_c = s_filt_c;
            } else {
                // pass through raw on fault; keep EMA state so it catches up next sample
                use_c = t;
            }
#endif
            // get timestamp (UTC)
            struct timeval tv; gettimeofday(&tv, NULL);
            int64_t ts_ms = (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
        
            //push into ring buffer
            reading_t r = { .t_c = use_c, .sr = sr, .ts_ms_utc = ts_ms };
            rb_push(r);
            
            ESP_LOGI(TAG, "Sample queued: raw=%.2f°C filt=%.2f°C -> send=%.2f°C (sr=0x%02X) @ %lld",
            t, s_have_filt ? s_filt_c : t, r.t_c, sr, (long long)ts_ms);
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

// health check, upload queue, alert LED
// wakes always from health timer, and sample timer when healthy
static void task_net(void *arg){

    int64_t last_health_us = 0;

    for(;;){
        // wait for “software interrupt” from health timer (or sample timer when healthy)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 1) periodic health check (max once per HEALTH_PERIOD_US)
        int64_t now = esp_timer_get_time();
        bool ok = s_server_ok;  // default to current view
        //health check every 60s
        if (now - last_health_us >= HEALTH_PERIOD_US) {
            ok = https_health_check();
            last_health_us = now;
            maybe_prefer_local_again();
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

            //while loop for if healthy, flush queued samples to server
            while (rb_pop(&r)) {
                int sc = http_post_reading(s_device_id, r.t_c, r.sr, r.ts_ms_utc);
                if (sc == 200) {
                    s_last_ingest_ok_us = esp_timer_get_time();
                    sent++;
                } else if (sc >= 500 || sc < 0) {
                    // server problem or transport error → requeue and stop for now
                    rb_push(r);
                    break;
                } else if (sc == 401 || sc == 403) {
                    ESP_LOGE(TAG, "Forbidden (API key?) — dropping sample and keeping alert active");
                    // drop this sample; optionally set a sticky flag to blink LED faster
                } else if (sc >= 400) {
                    ESP_LOGW(TAG, "Client error %d — dropping bad sample", sc);
                    // drop this sample (don’t requeue)
                } else {
                    // unexpected → be conservative
                    rb_push(r);
                    break;
                }
            }
            if (sent) ESP_LOGI(TAG, "Flushed %d queued reading(s)", sent);
        }

        // 3) Alert if no successful ingest for too long
        now = esp_timer_get_time();
        if (s_last_ingest_ok_us == 0) s_last_ingest_ok_us = now; // baseline at boot
        // Alert if no successful ingest for 2 minutes
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

static void maybe_prefer_local_again(void) {
    static int counter = 0;
    // every 5 health cycles (5 minutes)
    if (++counter % 5) return;

    // on cloud, check if local sever is available
    if (strcmp(s_base_url, URL_LOCAL) != 0) {

        // if local is reachable switch back to local
        if (try_health_once(URL_LOCAL, false)) {
            strncpy(s_base_url, URL_LOCAL, sizeof(s_base_url)-1);
            s_use_tls = false;
            ESP_LOGI(TAG, "Re-selected BASE=LOCAL: %s", s_base_url);
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
    return try_health_once(s_base_url, s_use_tls);
}

static bool try_health_once(const char *base, bool tls){
    char url[200];
    snprintf(url, sizeof(url), "%s/health", base);

    esp_http_client_config_t hc = {
        .url = url,
        .transport_type = tls ? HTTP_TRANSPORT_OVER_SSL : HTTP_TRANSPORT_OVER_TCP,
        .crt_bundle_attach = tls ? esp_crt_bundle_attach : NULL,
        .timeout_ms = 8000,
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t h = esp_http_client_init(&hc);
    if (!h) { ESP_LOGW(TAG, "health(init) failed"); return false; }

    bool ok = false;
    esp_err_t err = esp_http_client_perform(h);
    if (err == ESP_OK) {
        int sc = esp_http_client_get_status_code(h);
        ESP_LOGI(TAG, "GET /health -> %d (%s)", sc, base);
        // check if health is 200 (server connection success) or 503 (server connection failure)
        // we keep trying even if failure
        ok = (sc == 200 || sc == 503);
    } else {
        ESP_LOGW(TAG, "GET /health failed (%s): %s (errno=%d)",
                 base, esp_err_to_name(err), esp_http_client_get_errno(h));
    }
    esp_http_client_cleanup(h);
    return ok;
}

static void pick_base_url(void){
    // Try LOCAL server first
    if (try_health_once(URL_LOCAL, /*tls=*/false)) {
        strncpy(s_base_url, URL_LOCAL, sizeof(s_base_url)-1);
        s_use_tls = false;
        ESP_LOGI(TAG, "Selected BASE=LOCAL: %s", s_base_url);
        return;
    }
    // Fallback to CLOUD server with Transport Layer Security (Refer to 7 layers of OSI Model)
    if (try_health_once(URL_CLOUD, /*tls=*/true)) {
        strncpy(s_base_url, URL_CLOUD, sizeof(s_base_url)-1);
        s_use_tls = true;
        ESP_LOGI(TAG, "Selected BASE=CLOUD: %s", s_base_url);
        return;
    }
    // if neither reachable, try cloud anyways
    if (!s_base_url[0]) {
        strncpy(s_base_url, URL_CLOUD, sizeof(s_base_url)-1);
        s_use_tls = true;
        ESP_LOGW(TAG, "No server reachable; defaulting BASE=%s", s_base_url);
    }
}



#if ENABLE_HTTP_POST
// method building JSON and posts to BASE/ingest
// sets headers: content type -> applications and JSON
// X-API-KEY
static int http_post_reading(const char *device_id, float temp_c, uint8_t sr, int64_t ts_ms) {
    // character buffer to build JSON
    char body[256];
    // writes measurement logs into buffer
    int n = snprintf(body, sizeof(body),
                     "{\"device_id\":\"%s\",\"temp_c\":%.2f,\"sr\":%u,\"ts_ms\":%lld}",
                     device_id, temp_c, (unsigned)sr, (long long)ts_ms);
    if (n < 0 || n >= (int)sizeof(body)) return -1;

    char url[200];
    // writes ingest message
    snprintf(url, sizeof(url), "%s/ingest", s_base_url);

    //setting http client config
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .transport_type = s_use_tls ? HTTP_TRANSPORT_OVER_SSL : HTTP_TRANSPORT_OVER_TCP,
        
        // Transport Layer Security is enabled, it attaches the esp_crt_bundle_attach cert bundle
        // we use esp_crt_bundle_attach so that we dont get our own privacy-enhanced mail (PEM) cert
        .crt_bundle_attach = s_use_tls ? esp_crt_bundle_attach : NULL,
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
        ESP_LOGI(TAG, "POST /ingest -> %d (%s)", status, s_base_url);
        if (status != 200) {
            char buf[160];
            int rd = esp_http_client_read_response(client, buf, sizeof(buf)-1);
            if (rd > 0) { buf[rd]=0; ESP_LOGW(TAG, "resp: %s", buf); }
        }
    } else {
        ESP_LOGE(TAG, "HTTP POST failed (%s): %s, errno=%d",
                 s_base_url, esp_err_to_name(err), esp_http_client_get_errno(client));
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

// app_main or main method

void app_main(void) {
    // log level set WIFI, EAP, and WPA
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("eap",  ESP_LOG_INFO);
    esp_log_level_set("wpa",  ESP_LOG_INFO);

    // SPI and MAX31856 config init
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    // Init Wi-Fi stack
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    // Config device interface
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 1,                         // MAX31856 uses SPI mode 1
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    // Error check device handle
    spi_device_handle_t dev;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &dev));
    // configuring GPIO pin pull down
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLDOWN_ONLY);
    ESP_LOGI(TAG, "SPI bus initialized");

    // attach device and initialize max31856 interfaces
    max31856_attach(dev);
    max31856_init();

    // Wi-Fi initialize call
    wifi_netif_init_once();
    // Try Wi-Fi loading and connection
    if (wifi_try_load_and_connect_auto(WIFI_CONNECT_TIMEOUT_MS) != WIFI_RES_OK) {
        // Read mac address and write hotspot name into buffer
        char ap_ssid[32];
        uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
        snprintf(ap_ssid, sizeof(ap_ssid), "FreezerMonitor-%02X%02X", mac[4], mac[5]);
        // start softAP and portal
        wifi_start_softap(ap_ssid, "freezer123");
        portal_start();
        ESP_LOGI(TAG, "Provisioning: connect to SSID '%s', open http://192.168.4.1/", ap_ssid);
        // delay for 1000 ms
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Wi-Fi connected.");

    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MAX_MODEM) );
    
    //initialize clock rate configuration and enable sleep mode
    #if CONFIG_PM_ENABLE
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
    #endif

    // Transport Layer Security prerequisites 

    sntp_sync();

    // Pick LOCAL, else CLOUD -> this also checks both /health once
    pick_base_url();
    s_server_ok = try_health_once(s_base_url, s_use_tls);

    // Device ID
    char device_id[32] = {0};
    get_device_id(device_id, sizeof(device_id));
    ESP_LOGI(TAG, "Device ID: %s", device_id);
    strncpy(s_device_id, device_id, sizeof(s_device_id)-1);

    // Re-tune the already-initialized Task Watch dog timer (ESP-IDF auto-starts it)
    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms     = 30000,   // 30s
        .trigger_panic  = false,
        .idle_core_mask = (1<<0)|(1<<1) // watchdog IDLE0 & IDLE1
    };
    esp_task_wdt_reconfigure(&twdt_cfg);


    // quick LED blink to prove GPIO1 works -> config for LED
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << ALERT_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, 
        .pull_down_en = 0, 
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    for (int i = 0; i < 2; ++i) {
        // LED blinking
        gpio_set_level(ALERT_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(ALERT_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(150));
    }

    // Create tasks
    xTaskCreatePinnedToCore(task_sensor, "t_sensor", 4096, NULL, 8, &s_task_sensor, 1);
    xTaskCreatePinnedToCore(task_net,    "t_net",    6144, NULL, 8, &s_task_net,    1);

    // Create periodic timers (software “interrupts”)
    const esp_timer_create_args_t t_sample_args = {
        .callback = &cb_sample, .arg = NULL, .name = "sample"
    };

    // ESP-IDF high resolution software API -> essentially an Interrupt Service Routine to trigger sensor and network pipeline
    // we use 1000ULL because our constants are in ms, so what we want is micro seconds and ULL ensures 64 bit math
    // 64 bit math because we don't want overflows.
    ESP_ERROR_CHECK( esp_timer_create(&t_sample_args, &s_timer_sample) );
    ESP_ERROR_CHECK( esp_timer_start_periodic(s_timer_sample, (uint64_t)POST_PERIOD_MS * 1000ULL) );

    // health timer configuration
    const esp_timer_create_args_t t_health_args = {
        .callback = &cb_health, 
        .arg = NULL, 
        .name = "health"
    };

    // create the timer and the timer handler
    ESP_ERROR_CHECK( esp_timer_create(&t_health_args, &s_timer_health) );
    // start timer
    ESP_ERROR_CHECK( esp_timer_start_periodic(s_timer_health, HEALTH_PERIOD_US) );

    // Park main task
    vTaskDelete(NULL);
}

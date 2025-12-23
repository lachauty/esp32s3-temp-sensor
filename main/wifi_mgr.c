//wifi_mgr.c
#include "wifi_mgr.h"
#include "nvs_kv.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_eap_client.h"   // WPA2-Enterprise API

#include <string.h>
#include <stdbool.h>

static const char *TAG = "wifi_mgr";

// Event group & GOT IP Bit
static EventGroupHandle_t s_evt;
#define GOT_IP_BIT BIT0

// Try connect calls esp_wifi_connect but handles a common harmless case
static void try_connect(void) {
    esp_err_t e = esp_wifi_connect();
    if (e == ESP_ERR_WIFI_CONN) {
        // Already connecting; this is harmless and common if called twice.
        ESP_LOGW(TAG, "Already connecting; ignoring esp_wifi_connect()");
    } else if (e != ESP_OK) {
        // Log but do not crash—avoid boot loops on transient states.
        ESP_LOGW(TAG, "esp_wifi_connect returned: %s", esp_err_to_name(e));
    }
}

// Stops Wi-Fi but tolerates "not started/initialized" to call anytime
// Makes code simpler because there is no need to track states perfectly
static void wifi_stop_safely(void) {
    esp_err_t e = esp_wifi_stop();
    // Tolerate "not init / not started" states so we can call this freely
    if (e != ESP_OK && e != ESP_ERR_WIFI_NOT_INIT && e != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_ERROR_CHECK(e);
    }
}

// Called when Wi-Fi events happen
static void handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    // STA mode starts -> immediately attempt connection
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        try_connect();
    } 
    // Clear "got IP" -> offline now
    // wait 500 ms
    // retry connection
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_evt, GOT_IP_BIT);
        // small backoff before retrying
        vTaskDelay(pdMS_TO_TICKS(500));
        try_connect();
    } 
    // Got IP success signal
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_evt, GOT_IP_BIT);
    }
}

// One-time init function
void wifi_netif_init_once(void) {
    static bool inited = false;
    if (inited) return;

    // nonvolatile storage checked to be ready
    ESP_ERROR_CHECK(kv_init());

    // networking initialize and event creation
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // create default Wifi station mode (STA) and AP netifs to switch modes later
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Create Event group and event handlers
    s_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &handler, NULL, NULL));

    inited = true;
}


wifi_result_t wifi_connect_psk_now(const char *ssid, const char *pass, int timeout_ms) {
    // build a wifi config
    if (!ssid || !*ssid) return WIFI_RES_FAIL;

    // set ssid and password using strncpy
    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid) - 1);
    if (pass) strncpy((char *)wc.sta.password, pass, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;       // works for WPA/WPA2 mixed as well
    wc.sta.pmf_cfg.capable = true;
    wc.sta.pmf_cfg.required = false;

    // clear GOT IP bit
    xEventGroupClearBits(s_evt, GOT_IP_BIT);

    // stop Wifi and switch to station mode, set config, and start wifi
    wifi_stop_safely();                                   // safe if not started
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());                    // STA_START -> handler -> try_connect()

    // start Station mode, handler triggers connect, wait for got-ip
    // wait for GOT IP bit to timeout
    EventBits_t b = xEventGroupWaitBits(
        s_evt, GOT_IP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    return (b & GOT_IP_BIT) ? WIFI_RES_OK : WIFI_RES_FAIL;
}

wifi_result_t wifi_try_load_and_connect_psk(int timeout_ms) {
    char ssid[33] = {0}, pass[65] = {0};
    if (kv_get_str("ssid", ssid, sizeof(ssid)) != 0) {
        return WIFI_RES_FAIL;
    }
    // PSK may be empty (open network); treat missing key as empty
    if (kv_get_str("psk", pass, sizeof(pass)) != 0) {
        pass[0] = '\0';
    }

    ESP_LOGI(TAG, "Trying saved PSK SSID: %s", ssid);
    return wifi_connect_psk_wifi_connect_enterprise_nownow(ssid, pass, timeout_ms);
}

wifi_result_t wifi_connect_enterprise_now(const char *ssid,
                                          const char *user,
                                          const char *pass,
                                          const char *anon_opt,
                                          int timeout_ms)
{
    // set STA mode SSID
    if (!ssid || !*ssid || !user || !*user || !pass || !*pass) return WIFI_RES_FAIL;

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid) - 1);
    wc.sta.pmf_cfg.capable = true;
    wc.sta.pmf_cfg.required = false;

    xEventGroupClearBits(s_evt, GOT_IP_BIT);

    // start STA mode
    wifi_stop_safely();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());                    // STA_START -> handler -> try_connect()


    // Outer identity (anonymous) if provided; else use real username
    const char *outer = (anon_opt && *anon_opt) ? anon_opt : user;
    ESP_ERROR_CHECK(esp_eap_client_set_identity((const uint8_t *)outer, strlen(outer)));

    
    // configure EAP client
    // Inner credentials
    ESP_ERROR_CHECK(esp_eap_client_set_username((const uint8_t *)user, strlen(user)));
    ESP_ERROR_CHECK(esp_eap_client_set_password((const uint8_t *)pass, strlen(pass)));

    // If your network requires server CA validation, add:
    // ESP_ERROR_CHECK(esp_eap_client_set_ca_cert((const unsigned char*)ca_pem, strlen(ca_pem)));
    
    // enable enterprise mode
    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());

    // wait for ip
    EventBits_t b = xEventGroupWaitBits(
        s_evt, GOT_IP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));

    // disable enterprise if fail
    if (!(b & GOT_IP_BIT)) {
        esp_wifi_sta_enterprise_disable();
        return WIFI_RES_FAIL;
    }
    return WIFI_RES_OK;
}


wifi_result_t wifi_try_load_and_connect_auto(int timeout_ms) {
    // read "ent" from nonvolatile storage
    char ent[2] = {0};
    if (kv_get_str("ent", ent, sizeof(ent)) == 0 && ent[0] == '1') {
        char ssid[33] = {0}, user[65] = {0}, pass[65] = {0}, anid[65] = {0};

        
        // if "ent" == '1' try enterprise with "ssid" "e_user" "e_pass"
        // kv_get_str returns 0 on success; if any of these fail, skip to PSK
        if (kv_get_str("ssid",   ssid, sizeof(ssid)) == 0 &&
            kv_get_str("e_user", user, sizeof(user)) == 0 &&
            kv_get_str("e_pass", pass, sizeof(pass)) == 0)
        {
            // optional: e_anid credentials
            if (kv_get_str("e_anid", anid, sizeof(anid)) != 0) anid[0] = '\0';

            // try enterprise
            ESP_LOGI(TAG, "Trying Enterprise SSID: %s", ssid);
            wifi_result_t r = wifi_connect_enterprise_now(ssid, user, pass, anid, timeout_ms);
            if (r == WIFI_RES_OK) return r;

            // if enterprise fails fallback to PSK
            ESP_LOGW(TAG, "Enterprise failed; falling back to PSK (if present).");
        }
    }
    return wifi_try_load_and_connect_psk(timeout_ms);
}

// Creates AP config
void wifi_start_softap(const char *ap_ssid, const char *ap_pass) {
    // SSID/password parsing
    wifi_config_t ap = {0};
    strncpy((char *)ap.ap.ssid, ap_ssid, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid_len = strlen((char *)ap.ap.ssid);
    if (ap_pass) strncpy((char *)ap.ap.password, ap_pass, sizeof(ap.ap.password) - 1);
    // max 4 connections
    ap.ap.max_connection = 4;
    //open if no password, swap to WPA/WPA2 otherwise
    ap.ap.authmode = (ap_pass && *ap_pass) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;
    // fixed to channel 6
    ap.ap.channel = 6;

    // stop wi-fi safely
    wifi_stop_safely();

    //set mode AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    //apply config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    //start wifi
    ESP_ERROR_CHECK(esp_wifi_start());

    // display results
    ESP_LOGI(TAG, "SoftAP up: SSID=%s pass=%s",
             ap_ssid, (ap_pass && *ap_pass) ? ap_pass : "(open)");
}

// stops wifi
void wifi_stop_softap(void) {
    wifi_stop_safely();
}

//forget credentials
void wifi_forget_saved(void) {
    //disconnects STA mode
    ESP_LOGW(TAG, "Forgetting saved Wi-Fi credentials…");
    // Stop any Wi-Fi roles to avoid races while we modify NVS
    esp_wifi_disconnect();
    //stop softAP
    wifi_stop_softap();

    // delete all nonvolatile storage keys
    // Remove both PSK and Enterprise keys
    kv_del("ent");
    kv_del("ssid");
    kv_del("psk");
    kv_del("e_user");
    kv_del("e_pass");
    kv_del("e_anid");
    //commit the removal
    kv_commit();

    //removal message
    ESP_LOGW(TAG, "Wi-Fi credentials removed from NVS.");
}

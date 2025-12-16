//nvs_kv.c
//Wraps ESP-IDF's Non-Volatile Storage into simple key-value API
#include "nvs_kv.h"
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_kv";

//Handles NVS namespace
static nvs_handle_t s_nvs = 0;
//ensures NVS is initialized once
static bool s_open = false;

// Avoids re-initializing flash, reopening namespace repeatedly, and wasting memory resources/time

/* Ensures NVS is ready before any read/write occurs*/
static void kv_ensure_open(void)
{
    // Only run init logic once
    if (!s_open) {
        // sets the flash partition used by NVS
        esp_err_t err = nvs_flash_init();
        // handle corrupted / incompatible Nonvolatile Storage
        // Reasons:
        // -flash can fill up
        // -NVS format could change between firmware versions
        // -Erase and reinitialize for both cases.
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);

        // Opens a namespace
        // - Like a folder in flash
        // - Everything we save is under "store" keyword
        ESP_ERROR_CHECK(nvs_open("store", NVS_READWRITE, &s_nvs));
        // function skips init and uses handle instead bool flag set
        s_open = true;
        ESP_LOGI(TAG, "NVS opened");
    }
}

// function for application code to initialize to make it clean
esp_err_t kv_init(void)
{
    kv_ensure_open();
    return ESP_OK;
}

// reads a string safely.
// 1. how big is the value
// 2. checks if buffer size is sufficient
// 3. read value
int kv_get_str(const char *key, char *dst, size_t dst_len)
{
    kv_ensure_open();
    // does not read the string
    // returns how many bytes are required including null character
    size_t needed = 0;
    esp_err_t err = nvs_get_str(s_nvs, key, NULL, &needed);
    if (err != ESP_OK) return -1;

    // prevents buffer overflow
    if (needed > dst_len) return -1;

    // copies string
    err = nvs_get_str(s_nvs, key, dst, &needed);

    //returns either 0 (success) or 1 (fail)
    return (err == ESP_OK) ? 0 : -1;
}

//stores a string
int kv_set_str(const char *key, const char *value)
{
    kv_ensure_open();
    // Ternary Expression safety trick -> if value NULL store and empty string instead of crashing
    esp_err_t err = nvs_set_str(s_nvs, key, value ? value : "");
    return (err == ESP_OK) ? 0 : -1;
}

//deletes a key
esp_err_t kv_del(const char *key)
{
    kv_ensure_open();
    esp_err_t err = nvs_erase_key(s_nvs, key);
    //If key does not exist, it is already deleted.
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK; // treat missing as OK
    return err;
}

void kv_commit(void)
{
    kv_ensure_open();
    // batch multiple writes
    // reduce flash wear
    // recieve atomic updates -> atomic operations think add, store, load
    ESP_ERROR_CHECK(nvs_commit(s_nvs));
}

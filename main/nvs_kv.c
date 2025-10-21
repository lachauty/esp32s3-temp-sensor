//nvs_kv.c
#include "nvs_kv.h"
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_kv";

static nvs_handle_t s_nvs = 0;
static bool s_open = false;

/* Open NVS (namespace "store") once */
static void kv_ensure_open(void)
{
    if (!s_open) {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);

        ESP_ERROR_CHECK(nvs_open("store", NVS_READWRITE, &s_nvs));
        s_open = true;
        ESP_LOGI(TAG, "NVS opened");
    }
}

esp_err_t kv_init(void)
{
    kv_ensure_open();
    return ESP_OK;
}

int kv_get_str(const char *key, char *dst, size_t dst_len)
{
    kv_ensure_open();
    size_t needed = 0;
    esp_err_t err = nvs_get_str(s_nvs, key, NULL, &needed);
    if (err != ESP_OK) return -1;
    if (needed > dst_len) return -1;
    err = nvs_get_str(s_nvs, key, dst, &needed);
    return (err == ESP_OK) ? 0 : -1;
}

int kv_set_str(const char *key, const char *value)
{
    kv_ensure_open();
    esp_err_t err = nvs_set_str(s_nvs, key, value ? value : "");
    return (err == ESP_OK) ? 0 : -1;
}

esp_err_t kv_del(const char *key)
{
    kv_ensure_open();
    esp_err_t err = nvs_erase_key(s_nvs, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK; // treat missing as OK
    return err;
}

void kv_commit(void)
{
    kv_ensure_open();
    ESP_ERROR_CHECK(nvs_commit(s_nvs));
}

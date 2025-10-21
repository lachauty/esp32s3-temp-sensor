//nvs_kv.h
#pragma once
#include <stddef.h>
#include "esp_err.h"

/* Initialize/open NVS, returns ESP_OK so it works with ESP_ERROR_CHECK */
esp_err_t kv_init(void);

/* Return 0 on success, non-zero on failure. */
int  kv_get_str(const char *key, char *dst, size_t dst_len);
int  kv_set_str(const char *key, const char *value);
esp_err_t kv_del(const char *key);   // <-- NEW: erase a single key
void kv_commit(void);

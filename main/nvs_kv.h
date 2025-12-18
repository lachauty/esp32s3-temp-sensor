//nvs_kv.h
#pragma once
#include <stddef.h>
#include "esp_err.h"

/* Initialize/open NVS, returns ESP_OK so it works with ESP_ERROR_CHECK */
esp_err_t kv_init(void);

// Header definitions
// Key getter and setter
int  kv_get_str(const char *key, char *dst, size_t dst_len);
int  kv_set_str(const char *key, const char *value);
// Helper function to delete characters in key
esp_err_t kv_del(const char *key);
// helper function to commit a key
void kv_commit(void);

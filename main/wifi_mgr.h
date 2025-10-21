//wifi_mgr.h
#pragma once
#include <stdbool.h>

typedef enum { WIFI_RES_FAIL=0, WIFI_RES_OK=1 } wifi_result_t;

void wifi_netif_init_once(void);

wifi_result_t wifi_connect_psk_now(const char *ssid, const char *pass, int timeout_ms);
wifi_result_t wifi_try_load_and_connect_psk(int timeout_ms);

wifi_result_t wifi_connect_enterprise_now(const char *ssid,
                                          const char *user,
                                          const char *pass,
                                          const char *anon_opt,   // may be NULL/""
                                          int timeout_ms);

wifi_result_t wifi_try_load_and_connect_auto(int timeout_ms);

void wifi_start_softap(const char *ap_ssid, const char *ap_pass);
void wifi_stop_softap(void);

/* NEW: erase saved Wi-Fi credentials from NVS (PSK or Enterprise) */
void wifi_forget_saved(void);

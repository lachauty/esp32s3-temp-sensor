//portal.c
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_kv.h"     // kv_set_str(), kv_get_str(), kv_commit(), kv_del()
#include "wifi_mgr.h"   // wifi_forget_saved()

static const char *TAG = "portal";

/* ---------- Simple 1x1 transparent PNG for /favicon.ico (to silence 404s) ---------- */
static const uint8_t FAVICON_PNG[] = {
  0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A,0x00,0x00,0x00,0x0D,0x49,0x48,0x44,0x52,
  0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x08,0x06,0x00,0x00,0x00,0x1F,0x15,0xC4,
  0x89,0x00,0x00,0x00,0x0A,0x49,0x44,0x41,0x54,0x78,0x9C,0x63,0x00,0x01,0x00,0x00,
  0x05,0x00,0x01,0x0D,0x0A,0x2D,0xB4,0x00,0x00,0x00,0x00,0x49,0x45,0x4E,0x44,0xAE,
  0x42,0x60,0x82
};

/* ----------------------------------- HTML ----------------------------------- */
static const char FORM_HTML[] =
"<!doctype html><html><head><meta charset='utf-8'/>"
"<meta name='viewport' content='width=device-width, initial-scale=1'/>"
"<title>Freezer Monitor Setup</title>"
"<style>"
"body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu;max-width:720px;margin:24px auto;padding:0 16px}"
"h1{font-size:1.75rem;margin-bottom:1rem}"
"label{display:block;font-weight:600;margin:.75rem 0 .25rem}"
"input[type=text],input[type=password]{width:100%;padding:.65rem;border:1px solid #ccc;border-radius:8px;font-size:1rem}"
".row{display:grid;grid-template-columns:1fr auto;align-items:center;gap:.5rem}"
"button{margin-top:1rem;padding:.8rem 1.1rem;border:0;border-radius:10px;background:#111;color:#fff;font-weight:700;cursor:pointer}"
".hint{color:#444;margin-top:1rem;font-size:.95rem}"
".card{background:#fafafa;border:1px solid #eee;border-radius:12px;padding:16px}"
".danger{background:#fff5f5;border-color:#ffd7d7;color:#a40000}"
"</style>"
"</head><body>"
"<h1>Connect to Wi-Fi</h1>"
"<form class='card' method='POST' action='/submit'>"
"  <label>Wi-Fi SSID</label>"
"  <input id='ssid' name='ssid' type='text' placeholder='Your Wi-Fi name' required/>"
"  <label>Password (PSK)</label>"
"  <input id='psk' name='psk' type='password' placeholder='Leave empty for Enterprise'/>"
"  <div class='row' style='margin-top:.5rem'>"
"    <label style='margin:0'>WPA2-Enterprise (PEAP)</label>"
"    <input id='ent' name='ent' type='checkbox' value='1'/>"
"  </div>"
"  <label>Username</label>"
"  <input id='user' name='user' type='text'/>"
"  <label>Password</label>"
"  <input id='epass' name='epass' type='password'/>"
"  <label>Anonymous Identity (optional)</label>"
"  <input id='anid' name='anid' type='text' placeholder='anonymous'/>"
"  <button id='save' type='submit'>Save & Reboot</button>"
"  <p class='hint'>Tip: leave PSK blank and check Enterprise for campus networks.</p>"
"</form>"
"<form class='card danger' method='GET' action='/forget'>"
"  <h2 style='margin-top:0'>Trouble connecting?</h2>"
"  <p>This will clear saved Wi-Fi credentials and reboot into setup mode.</p>"
"  <button type='submit'>Forget Wi-Fi & Reboot</button>"
"</form>"
"<script>"
"function update(){"
"  var ent=document.getElementById('ent').checked;"
"  document.getElementById('user').disabled=!ent;"
"  document.getElementById('epass').disabled=!ent;"
"  document.getElementById('anid').disabled=!ent;"
"}"
"document.addEventListener('DOMContentLoaded', update);"
"document.getElementById('ent').addEventListener('change', update);"
"</script>"
"</body></html>";

/* -------------------------- tiny helpers: form parsing -------------------------- */

static int hexv(int c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

/* In-place URL decode of application/x-www-form-urlencoded */
static void urldecode(char *s) {
  char *o = s;
  while (*s) {
    if (*s == '+') { *o++ = ' '; s++; }
    else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      int hi = hexv(s[1]), lo = hexv(s[2]);
      if (hi >= 0 && lo >= 0) { *o++ = (char)((hi<<4) | lo); s += 3; }
      else { *o++ = *s++; }
    } else { *o++ = *s++; }
  }
  *o = '\0';
}

/* Extract a key=val pair from the urlencoded body. Returns length copied (<= dst_len-1) */
static int form_get(const char *body, const char *key, char *dst, size_t dst_len) {
  size_t klen = strlen(key);
  const char *p = body;
  while (p && *p) {
    const char *amp = strchr(p, '&');
    size_t seglen = amp ? (size_t)(amp - p) : strlen(p);
    if (seglen > klen + 1 && strncmp(p, key, klen) == 0 && p[klen] == '=') {
      size_t vlen = seglen - (klen + 1);
      if (vlen >= dst_len) vlen = dst_len - 1;
      memcpy(dst, p + klen + 1, vlen);
      dst[vlen] = '\0';
      urldecode(dst);
      return (int)vlen;
    }
    p = amp ? amp + 1 : NULL;
  }
  if (dst_len) dst[0] = '\0';
  return 0;
}

/* --------------------------------- Handlers --------------------------------- */

static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, FORM_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t favicon_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "image/png");
  return httpd_resp_send(req, (const char*)FAVICON_PNG, sizeof(FAVICON_PNG));
}

static void reboot_task(void *arg) {
  vTaskDelay(pdMS_TO_TICKS(300));   // let TCP finish
  esp_restart();
}

/* POST /submit */
static esp_err_t submit_post_handler(httpd_req_t *req) {
  /* Read body */
  int len = req->content_len;
  if (len <= 0 || len > 1024) len = 1024;     // guard
  char *buf = (char*)calloc(1, len + 1);
  if (!buf) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
  int rcv = 0;
  while (rcv < len) {
    int ret = httpd_req_recv(req, buf + rcv, len - rcv);
    if (ret <= 0) { free(buf); return ESP_OK; }
    rcv += ret;
  }
  buf[rcv] = '\0';

  /* Parse fields */
  char ssid[33]={0}, psk[65]={0}, user[65]={0}, epass[65]={0}, anid[65]={0}, ent_str[8]={0};
  form_get(buf, "ssid", ssid, sizeof(ssid));
  form_get(buf, "psk",  psk,  sizeof(psk));
  form_get(buf, "user", user, sizeof(user));
  form_get(buf, "epass",epass,sizeof(epass));
  form_get(buf, "anid", anid, sizeof(anid));
  form_get(buf, "ent",  ent_str, sizeof(ent_str));
  free(buf);

  bool ent = (ent_str[0] == '1' || ent_str[0] == 'o'); // checkbox = "1" or "on"
  ESP_LOGI(TAG, "SUBMIT: ssid='%s' ent=%d user_len=%d psk_len=%d",
           ssid, (int)ent, (int)strlen(user), (int)strlen(psk));

  /* Server-side validation */
  if (!ssid[0]) {
    httpd_resp_sendstr(req, "SSID required");
    return ESP_OK;
  }
  if (ent) {
    if (!user[0] || !epass[0]) {
      httpd_resp_sendstr(req, "Enterprise needs username+password");
      return ESP_OK;
    }
  } else {
    if (!psk[0]) {
      httpd_resp_sendstr(req, "PSK password required (or select Enterprise)");
      return ESP_OK;
    }
  }

  /* Save to NVS */
  if (ent) {
    kv_set_str("ent", "1");
    kv_set_str("ssid", ssid);
    kv_set_str("e_user", user);
    kv_set_str("e_pass", epass);
    kv_set_str("e_anid", anid);
  } else {
    kv_set_str("ent", "0");
    kv_set_str("ssid", ssid);
    kv_set_str("psk",  psk);
  }
  kv_commit();
  ESP_LOGI(TAG, "Saved credentials to NVS; rebooting into STA…");

  /* Reply then reboot */
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_sendstr(req, "Saved. The device will reboot now.");
  xTaskCreate(reboot_task, "reboot_task", 2048, NULL, 5, NULL);
  return ESP_OK;
}

/* GET /forget — clear Wi-Fi creds and reboot to provisioning */
static esp_err_t forget_get_handler(httpd_req_t *req) {
  wifi_forget_saved();
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_sendstr(req, "Wi-Fi credentials cleared. Rebooting into setup…");
  xTaskCreate(reboot_task, "reboot_task", 2048, NULL, 5, NULL);
  return ESP_OK;
}

/* ----------------------------- Public start API ----------------------------- */
void portal_start(void)
{
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.lru_purge_enable = true;
  cfg.max_uri_handlers = 9;
  cfg.server_port = 80;

  static httpd_handle_t s_srv = NULL;
  if (s_srv) {
    ESP_LOGW(TAG, "Portal already running");
    return;
  }

  if (httpd_start(&s_srv, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "httpd_start failed");
    return;
  }

  httpd_uri_t root =     { .uri="/",          .method=HTTP_GET,  .handler=root_get_handler,    .user_ctx=NULL };
  httpd_uri_t submit =   { .uri="/submit",    .method=HTTP_POST, .handler=submit_post_handler, .user_ctx=NULL };
  httpd_uri_t favicon =  { .uri="/favicon.ico", .method=HTTP_GET, .handler=favicon_get_handler, .user_ctx=NULL };
  httpd_uri_t forget =   { .uri="/forget",    .method=HTTP_GET,  .handler=forget_get_handler,  .user_ctx=NULL };

  httpd_register_uri_handler(s_srv, &root);
  httpd_register_uri_handler(s_srv, &submit);
  httpd_register_uri_handler(s_srv, &favicon);
  httpd_register_uri_handler(s_srv, &forget);

  ESP_LOGI(TAG, "Portal started at http://192.168.4.1/");
}

void portal_stop(void) {
  // Not strictly needed in your flow, but here for completeness
  // (You would need to keep the httpd handle if you want to stop it later)
}

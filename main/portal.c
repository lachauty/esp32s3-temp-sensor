//portal.c
// Web portal to setup wifi connection on esp32-s3
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

/* HTML code for SOFTAP setup */
static const char FORM_HTML[] =
"<!doctype html><html><head><meta charset='utf-8'/>"
"<meta name='viewport' content='width=device-width, initial-scale=1'/>"
"<title>Freezer Monitor Setup</title>"
"<style>"
"body{max-width:720px;margin:24px auto;padding:0 16px}"
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

/* Helper Functions */

// Helper function that decodes hexadecimal characters from packets sent to and from the internet
static int hexv(int c) {
  // Case 1: '0' - '9'
  if (c >= '0' && c <= '9') return c - '0';
  // Case 2: 'a' - 'f'
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  // Case 3: 'A' - 'F'
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

/* In-place URL decode of application/x-www-form-urlencoded */
static void urldecode(char *s) {
  char *o = s;
  while (*s) {
    if (*s == '+') { *o++ = ' '; s++; }
    else if (*s == '%' && s[1] && s[2] && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      
      // Extract hex values.
      int hi = hexv(s[1]), lo = hexv(s[2]);

      // Validate hex digits to ensure both characters are valid hex digits.
      // Combine the hex digits into one byte and writes the decoded character.
      if (hi >= 0 && lo >= 0) { *o++ = (char)((hi<<4) | lo); s += 3; }

      // Skips past the %XX sequence
      else { *o++ = *s++; }
    } else { *o++ = *s++; }
  }
  *o = '\0';
}

/* Extract a key=val pair from the urlencoded body. Returns length copied (<= dst_len-1) */
static int form_get(const char *body, const char *key, char *dst, size_t dst_len) {
  // Compute key length & set pointer to start
  size_t klen = strlen(key);
  const char *p = body;

  // loop over each key=value segment
  while (p && *p) {

    // Find the next & separator
    const char *amp = strchr(p, '&');
    size_t seglen = amp ? (size_t)(amp - p) : strlen(p);

    // Check if this segment matches the key
    if (seglen > klen + 1 && strncmp(p, key, klen) == 0 && p[klen] == '=') {

      // compute value length and copy safely
      size_t vlen = seglen - (klen + 1);
      if (vlen >= dst_len) vlen = dst_len - 1;
      memcpy(dst, p + klen + 1, vlen);
      dst[vlen] = '\0';

      urldecode(dst);
      return (int)vlen;
    }

    // If key not found, move to next segment
    p = amp ? amp + 1 : NULL;
  }
  // If not found at all
  if (dst_len) dst[0] = '\0';
  return 0;
}

/* Handlers */
// Allows us to open a browser to ESP32 ip
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, FORM_HTML, HTTPD_RESP_USE_STRLEN);
}

// we need to reboot after sending a HTTP response
static void reboot_task(void *arg) {
  vTaskDelay(pdMS_TO_TICKS(300));   // let TCP finish
  esp_restart();
}

/* POST /submit */
static esp_err_t submit_post_handler(httpd_req_t *req) {
  /* Read body */
  // Read the POST body into a buffer
  // req->content_len = how many bytes the client says when sending the POST body
  // if it's empty reject with 400 Bad Request
  // if it's too big reject 413 payload too large
  int len = req->content_len;
  
  if(len <= 0){
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
  }
  if(len > 1024){
    return httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Too large");
  }

  // Allocate a buffer and read the body
  // Allocate len + 1 to null terminate
  // calloc zero fills memory for safety
  // return 500 internal server error if allocation fails ("Out of Memory")
  char *buf = (char*)calloc(1, len + 1);
  if(!buf) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
  
  // httpd_req_recv() returns body in chunks
  // while loop keeps reading until len bytes
  // error or client closed -> free buffer & exit
  // buf[rcv] = '\0' null terminates making a c string
  int rcv = 0;
  while (rcv < len) {
    int ret = httpd_req_recv(req, buf + rcv, len - rcv);
    if (ret <= 0) { free(buf); return ESP_OK; }
    rcv += ret;
  }
  buf[rcv] = '\0';

  /* Extract form fields from the body */
  // Fixed-size destination buffers for Wi-Fi limits
  // Free buffer after parsing fields
  char ssid[33]={0}, psk[65]={0}, user[65]={0}, epass[65]={0}, anid[65]={0}, ent_str[8]={0};
  form_get(buf, "ssid", ssid, sizeof(ssid));
  form_get(buf, "psk",  psk,  sizeof(psk));
  form_get(buf, "user", user, sizeof(user));
  form_get(buf, "epass",epass,sizeof(epass));
  form_get(buf, "anid", anid, sizeof(anid));
  form_get(buf, "ent",  ent_str, sizeof(ent_str));
  free(buf);

  // Enterprise checkbox
  // if checked, browser submits ent=1 or on
  // then display a summary of collected user info
  bool ent = (ent_str[0] == '1' || ent_str[0] == 'o'); // checkbox = "1" or "on"
  ESP_LOGI(TAG, "SUBMIT: ssid='%s' ent=%d user_len=%d psk_len=%d",
           ssid, (int)ent, (int)strlen(user), (int)strlen(psk));

  /* Checks inputs on the server for ssid */
  if (!ssid[0]) {
    httpd_resp_sendstr(req, "SSID required");
    return ESP_OK;
  }
  if (ent) {
    // if enterprise mode check for username and password
    if (!user[0] || !epass[0]) {
      httpd_resp_sendstr(req, "Enterprise needs username+password");
      return ESP_OK;
    }
  } else {
    // if normal WPA2-PSK
    if (!psk[0]) {
      httpd_resp_sendstr(req, "PSK password required (or select Enterprise)");
      return ESP_OK;
    }
  }

  /* Save to NVS */
  // ent is mode flag
  // 1 configs WPA2-Enterprise
  // 0 configs normal Wi-Fi PSK
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
  // commit makes sure it's written to flash
  kv_commit();
  ESP_LOGI(TAG, "Saved credentials to NVS; rebooting into STA…");

  /* Reply then reboot */
  // send a response first so the browser doesn't get a connection reset
  // wait for 300 ms and calls esp_restart
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_sendstr(req, "Saved. The device will reboot now.");
  xTaskCreate(reboot_task, "reboot_task", 2048, NULL, 5, NULL);
  return ESP_OK;
}

/* GET /forget — clear Wi-Fi creds and reboot to provisioning */
static esp_err_t forget_get_handler(httpd_req_t *req) {
  //erase wi-fi related keys from NVS
  //resets internal flags for the device to go back to SoftAP provisioning mode
  wifi_forget_saved();

  //Send a response to the browser
  //sets response MIME type to plain text
  //sends a human-readable confirmation
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_sendstr(req, "Wi-Fi credentials cleared. Rebooting into setup…");
  
  //reboot safely and finish handler
  xTaskCreate(reboot_task, "reboot_task", 2048, NULL, 5, NULL);
  return ESP_OK;
}

// Bring web server online
void portal_start(void)
{
  // Create HTTP server config
  // Stack size, task priority, max open sockets, timeouts
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();

  // Least recently used socket purge
  // oldest idle is closed
  // prevents the server from getting stuck
  cfg.lru_purge_enable = true;
  // max number of routes server can register
  // we use / /submit and /forget
  cfg.max_uri_handlers = 9;
  // standard http port
  cfg.server_port = 80;

  // static -> persists across function calls
  // prevents starting multiple http servers
  // avoids port binding errors and crashes
  static httpd_handle_t s_srv = NULL;
  if (s_srv) {
    ESP_LOGW(TAG, "Portal already running");
    return;
  }

  // launches http server tasks
  // fails if you log an error and bail
  if (httpd_start(&s_srv, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "httpd_start failed");
    return;
  }

  // Form submission for root, submit, and forget
  httpd_uri_t root =     { .uri="/",          .method=HTTP_GET,  .handler=root_get_handler,    .user_ctx=NULL };
  httpd_uri_t submit =   { .uri="/submit",    .method=HTTP_POST, .handler=submit_post_handler, .user_ctx=NULL };
  httpd_uri_t forget =   { .uri="/forget",    .method=HTTP_GET,  .handler=forget_get_handler,  .user_ctx=NULL };

  // Register handlers with the server
  httpd_register_uri_handler(s_srv, &root);
  httpd_register_uri_handler(s_srv, &submit);
  httpd_register_uri_handler(s_srv, &forget);

  ESP_LOGI(TAG, "Portal started at http://192.168.4.1/");
}


void portal_stop(void) {
  // Not strictly needed in your flow, but here for completeness
  // (You would need to keep the httpd handle if you want to stop it later)
}

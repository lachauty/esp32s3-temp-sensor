#include "max31856.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAX31856_DRV";
static spi_device_handle_t s_dev = NULL;

// Calibration & sanity window (driver-local)
#define CALIBRATION_OFFSET 0.0f
#define TEMP_MIN_C  (-100.0f)
#define TEMP_MAX_C  (100.0f)

// MAX31856 registers
#define REG_CR0      0x00
#define REG_CR1      0x01
#define REG_MASK     0x02
#define REG_CJHF     0x03
#define REG_CJLF     0x04
#define REG_LTHFTH   0x05
#define REG_LTHFTL   0x06
#define REG_LTLFTH   0x07
#define REG_LTLFTL   0x08
#define REG_CJTO     0x09
#define REG_CJTH     0x0A
#define REG_CJTL     0x0B
#define REG_LTCBH    0x0C
#define REG_LTCBM    0x0D
#define REG_LTCBL    0x0E
#define REG_SR       0x0F

// CR0 bits
#define CR0_CMODE    (1u << 7)
#define CR0_1SHOT    (1u << 6)
#define CR0_OCFAULT1 (1u << 5)
#define CR0_OCFAULT0 (1u << 4)
#define CR0_CJDIS    (1u << 3)
#define CR0_FAULTMOD (1u << 2)
#define CR0_FAULTCLR (1u << 1)
#define CR0_FILT50HZ (1u << 0)

// SR bits
#define SR_OPEN     (1u << 0)
#define SR_OVUV     (1u << 1)
#define SR_TCLOW    (1u << 2)
#define SR_TCHIGH   (1u << 3)
#define SR_CJLOW    (1u << 4)
#define SR_CJHIGH   (1u << 5)
#define SR_TCRANGE  (1u << 6)
#define SR_CJRANGE  (1u << 7)

// ---------- Low-level SPI helpers ----------
static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t tx[2] = { (uint8_t)(0x80 | (reg & 0x7F)), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_transmit(s_dev, &t);
}

static esp_err_t read_regs(uint8_t start_reg, uint8_t *dst, size_t n) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    if (!dst || n == 0 || n > 32) return ESP_ERR_INVALID_ARG;

    uint8_t tx[1 + 32] = {0};
    uint8_t rx[1 + 32] = {0};
    tx[0] = start_reg & 0x7F; // A7=0 → read

    spi_transaction_t t = {
        .length = (1 + n) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    esp_err_t err = spi_device_transmit(s_dev, &t);
    if (err != ESP_OK) return err;
    memcpy(dst, &rx[1], n); // skip first dummy byte
    return ESP_OK;
}

static uint8_t read_reg1(uint8_t reg) {
    uint8_t v = 0;
    if (read_regs(reg, &v, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed", reg);
    }
    return v;
}

static void log_faults(uint8_t sr) {
    if (!sr) return;
    ESP_LOGW(TAG, "Fault SR=0x%02X%s%s%s%s%s%s%s%s",
             sr,
             (sr & SR_OPEN)     ? " OPEN"     : "",
             (sr & SR_OVUV)     ? " OVUV"     : "",
             (sr & SR_TCLOW)    ? " TCLOW"    : "",
             (sr & SR_TCHIGH)   ? " TCHIGH"   : "",
             (sr & SR_CJLOW)    ? " CJLOW"    : "",
             (sr & SR_CJHIGH)   ? " CJHIGH"   : "",
             (sr & SR_TCRANGE)  ? " TCRANGE"  : "",
             (sr & SR_CJRANGE)  ? " CJRANGE"  : "");
}

// ---------- Public API ----------
void max31856_attach(spi_device_handle_t dev) {
    s_dev = dev;
}

void max31856_init(void) {
    // Wide thresholds
    write_reg(REG_CJHF,   0x7F); // +127°C
    write_reg(REG_CJLF,   0xC0); // -64°C
    write_reg(REG_LTHFTH, 0x7F); // TC high max
    write_reg(REG_LTHFTL, 0xFF);
    write_reg(REG_LTLFTH, 0x80); // TC low min
    write_reg(REG_LTLFTL, 0x00);

    // Cold-junction offset = 0
    write_reg(REG_CJTO, 0x00);

    // Continuous, 60 Hz (bit0=0), T-type + AVG=2
    write_reg(REG_CR0, CR0_CMODE);      // 0x80
    write_reg(REG_CR1, 0x10 | 0x07);    // AVG=2 | T-type

    vTaskDelay(pdMS_TO_TICKS(50));

    // Sanity log
    uint8_t cr0 = read_reg1(REG_CR0);
    uint8_t cr1 = read_reg1(REG_CR1);
    uint8_t cjhf = read_reg1(REG_CJHF), cjlf = read_reg1(REG_CJLF);
    uint8_t thh  = read_reg1(REG_LTHFTH), thl = read_reg1(REG_LTHFTL);
    uint8_t tlh  = read_reg1(REG_LTLFTH), tll = read_reg1(REG_LTLFTL);
    ESP_LOGI(TAG, "Init OK: CR0=0x%02X CR1=0x%02X | CJHF=0x%02X CJLF=0x%02X | TCH=0x%02X%02X TCL=0x%02X%02X",
             cr0, cr1, cjhf, cjlf, thh, thl, tlh, tll);
}

bool max31856_get_temp_c(float *out_c, uint8_t *out_sr) {
    if (!out_c) return false;

    uint8_t sr = 0;
    if (read_regs(REG_SR, &sr, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Read SR failed");
        return false;
    }
    log_faults(sr);
    if (out_sr) *out_sr = sr;

    uint8_t buf[3] = {0};
    if (read_regs(REG_LTCBH, buf, 3) != ESP_OK) {
        ESP_LOGE(TAG, "Read temp bytes failed");
        return false;
    }

    int32_t raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    raw >>= 5;                        // 19-bit value
    if (raw & 0x40000) raw |= 0xFFF80000; // sign-extend to 32-bit

    float t = (float)raw * 0.0078125f + CALIBRATION_OFFSET; // 1/128 °C
    if (t < TEMP_MIN_C || t > TEMP_MAX_C) {
        ESP_LOGW(TAG, "Temperature %.2f°C outside sanity window (%.1f..%.1f)!", t, TEMP_MIN_C, TEMP_MAX_C);
    }
    *out_c = t;
    return true;
}

void max31856_read_cj_debug(void) {
    uint8_t b[2];
    if (read_regs(REG_CJTH, b, 2) == ESP_OK) {
        int16_t cj_raw = ((int16_t)b[0] << 8) | b[1];
        cj_raw >>= 2; // 14-bit 1/16°C
        float cj_c = cj_raw / 16.0f;
        ESP_LOGI(TAG, "CJ Temp: %.2f°C", cj_c);
    }
}

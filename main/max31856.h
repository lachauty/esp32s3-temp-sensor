// max31856.h
// Header file for max31856 interface
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Provide the SPI device handle from app_main after spi_bus_add_device()
void max31856_attach(spi_device_handle_t dev);

// Configure MAX31856 (wide thresholds, T-type, AVG=2, 60 Hz)
void max31856_init(void);

// Read thermocouple temperature (°C). Returns true on success.
// Writes fault status register to *out_sr if non-NULL.
bool max31856_get_temp_c(float *out_c, uint8_t *out_sr);

// read cold-junction temp (guarded in .c)
void max31856_read_cj_debug(void);

#ifdef __cplusplus
}
#endif

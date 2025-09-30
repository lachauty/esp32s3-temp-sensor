#pragma once
#include "esp_err.h"

/* Start the provisioning portal (no callback; the portal reboots itself) */
void portal_start(void);
void portal_stop(void);

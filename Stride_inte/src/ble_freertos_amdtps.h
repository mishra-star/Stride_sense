//*****************************************************************************
//
//! @file ble_freertos_amdtps.h
//!
//! @brief Global includes for the BMP390 + BMI323 BLE telemetry app.
//!
//! Packet layout (CSV string format):
//!   "timestamp,temp,pressure,altitude,ax,ay,az,gx,gy,gz"
//!   Example: "1000,25.50,101325,612.50,1.000,0.000,0.000,0.0,0.0,0.0"
//!   Max length: ~80 bytes
//!
//! NOTE: radio_task.h is intentionally NOT included here.
//! Include it directly in .c files that need it, AFTER WSF headers.
//
//*****************************************************************************

#ifndef FREERTOS_AMDTP_H
#define FREERTOS_AMDTP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// AmbiqSuite HAL
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "portable.h"
#include "semphr.h"
#include "event_groups.h"

//*****************************************************************************
// BLE telemetry packet — CSV string format sent over AMDTP.
// Max length: 80 bytes (fits in a single ATT notification with DLE).
//*****************************************************************************
#define TELEMETRY_STRING_MAX_LEN 80

typedef char telemetry_string_t[TELEMETRY_STRING_MAX_LEN];

// Legacy alias for compatibility
typedef telemetry_string_t bmp_bmi_ble_packet_t;
typedef telemetry_string_t bmp390_ble_packet_t;

#endif // FREERTOS_AMDTP_H
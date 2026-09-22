//*****************************************************************************
//
//! @file rtos.h
//!
//! @brief Shared telemetry globals and RTOS entry points for the
//!        BMP390 (I2C) + BMI323 (SPI) BLE telemetry project.
//!
//! Data flow:
//!   SensorTask (rtos.c) reads BMP390 every 50 ms and BMI323 every 50 ms,
//!   packs results as a 24-byte binary telemetry_packet_t into
//!   g_sTelemetryPacket, then sets g_bNewTelemetryAvailable.
//!
//!   RadioTask (radio_task.c/Stride_inte.c) waits for g_bTxSubscribed
//!   (CCC=NOTIFY) then forwards the packet via AmdtpsSendPacket() and
//!   clears g_bNewTelemetryAvailable.
//!
//! FIX SUMMARY (vs. original)
//! --------------------------
//!   1. Added telemetry_packet_t — the binary struct that rtos.c actually
//!      packs and radio_task.c / Stride_inte.c actually send.  The original
//!      code declared g_sTelemetryPacket as char[80] (CSV) in the header but
//!      used it as a typed binary struct everywhere else, causing a type
//!      mismatch that silently produces garbage BLE packets.
//!
//!   2. Corrected g_sTelemetryPacket extern type to volatile telemetry_packet_t.
//!
//!   3. Removed the stale TELEMETRY_STRING_MAX_LEN / char-array alias — the
//!      CSV format was never used in any .c file.
//
//*****************************************************************************

#ifndef RTOS_H
#define RTOS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Binary telemetry packet — 24 bytes, sent as-is over AMDTP.
//
// Encoding conventions (minimise RAM, maximise precision over BLE):
//   Temperature  : int16_t, unit = 0.01 °C  → ±327.67 °C range, 0.01 °C res
//   Pressure     : uint32_t, unit = 1 Pa     → matches BMP390 raw output
//   Altitude     : int16_t, unit = 0.01 m   → ±327.67 m relative range
//   Accel        : int16_t, unit = 0.001 g  → ±32.767 g (saturates at ±16 g)
//   Gyro         : int16_t, unit = 0.01 dps → ±327.67 dps (saturates at ±250 dps,
//                                              change multiplier for 2000 dps range)
//   Timestamp    : uint32_t FreeRTOS tick count (1 ms per tick default)
//
// Total size: 4 + 2 + 4 + 2 + 2+2+2 + 2+2+2 = 24 bytes
//*****************************************************************************
typedef struct __attribute__((packed))
{
    uint32_t ui32Timestamp;     //!< FreeRTOS xTaskGetTickCount() at sample time
    int16_t  i16TempCx100;     //!< Temperature × 100 (°C × 100)
    uint32_t ui32PressurePa;   //!< Pressure in Pascals
    int16_t  i16AltMx100;      //!< Altitude × 100 (m × 100, relative to baseline)
    int16_t  i16AxGx1000;      //!< Accel X × 1000 (g × 1000)
    int16_t  i16AyGx1000;      //!< Accel Y × 1000
    int16_t  i16AzGx1000;      //!< Accel Z × 1000
    int16_t  i16GxDpsx100;     //!< Gyro X × 100 (dps × 100)
    int16_t  i16GyDpsx100;     //!< Gyro Y × 100
    int16_t  i16GzDpsx100;     //!< Gyro Z × 100
} telemetry_packet_t;

// Compile-time size guard — update assertion if struct changes
_Static_assert(sizeof(telemetry_packet_t) == 24,
               "telemetry_packet_t must be exactly 24 bytes");

//*****************************************************************************
// Shared telemetry globals  (defined in rtos.c)
//
//   g_sTelemetryPacket       — latest sensor sample (binary struct), written
//                              by SensorTask, read by RadioTask before AMDTP TX.
//   g_bNewTelemetryAvailable — flag: SensorTask sets after packing; RadioTask
//                              clears after queuing the AMDTP packet.
//*****************************************************************************
extern volatile telemetry_packet_t g_sTelemetryPacket;
extern volatile bool               g_bNewTelemetryAvailable;

//*****************************************************************************
// BLE subscription flag (defined in radio_task.c)
//   g_bTxSubscribed — set when phone has written CCC=NOTIFY; cleared on
//                     disconnect/unsubscribe.
//*****************************************************************************
extern volatile bool g_bTxSubscribed;

//*****************************************************************************
// RTOS entry-point called from main()
//*****************************************************************************
extern void run_tasks(void);

//*****************************************************************************
// Tickless-idle hooks (wired into FreeRTOS via FreeRTOSConfig.h)
//*****************************************************************************
extern uint32_t am_freertos_sleep(uint32_t idleTime);
extern void     am_freertos_wakeup(uint32_t idleTime);

#ifdef __cplusplus
}
#endif

#endif // RTOS_H
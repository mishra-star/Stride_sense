//*****************************************************************************
//
//! @file radio_task.h
//!
//! @brief Public interface for the BLE radio task (AMDTP server).
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk5p1p0-609aff2828 of the AmbiqSuite SDK.
//
//*****************************************************************************

#ifndef RADIO_TASK_H
#define RADIO_TASK_H

#include <stdbool.h>

/* FreeRTOS — needed for TaskHandle_t */
#include "FreeRTOS.h"
#include "task.h"

//*****************************************************************************
//
// FreeRTOS task handle — created by setup_task in rtos.c.
//
//*****************************************************************************
extern TaskHandle_t radio_task_handle;

//*****************************************************************************
//
// BLE connection state flag.
//
// Set to true  inside RadioTask when the AMDTP DM_CONN_OPEN_IND event fires.
// Set to false inside RadioTask when the DM_CONN_CLOSE_IND event fires.
//
// SensorTask (or any other task) may read this flag to decide whether to
// bother preparing a telemetry packet — but it is not mandatory.
//
//*****************************************************************************
extern volatile bool g_bBleConnected;

//*****************************************************************************
//
// AMDTP TX-subscription flag.
//
// true  = phone has written CCC = NOTIFY on the AMDTP TX characteristic.
//         AMDTP stack is ready to send notifications.
// false = phone connected but NOT yet subscribed, OR disconnected.
//         AmdtpsSendPacket() returns error 7 ("not ready for notification")
//         if called while this is false, so RadioTask must gate sending on
//         this flag rather than on g_bBleConnected alone.
//
// Set by:    amdtpProcCccState() in ble_freertos_amdtps.c when CCC=NOTIFY.
// Cleared by: same function on unsubscribe, and on DM_CONN_OPEN_IND /
//             DM_CONN_CLOSE_IND.
//
//*****************************************************************************
extern volatile bool g_bTxSubscribed;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void RadioTaskSetup(void);
extern void RadioTask(void *pvParameters);

#endif // RADIO_TASK_H
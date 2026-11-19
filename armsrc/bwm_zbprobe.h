//-----------------------------------------------------------------------------
// Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE.txt for the text of the license.
//-----------------------------------------------------------------------------
// EXPERIMENTAL BWM ESP32-C2 BLE PHY / 802.15.4 probe (PM5 / AT32 side).
//
// Drives bwm_cmd() against the BWM-side app_zb_probe commands. See
// CMD_PM5_BWM_ZBPROBE in pm3_cmd.h and Proxmark5_BWM_esp32's
// components/app_zb_probe for what's actually being tested and why this
// is not expected to produce decoded Zigbee frames yet.
//-----------------------------------------------------------------------------
#ifndef BWM_ZBPROBE_H
#define BWM_ZBPROBE_H

#include "common.h"
#include "pm3_cmd.h"   // bwm_zbprobe_regs_t

// app_com command codes (authoritative, from BWM main/app_com_defs.h)
#define BWM_CMD_ZB_PROBE_START    6000   // req: access_addr(u32 LE) + ble_channel(u8) + duration_s(u16 LE); resp: status(u8)
#define BWM_CMD_ZB_PROBE_STOP     6001   // resp: status(u8)
#define BWM_CMD_ZB_PROBE_GET_REGS 6002   // resp: bwm_zbprobe_regs_t wire layout (70 bytes)

// ble_channel is a real BLE Core Spec RF channel index (f = 2402 + 2*ch MHz,
// ch 0-39), not a guess. duration_s is clamped to 300 on the BWM (0 also
// means 300) so the probe never holds the BLE controller unattended forever.
int bwm_zbprobe_start(uint32_t access_addr, uint8_t ble_channel, uint16_t duration_s);
int bwm_zbprobe_stop(void);
int bwm_zbprobe_get_regs(bwm_zbprobe_regs_t *out);

#endif

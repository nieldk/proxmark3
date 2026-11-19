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
// EXPERIMENTAL BWM ESP32-C2 BLE PHY / 802.15.4 probe. See bwm_zbprobe.h.
//-----------------------------------------------------------------------------
#include "bwm_zbprobe.h"
#include "bwm_wifi.h"   // bwm_cmd()
#include "string.h"

int bwm_zbprobe_start(uint32_t access_addr, uint8_t ble_channel, uint16_t duration_s) {
    uint8_t req[7];
    memcpy(req + 0, &access_addr, 4);
    req[4] = ble_channel;
    memcpy(req + 5, &duration_s, 2);
    uint8_t status = 0xFF;
    uint16_t len = sizeof(status);
    int res = bwm_cmd(BWM_CMD_ZB_PROBE_START, req, sizeof(req), &status, &len, 2000);
    if (res != PM3_SUCCESS) {
        return res;
    }
    return (status == 0) ? PM3_SUCCESS : PM3_EFAILED;
}

int bwm_zbprobe_stop(void) {
    uint8_t status = 0xFF;
    uint16_t len = sizeof(status);
    int res = bwm_cmd(BWM_CMD_ZB_PROBE_STOP, NULL, 0, &status, &len, 2000);
    if (res != PM3_SUCCESS) {
        return res;
    }
    return (status == 0) ? PM3_SUCCESS : PM3_EFAILED;
}

int bwm_zbprobe_get_regs(bwm_zbprobe_regs_t *out) {
    memset(out, 0, sizeof(*out)); // caller sends *out back regardless of our return value
    uint8_t wire[70];
    uint16_t len = sizeof(wire);
    int res = bwm_cmd(BWM_CMD_ZB_PROBE_GET_REGS, NULL, 0, wire, &len, 2000);
    if (res != PM3_SUCCESS) {
        return res;
    }
    if (len != sizeof(wire)) {
        return PM3_EFAILED;
    }
    memcpy(&out->reg_freq,   wire +  0, 4);
    memcpy(&out->reg_mode_a, wire +  4, 4);
    memcpy(&out->reg_mode_b, wire +  8, 4);
    memcpy(&out->reg_whiten, wire + 12, 4);
    memcpy(&out->reg_aa_lo,  wire + 16, 4);
    memcpy(&out->reg_aa_hi,  wire + 20, 4);
    memcpy(&out->poll_count, wire + 24, 4);
    memcpy(&out->rxbuf_ptr,  wire + 28, 4);
    memcpy(&out->rxbuf_aux,  wire + 32, 4);
    out->rxbuf_in_dram = wire[36];
    memcpy(out->rxbuf_snapshot, wire + 37, BWM_ZBPROBE_RXBUF_SNAPSHOT_LEN);
    out->running = wire[37 + BWM_ZBPROBE_RXBUF_SNAPSHOT_LEN];
    return PM3_SUCCESS;
}

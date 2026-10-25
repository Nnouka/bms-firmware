/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MQTT_PAYLOAD_H_
#define MQTT_PAYLOAD_H_

/**
 * @file
 * @brief Builds the MQTT JSON payloads of docs/mqtt-schema.md (schema v1).
 *
 * Plain C, no Zephyr headers, so it can be unit-tested on a PC against the Python mapping in
 * services/wifi-sim/mapping.py (the rules here must stay identical to that file).
 */

#include <stddef.h>
#include <stdint.h>

/** Cell channels at or below this voltage are "no cell connected" (same rule as the relay). */
#define MQTT_PAYLOAD_NOT_CONNECTED_V 0.5f

#define MQTT_PAYLOAD_ERR_NO_CELLS (-1)  /* valid sample, but no battery cell connected */
#define MQTT_PAYLOAD_ERR_INVALID  (-2)  /* NaN/inf value */
#define MQTT_PAYLOAD_ERR_SIZE     (-3)  /* output buffer too small */

struct mqtt_meas_sample
{
    int state;                  /* enum bms_state: 0 OFF, 1 CHG, 2 DIS, 3 NORMAL, 4 SHUTDOWN */
    float pack_voltage_v;
    float pack_current_a;       /* positive = charging */
    float soc_pct;
    const float *cell_voltages_v;
    int num_cell_channels;
    const float *cell_temps_c;
    int num_temps;
    float mosfet_temp_c;
    uint32_t balancing_mask;
    uint32_t error_flags;       /* BMS_ERR_* bits */
};

struct mqtt_info_data
{
    const char *manufacturer;
    const char *model;
    const char *firmware;
    const char *chemistry;
    int cell_count;
    float capacity_ah;
    float cell_overvoltage_v;
    float cell_undervoltage_v;
    float chg_max_temp_c;
    float dis_max_temp_c;
    float temp_hysteresis_c;
    float cell_failure_diff_v;
};

/** Seconds since 1970 to "YYYY-MM-DDTHH:MM:SSZ". `buf` needs 21 bytes. */
void mqtt_payload_iso8601(int64_t epoch_s, char *buf, size_t len);

/** Build the `meas` payload. Returns the length, or a negative MQTT_PAYLOAD_ERR_*. */
int mqtt_payload_meas(char *buf, size_t len, const char *device_id, const char *ts,
                      const struct mqtt_meas_sample *s);

/** Build the `info` payload. Returns the length, or a negative MQTT_PAYLOAD_ERR_*. */
int mqtt_payload_info(char *buf, size_t len, const char *device_id, const struct mqtt_info_data *d);

#endif /* MQTT_PAYLOAD_H_ */

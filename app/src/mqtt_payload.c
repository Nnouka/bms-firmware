/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt_payload.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#define SCHEMA_VERSION 1

static const char *const state_names[] = {"OFF", "CHG", "DIS", "NORMAL", "SHUTDOWN"};

/* index = bit number of BMS_ERR_* in bms_common.h; order is the schema's error list */
static const char *const error_names[] = {
    "CELL_UNDERVOLTAGE", "CELL_OVERVOLTAGE", "SHORT_CIRCUIT", "DIS_OVERCURRENT",
    "CHG_OVERCURRENT",   "OPEN_WIRE",        "DIS_UNDERTEMP", "DIS_OVERTEMP",
    "CHG_UNDERTEMP",     "CHG_OVERTEMP",     "INT_OVERTEMP",  "CELL_FAILURE",
    "DIS_OFF",           "CHG_OFF",          "FET_OVERTEMP",
};
#define NUM_ERROR_NAMES (sizeof(error_names) / sizeof(error_names[0]))

/* Appends to buf; remembers overflow so the caller checks once at the end. */
struct out
{
    char *buf;
    size_t len;
    size_t pos;
    int err;
};

static void put(struct out *o, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void put(struct out *o, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (o->err) {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(o->buf + o->pos, o->len - o->pos, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= o->len - o->pos) {
        o->err = MQTT_PAYLOAD_ERR_SIZE;
        return;
    }
    o->pos += (size_t)n;
}

static void put_num(struct out *o, float v, int decimals)
{
    if (!isfinite(v)) {
        o->err = MQTT_PAYLOAD_ERR_INVALID;
        return;
    }
    put(o, "%.*f", decimals, (double)v);
}

void mqtt_payload_iso8601(int64_t epoch_s, char *buf, size_t len)
{
    /* Howard Hinnant's civil_from_days */
    int64_t days = epoch_s / 86400;
    int64_t rem = epoch_s % 86400;
    if (rem < 0) {
        rem += 86400;
        days--;
    }
    days += 719468;
    int64_t era = (days >= 0 ? days : days - 146096) / 146097;
    int64_t doe = days - era * 146097;
    int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t y = yoe + era * 400;
    int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int64_t mp = (5 * doy + 2) / 153;
    int64_t d = doy - (153 * mp + 2) / 5 + 1;
    int64_t m = mp < 10 ? mp + 3 : mp - 9;
    if (m <= 2) {
        y++;
    }
    snprintf(buf, len, "%04d-%02d-%02dT%02d:%02d:%02dZ", (int)y, (int)m, (int)d, (int)(rem / 3600),
             (int)(rem % 3600 / 60), (int)(rem % 60));
}

int mqtt_payload_meas(char *buf, size_t len, const char *device_id, const char *ts,
                      const struct mqtt_meas_sample *s)
{
    struct out o = {.buf = buf, .len = len};
    int cells = 0;
    int first;

    for (int i = 0; i < s->num_cell_channels; i++) {
        if (s->cell_voltages_v[i] > MQTT_PAYLOAD_NOT_CONNECTED_V) {
            cells++;
        }
    }
    if (cells == 0) {
        return MQTT_PAYLOAD_ERR_NO_CELLS;
    }
    if (s->state < 0 || s->state >= (int)(sizeof(state_names) / sizeof(state_names[0]))) {
        return MQTT_PAYLOAD_ERR_INVALID;
    }

    float soc = s->soc_pct;
    if (isfinite(soc)) {
        soc = soc < 0.0f ? 0.0f : (soc > 100.0f ? 100.0f : soc);
    }

    put(&o, "{\"schema\":%d,\"device_id\":\"%s\",\"ts\":\"%s\",\"state\":\"%s\"", SCHEMA_VERSION,
        device_id, ts, state_names[s->state]);
    put(&o, ",\"pack_voltage_v\":");
    put_num(&o, s->pack_voltage_v, 2);
    put(&o, ",\"pack_current_a\":");
    put_num(&o, s->pack_current_a, 2);
    put(&o, ",\"soc_pct\":");
    put_num(&o, soc, 1);

    put(&o, ",\"cell_voltages_v\":[");
    first = 1;
    for (int i = 0; i < s->num_cell_channels; i++) {
        if (s->cell_voltages_v[i] > MQTT_PAYLOAD_NOT_CONNECTED_V) {
            put(&o, first ? "" : ",");
            put_num(&o, s->cell_voltages_v[i], 3);
            first = 0;
        }
    }
    put(&o, "],\"cell_temps_c\":[");
    for (int i = 0; i < s->num_temps; i++) {
        put(&o, i ? "," : "");
        put_num(&o, s->cell_temps_c[i], 1);
    }
    put(&o, "],\"mosfet_temp_c\":");
    put_num(&o, s->mosfet_temp_c, 1);
    put(&o, ",\"balancing_mask\":%u,\"errors\":[", (unsigned int)s->balancing_mask);
    first = 1;
    for (size_t b = 0; b < NUM_ERROR_NAMES; b++) {
        if (s->error_flags >> b & 1U) {
            put(&o, "%s\"%s\"", first ? "" : ",", error_names[b]);
            first = 0;
        }
    }
    put(&o, "]}");

    return o.err ? o.err : (int)o.pos;
}

int mqtt_payload_info(char *buf, size_t len, const char *device_id, const struct mqtt_info_data *d)
{
    struct out o = {.buf = buf, .len = len};

    if (d->cell_count <= 0) {
        return MQTT_PAYLOAD_ERR_NO_CELLS;
    }

    put(&o,
        "{\"schema\":%d,\"device_id\":\"%s\",\"manufacturer\":\"%s\",\"model\":\"%s\","
        "\"firmware\":\"%s\",\"chemistry\":\"%s\",\"cell_count\":%d,\"capacity_ah\":",
        SCHEMA_VERSION, device_id, d->manufacturer, d->model, d->firmware, d->chemistry,
        d->cell_count);
    put_num(&o, d->capacity_ah, 1);
    put(&o, ",\"limits\":{\"cell_overvoltage_v\":");
    put_num(&o, d->cell_overvoltage_v, 3);
    put(&o, ",\"cell_undervoltage_v\":");
    put_num(&o, d->cell_undervoltage_v, 3);
    put(&o, ",\"chg_max_temp_c\":");
    put_num(&o, d->chg_max_temp_c, 1);
    put(&o, ",\"dis_max_temp_c\":");
    put_num(&o, d->dis_max_temp_c, 1);
    put(&o, ",\"temp_hysteresis_c\":");
    put_num(&o, d->temp_hysteresis_c, 1);
    put(&o, ",\"cell_failure_diff_v\":");
    put_num(&o, d->cell_failure_diff_v, 3);
    put(&o, "}}");

    return o.err ? o.err : (int)o.pos;
}

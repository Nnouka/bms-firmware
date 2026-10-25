/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Publishes the BMS measurements over MQTT (topics and JSON: docs/mqtt-schema.md).
 *
 * Runs in its own thread: waits for an IP address from the ThingSet Wi-Fi module, syncs the clock
 * with SNTP, connects to the broker and publishes
 *   bms/<id>/status  online / offline (Last Will), retained, QoS 1
 *   bms/<id>/info    static metadata and limits, retained, QoS 1 (once the cell count is known)
 *   bms/<id>/meas    the measurements every CONFIG_BMS_MQTT_PUBLISH_INTERVAL_S seconds, QoS 0
 *
 * Settings (ThingSet group "Networking", stored in flash): sMqttHost, sMqttPort, sDeviceId.
 * Health, state of health and fault events are NOT computed here: bms-bridge derives them.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/sntp.h>

#include <bms/bms.h>
#include <thingset.h>
#include <thingset/sdk.h>

#include <stdio.h>
#include <string.h>

#include "mqtt_payload.h"

LOG_MODULE_REGISTER(bms_mqtt, CONFIG_LOG_DEFAULT_LEVEL);

#define TS_ID_NET_MQTT_HOST 0x290
#define TS_ID_NET_MQTT_PORT 0x291
#define TS_ID_NET_DEVICE_ID 0x292

#define KEEPALIVE_S      30
#define CONNECT_WAIT_MS  8000
#define RETRY_DELAY_S    5
#define PAYLOAD_BUF_SIZE 1024

extern struct bms_context bms;
extern char ipv4_addr[16]; /* set by the ThingSet Wi-Fi module once DHCP has finished */

static char mqtt_host[64] = CONFIG_BMS_MQTT_DEFAULT_HOST;
static uint16_t mqtt_port = CONFIG_BMS_MQTT_DEFAULT_PORT;
static char device_id[32] = CONFIG_BMS_MQTT_DEFAULT_DEVICE_ID;

THINGSET_ADD_ITEM_STRING(TS_ID_NET, TS_ID_NET_MQTT_HOST, "sMqttHost", mqtt_host, sizeof(mqtt_host),
                         THINGSET_ANY_RW, TS_SUBSET_NVM);
THINGSET_ADD_ITEM_UINT16(TS_ID_NET, TS_ID_NET_MQTT_PORT, "sMqttPort", &mqtt_port, THINGSET_ANY_RW,
                         TS_SUBSET_NVM);
THINGSET_ADD_ITEM_STRING(TS_ID_NET, TS_ID_NET_DEVICE_ID, "sDeviceId", device_id, sizeof(device_id),
                         THINGSET_ANY_RW, TS_SUBSET_NVM);

static struct mqtt_client client;
static struct sockaddr_storage broker;
static uint8_t rx_buf[1024];
static uint8_t tx_buf[1024];
static char payload[PAYLOAD_BUF_SIZE];

static char topic_status[64];
static char topic_info[64];
static char topic_meas[64];
static char client_id[48];
static struct mqtt_topic will_topic;
static struct mqtt_utf8 will_message;

static volatile bool connected;
static uint16_t next_message_id;

/* Unix time is not available at boot: epoch_offset_s + uptime once SNTP has answered. */
static int64_t epoch_offset_s;

static int64_t epoch_now(void)
{
    return epoch_offset_s + k_uptime_get() / 1000;
}

static int sync_time(void)
{
    struct sntp_time t;
    int rc = sntp_simple(CONFIG_BMS_MQTT_NTP_SERVER, 5000, &t);

    if (rc != 0) {
        return rc;
    }
    epoch_offset_s = (int64_t)t.seconds - k_uptime_get() / 1000;
    return 0;
}

static void evt_handler(struct mqtt_client *c, const struct mqtt_evt *evt)
{
    switch (evt->type) {
        case MQTT_EVT_CONNACK:
            if (evt->result == 0) {
                connected = true;
            } else {
                LOG_ERR("Broker refused the connection: %d", evt->result);
            }
            break;
        case MQTT_EVT_DISCONNECT:
            connected = false;
            break;
        case MQTT_EVT_PINGRESP:
        case MQTT_EVT_PUBACK:
        default:
            break;
    }
}

static int resolve_broker(void)
{
    struct zsock_addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct zsock_addrinfo *res = NULL;
    char port[8];
    int rc;

    snprintf(port, sizeof(port), "%u", mqtt_port);
    rc = zsock_getaddrinfo(mqtt_host, port, &hints, &res);
    if (rc != 0 || res == NULL) {
        LOG_ERR("Cannot resolve broker '%s' (%d)", mqtt_host, rc);
        return -EIO;
    }
    memcpy(&broker, res->ai_addr, res->ai_addrlen);
    zsock_freeaddrinfo(res);
    return 0;
}

static int broker_connect(void)
{
    struct zsock_pollfd fds[1];
    int rc = resolve_broker();

    if (rc != 0) {
        return rc;
    }

    snprintf(topic_status, sizeof(topic_status), "bms/%s/status", device_id);
    snprintf(topic_info, sizeof(topic_info), "bms/%s/info", device_id);
    snprintf(topic_meas, sizeof(topic_meas), "bms/%s/meas", device_id);
    snprintf(client_id, sizeof(client_id), "esp32-%s", device_id);

    will_topic.topic.utf8 = (uint8_t *)topic_status;
    will_topic.topic.size = strlen(topic_status);
    will_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    will_message.utf8 = (uint8_t *)"offline";
    will_message.size = strlen("offline");

    mqtt_client_init(&client);
    client.broker = &broker;
    client.evt_cb = evt_handler;
    client.client_id.utf8 = (uint8_t *)client_id;
    client.client_id.size = strlen(client_id);
    client.protocol_version = MQTT_VERSION_3_1_1;
    client.rx_buf = rx_buf;
    client.rx_buf_size = sizeof(rx_buf);
    client.tx_buf = tx_buf;
    client.tx_buf_size = sizeof(tx_buf);
    client.transport.type = MQTT_TRANSPORT_NON_SECURE;
    client.keepalive = KEEPALIVE_S;
    client.clean_session = 1;
    client.will_topic = &will_topic;
    client.will_message = &will_message;
    client.will_retain = 1;

    connected = false;
    rc = mqtt_connect(&client);
    if (rc != 0) {
        LOG_ERR("mqtt_connect failed: %d", rc);
        return rc;
    }

    fds[0].fd = client.transport.tcp.sock;
    fds[0].events = ZSOCK_POLLIN;
    rc = zsock_poll(fds, 1, CONNECT_WAIT_MS);
    if (rc > 0) {
        mqtt_input(&client);
    }
    if (!connected) {
        LOG_ERR("No CONNACK from the broker");
        mqtt_abort(&client);
        return -ETIMEDOUT;
    }
    return 0;
}

static int publish(const char *topic, const char *data, size_t len, enum mqtt_qos qos, bool retain)
{
    struct mqtt_publish_param param = {
        .message.topic.qos = qos,
        .message.topic.topic.utf8 = (const uint8_t *)topic,
        .message.topic.topic.size = strlen(topic),
        .message.payload.data = (uint8_t *)data,
        .message.payload.len = len,
        .message_id = ++next_message_id ? next_message_id : ++next_message_id,
        .dup_flag = 0,
        .retain_flag = retain,
    };

    return mqtt_publish(&client, &param);
}

static void fill_ts(char *ts, size_t len)
{
    mqtt_payload_iso8601(epoch_now(), ts, len);
}

static int publish_meas(void)
{
    struct mqtt_meas_sample s = {
        .state = bms.state,
        .pack_voltage_v = bms.ic_data.total_voltage,
        .soc_pct = bms.soc,
        .cell_voltages_v = bms.ic_data.cell_voltages,
        .num_cell_channels = ARRAY_SIZE(bms.ic_data.cell_voltages),
        .cell_temps_c = bms.ic_data.cell_temps,
        .num_temps = ARRAY_SIZE(bms.ic_data.cell_temps),
        .balancing_mask = bms.ic_data.balancing_status,
        .error_flags = bms.ic_data.error_flags,
    };
    char ts[24];
    int n;

#ifdef CONFIG_BMS_IC_CURRENT_MONITORING
    s.pack_current_a = bms.ic_data.current;
#endif
#ifdef CONFIG_BMS_IC_SWITCHES
    s.mosfet_temp_c = bms.ic_data.mosfet_temp;
#endif

    fill_ts(ts, sizeof(ts));
    n = mqtt_payload_meas(payload, sizeof(payload), device_id, ts, &s);
    if (n == MQTT_PAYLOAD_ERR_NO_CELLS) {
        return 0; /* valid, but nothing to say yet (the emulated chip has no cells) */
    }
    if (n < 0) {
        LOG_WRN("Skipping sample: payload error %d", n);
        return 0;
    }
    return publish(topic_meas, payload, n, MQTT_QOS_0_AT_MOST_ONCE, false);
}

static int publish_info(void)
{
    const struct bms_ic_conf *conf = &bms.ic_conf;
    struct mqtt_info_data d = {
        .manufacturer = "Libre Solar",
        .model = DT_PROP(DT_PATH(pcb), type),
        .firmware = FIRMWARE_VERSION_ID,
#if defined(CONFIG_CELL_TYPE_NMC)
        .chemistry = "NMC",
#elif defined(CONFIG_CELL_TYPE_LTO)
        .chemistry = "LTO",
#else
        .chemistry = "LFP",
#endif
        .cell_count = bms.ic_data.connected_cells,
        .capacity_ah = bms.nominal_capacity_Ah,
        .cell_overvoltage_v = conf->cell_ov_limit,
        .cell_undervoltage_v = conf->cell_uv_limit,
        .chg_max_temp_c = conf->chg_ot_limit,
        .dis_max_temp_c = conf->dis_ot_limit,
        .temp_hysteresis_c = conf->temp_limit_hyst,
        .cell_failure_diff_v = 0.4f, /* the firmware does not expose it: same as the relay */
    };
    int n = mqtt_payload_info(payload, sizeof(payload), device_id, &d);

    if (n < 0) {
        return n;
    }
    return publish(topic_info, payload, n, MQTT_QOS_1_AT_LEAST_ONCE, true);
}

static void mqtt_thread(void *a, void *b, void *c)
{
    int64_t next_pub_ms;
    bool info_sent;
    bool warned_no_host = false;

    while (true) {
        if (ipv4_addr[0] == '\0') {
            k_sleep(K_SECONDS(2));
            continue;
        }
        if (mqtt_host[0] == '\0') {
            if (!warned_no_host) {
                LOG_WRN("No broker configured: set Networking/sMqttHost");
                warned_no_host = true;
            }
            k_sleep(K_SECONDS(RETRY_DELAY_S));
            continue;
        }
        warned_no_host = false;

        if (sync_time() != 0) {
            LOG_WRN("SNTP failed (%s), retrying", CONFIG_BMS_MQTT_NTP_SERVER);
            k_sleep(K_SECONDS(RETRY_DELAY_S));
            continue;
        }

        if (broker_connect() != 0) {
            k_sleep(K_SECONDS(RETRY_DELAY_S));
            continue;
        }
        LOG_INF("MQTT connected to %s:%u as bms/%s", mqtt_host, mqtt_port, device_id);

        publish(topic_status, "online", strlen("online"), MQTT_QOS_1_AT_LEAST_ONCE, true);
        info_sent = false;
        next_pub_ms = k_uptime_get();

        while (connected) {
            struct zsock_pollfd fds[1];
            int64_t now = k_uptime_get();
            int32_t wait_ms;
            int rc;

            if (!info_sent && bms.ic_data.connected_cells > 0 && publish_info() == 0) {
                info_sent = true;
                LOG_INF("Published info (%d cells)", bms.ic_data.connected_cells);
            }
            if (now >= next_pub_ms) {
                if (publish_meas() != 0) {
                    break;
                }
                next_pub_ms = now + CONFIG_BMS_MQTT_PUBLISH_INTERVAL_S * 1000;
            }

            wait_ms = MIN(next_pub_ms - k_uptime_get(), mqtt_keepalive_time_left(&client));
            fds[0].fd = client.transport.tcp.sock;
            fds[0].events = ZSOCK_POLLIN;
            rc = zsock_poll(fds, 1, MAX(wait_ms, 0));
            if (rc > 0) {
                if (fds[0].revents & ZSOCK_POLLIN) {
                    rc = mqtt_input(&client);
                    if (rc != 0 && rc != -EAGAIN) {
                        break;
                    }
                }
                if (fds[0].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP | ZSOCK_POLLNVAL)) {
                    break;
                }
            }
            rc = mqtt_live(&client);
            if (rc != 0 && rc != -EAGAIN) {
                break;
            }
        }

        LOG_WRN("MQTT connection lost, reconnecting");
        mqtt_abort(&client);
        connected = false;
        k_sleep(K_SECONDS(RETRY_DELAY_S));
    }
}

K_THREAD_DEFINE(bms_mqtt_tid, 4096, mqtt_thread, NULL, NULL, NULL, 7, 0, 0);

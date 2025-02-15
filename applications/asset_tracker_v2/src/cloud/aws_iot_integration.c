/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "cloud/cloud_wrapper.h"
#include <zephyr/kernel.h>
#include <net/aws_iot.h>
#include <hw_id.h>

#define MODULE aws_iot_integration

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_CLOUD_INTEGRATION_LOG_LEVEL);

#if !defined(CONFIG_CLOUD_CLIENT_ID_USE_CUSTOM)
#define AWS_CLOUD_CLIENT_ID_LEN (HW_ID_LEN - 1)
#else
#define AWS_CLOUD_CLIENT_ID_LEN (sizeof(CONFIG_CLOUD_CLIENT_ID) - 1)
#endif

#define AWS "$aws/things/"
#define CFG_TOPIC AWS "/shadow/get/accepted/desired/cfg"
#define CFG_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(CFG_TOPIC) - 1)
#define BATCH_TOPIC "/batch"
#define BATCH_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(BATCH_TOPIC) - 1)
#define MESSAGES_TOPIC "/messages"
#define MESSAGES_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(MESSAGES_TOPIC) - 1)
#define GROUND_FIX_TOPIC "/ground-fix"
#define GROUND_FIX_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(GROUND_FIX_TOPIC) - 1)
#define AGNSS_REQUEST_TOPIC "/agnss/get"
#define AGNSS_REQUEST_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(AGNSS_REQUEST_TOPIC) - 1)
#define AGNSS_RESPONSE_TOPIC "/agnss"
#define AGNSS_RESPONSE_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(AGNSS_RESPONSE_TOPIC) - 1)
#define PGPS_REQUEST_TOPIC "/pgps/get"
#define PGPS_REQUEST_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(PGPS_REQUEST_TOPIC) - 1)
#define PGPS_RESPONSE_TOPIC "/pgps"
#define PGPS_RESPONSE_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(PGPS_RESPONSE_TOPIC) - 1)
#define MEMFAULT_TOPIC "/memfault"							\
		IF_ENABLED(CONFIG_DEBUG_MODULE_MEMFAULT_USE_EXTERNAL_TRANSPORT,		\
			   ("/" CONFIG_MEMFAULT_NCS_PROJECT_KEY))
#if defined(CONFIG_DEBUG_MODULE_MEMFAULT_USE_EXTERNAL_TRANSPORT)
#define MEMFAULT_TOPIC_LEN \
	(AWS_CLOUD_CLIENT_ID_LEN \
	+ sizeof(MEMFAULT_TOPIC) - 1 \
	+ sizeof(CONFIG_MEMFAULT_NCS_PROJECT_KEY))
#else
#define MEMFAULT_TOPIC_LEN (AWS_CLOUD_CLIENT_ID_LEN + sizeof(MEMFAULT_TOPIC) - 1)
#endif /* if defined(CONFIG_DEBUG_MODULE_MEMFAULT_USE_EXTERNAL_TRANSPORT) */

#define APP_SUB_TOPIC_IDX_CFG			0
#define APP_SUB_TOPIC_IDX_AGNSS			1
#define APP_SUB_TOPIC_IDX_PGPS			2

#define APP_PUB_TOPIC_IDX_BATCH			0
#define APP_PUB_TOPIC_IDX_UI			1
#define APP_PUB_TOPIC_IDX_NEIGHBOR_CELLS	2
#define APP_PUB_TOPIC_IDX_AGNSS			3
#define APP_PUB_TOPIC_IDX_PGPS			4
#define APP_PUB_TOPIC_IDX_MEMFAULT		5

#define APP_SUB_TOPICS_COUNT			3
#define APP_PUB_TOPICS_COUNT			6

#define REQUEST_SHADOW_DOCUMENT_STRING ""

static char client_id_buf[AWS_CLOUD_CLIENT_ID_LEN + 1];
static char batch_topic[BATCH_TOPIC_LEN + 1];
static char cfg_topic[CFG_TOPIC_LEN + 1];
static char messages_topic[MESSAGES_TOPIC_LEN + 1];
static char ground_fix_topic[GROUND_FIX_TOPIC_LEN + 1];
static char agnss_request_topic[AGNSS_REQUEST_TOPIC_LEN + 1];
static char agnss_response_topic[AGNSS_RESPONSE_TOPIC_LEN + 1];
static char pgps_request_topic[PGPS_REQUEST_TOPIC_LEN + 1];
static char pgps_response_topic[PGPS_RESPONSE_TOPIC_LEN + 1];
static char memfault_topic[MEMFAULT_TOPIC_LEN + 1];

static struct mqtt_topic sub_topics[APP_SUB_TOPICS_COUNT];
static struct mqtt_topic pub_topics[APP_PUB_TOPICS_COUNT];

static cloud_wrap_evt_handler_t wrapper_evt_handler;

static void cloud_wrapper_notify_event(const struct cloud_wrap_event *evt)
{
	if ((wrapper_evt_handler != NULL) && (evt != NULL)) {
		wrapper_evt_handler(evt);
	} else {
		LOG_ERR("Library event handler not registered, or empty event");
	}
}

static int populate_app_endpoint_topics(void)
{
	int err;

	err = snprintf(batch_topic, sizeof(batch_topic), "%s" BATCH_TOPIC,
		       client_id_buf);
	if (err != BATCH_TOPIC_LEN) {
		return -ENOMEM;
	}

	pub_topics[APP_PUB_TOPIC_IDX_BATCH].topic.utf8 = batch_topic;
	pub_topics[APP_PUB_TOPIC_IDX_BATCH].topic.size = BATCH_TOPIC_LEN;

	err = snprintf(messages_topic, sizeof(messages_topic), "%s" MESSAGES_TOPIC,
		       client_id_buf);
	if (err != MESSAGES_TOPIC_LEN) {
		return -ENOMEM;
	}

	pub_topics[APP_PUB_TOPIC_IDX_UI].topic.utf8 = messages_topic;
	pub_topics[APP_PUB_TOPIC_IDX_UI].topic.size = MESSAGES_TOPIC_LEN;

	err = snprintf(ground_fix_topic, sizeof(ground_fix_topic),
		       "%s" GROUND_FIX_TOPIC, client_id_buf);
	if (err != GROUND_FIX_TOPIC_LEN) {
		return -ENOMEM;
	}

	pub_topics[APP_PUB_TOPIC_IDX_NEIGHBOR_CELLS].topic.utf8 = ground_fix_topic;
	pub_topics[APP_PUB_TOPIC_IDX_NEIGHBOR_CELLS].topic.size = GROUND_FIX_TOPIC_LEN;

	err = snprintf(agnss_request_topic, sizeof(agnss_request_topic),
		       "%s" AGNSS_REQUEST_TOPIC, client_id_buf);
	if (err != AGNSS_REQUEST_TOPIC_LEN) {
		return -ENOMEM;
	}

	pub_topics[APP_PUB_TOPIC_IDX_AGNSS].topic.utf8 = agnss_request_topic;
	pub_topics[APP_PUB_TOPIC_IDX_AGNSS].topic.size = AGNSS_REQUEST_TOPIC_LEN;

	err = snprintf(pgps_request_topic, sizeof(pgps_request_topic),
		       "%s" PGPS_REQUEST_TOPIC, client_id_buf);
	if (err != PGPS_REQUEST_TOPIC_LEN) {
		return -ENOMEM;
	}

	pub_topics[APP_PUB_TOPIC_IDX_PGPS].topic.utf8 = pgps_request_topic;
	pub_topics[APP_PUB_TOPIC_IDX_PGPS].topic.size = PGPS_REQUEST_TOPIC_LEN;

	err = snprintf(memfault_topic, sizeof(memfault_topic), "%s" MEMFAULT_TOPIC, client_id_buf);
	if (err != MEMFAULT_TOPIC_LEN) {
		return -ENOMEM;
	}

	pub_topics[APP_PUB_TOPIC_IDX_MEMFAULT].topic.utf8 = memfault_topic;
	pub_topics[APP_PUB_TOPIC_IDX_MEMFAULT].topic.size = MEMFAULT_TOPIC_LEN;

	err = snprintf(cfg_topic, sizeof(cfg_topic), "%s" CFG_TOPIC, client_id_buf);
	if (err != CFG_TOPIC_LEN) {
		return -ENOMEM;
	}

	sub_topics[APP_SUB_TOPIC_IDX_CFG].topic.utf8 = cfg_topic;
	sub_topics[APP_SUB_TOPIC_IDX_CFG].topic.size = CFG_TOPIC_LEN;

	err = snprintf(agnss_response_topic, sizeof(agnss_response_topic),
		       "%s" AGNSS_RESPONSE_TOPIC, client_id_buf);
	if (err != AGNSS_RESPONSE_TOPIC_LEN) {
		return -ENOMEM;
	}

	sub_topics[APP_SUB_TOPIC_IDX_AGNSS].topic.utf8 = agnss_response_topic;
	sub_topics[APP_SUB_TOPIC_IDX_AGNSS].topic.size = AGNSS_RESPONSE_TOPIC_LEN;

	err = snprintf(pgps_response_topic, sizeof(pgps_response_topic), "%s" PGPS_RESPONSE_TOPIC,
		       client_id_buf);
	if (err != PGPS_RESPONSE_TOPIC_LEN) {
		return -ENOMEM;
	}

	sub_topics[APP_SUB_TOPIC_IDX_PGPS].topic.utf8 = pgps_response_topic;
	sub_topics[APP_SUB_TOPIC_IDX_PGPS].topic.size = PGPS_RESPONSE_TOPIC_LEN;

	err = aws_iot_application_topics_set(sub_topics, ARRAY_SIZE(sub_topics));
	if (err) {
		LOG_ERR("cloud_ep_subscriptions_add, error: %d", err);
		return err;
	}

	return 0;
}

/**
 * @brief Function that handles incoming data from the AWS IoT library. The function notifies the
 *	  cloud module with the appropriate event based on the incoming topic.
 *
 * @param[in] event  Pointer to AWS IoT data event.
 */
static void incoming_message_handle(struct aws_iot_evt *event)
{
	struct cloud_wrap_event cloud_wrap_evt = {
		.data.buf = event->data.msg.ptr,
		.data.len = event->data.msg.len,
		.type = CLOUD_WRAP_EVT_DATA_RECEIVED
	};

	/* Check if incoming topic is equal the subscribed A-GNSS or P-GPS response topics. */
	if (strncmp(event->data.msg.topic.str, agnss_response_topic,
		    event->data.msg.topic.len) == 0) {
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_AGNSS_DATA_RECEIVED;
	} else if (strncmp(event->data.msg.topic.str, pgps_response_topic,
		    event->data.msg.topic.len) == 0) {
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_PGPS_DATA_RECEIVED;
	}

	cloud_wrapper_notify_event(&cloud_wrap_evt);
}

void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
	struct cloud_wrap_event cloud_wrap_evt = { 0 };
	bool notify = false;

	switch (evt->type) {
	case AWS_IOT_EVT_CONNECTING:
		LOG_DBG("AWS_IOT_EVT_CONNECTING");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_CONNECTING;
		notify = true;
		break;
	case AWS_IOT_EVT_CONNECTED:
		LOG_DBG("AWS_IOT_EVT_CONNECTED");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_CONNECTED;
		notify = true;
		break;
	case AWS_IOT_EVT_DISCONNECTED:
		LOG_DBG("AWS_IOT_EVT_DISCONNECTED");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_DISCONNECTED;
		notify = true;
		break;
	case AWS_IOT_EVT_DATA_RECEIVED:
		LOG_DBG("AWS_IOT_EVT_DATA_RECEIVED");
		incoming_message_handle((struct aws_iot_evt *)evt);
		break;
	case AWS_IOT_EVT_PUBACK:
		LOG_DBG("AWS_IOT_EVT_PUBACK");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_DATA_ACK;
		cloud_wrap_evt.message_id = evt->data.message_id;
		notify = true;
		break;
	case AWS_IOT_EVT_PINGRESP:
		LOG_DBG("AWS_IOT_EVT_PINGRESP");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_PING_ACK;
		notify = true;
		break;
	case AWS_IOT_EVT_FOTA_START:
		LOG_DBG("AWS_IOT_EVT_FOTA_START");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_FOTA_START;
		notify = true;
		break;
	case AWS_IOT_EVT_FOTA_ERASE_PENDING:
		LOG_DBG("AWS_IOT_EVT_FOTA_ERASE_PENDING");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_FOTA_ERASE_PENDING;
		notify = true;
		break;
	case AWS_IOT_EVT_FOTA_ERASE_DONE:
		LOG_DBG("AWS_IOT_EVT_FOTA_ERASE_DONE");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_FOTA_ERASE_DONE;
		notify = true;
		break;
	case AWS_IOT_EVT_FOTA_DONE:
		LOG_DBG("AWS_IOT_EVT_FOTA_DONE");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_FOTA_DONE;
		notify = true;
		break;
	case AWS_IOT_EVT_FOTA_DL_PROGRESS:
		/* Dont spam FOTA progress events. */
		break;
	case AWS_IOT_EVT_FOTA_ERROR:
		LOG_DBG("AWS_IOT_EVT_FOTA_ERROR");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_FOTA_ERROR;
		notify = true;
		break;
	case AWS_IOT_EVT_ERROR:
		LOG_DBG("AWS_IOT_EVT_ERROR");
		cloud_wrap_evt.type = CLOUD_WRAP_EVT_ERROR;
		cloud_wrap_evt.err = evt->data.err;
		notify = true;
		break;
	default:
		LOG_ERR("Unknown AWS IoT event type: %d", evt->type);
		break;
	}

	if (notify) {
		cloud_wrapper_notify_event(&cloud_wrap_evt);
	}
}

int cloud_wrap_init(cloud_wrap_evt_handler_t event_handler)
{
	int err;

#if !defined(CONFIG_CLOUD_CLIENT_ID_USE_CUSTOM)
	char hw_id_buf[HW_ID_LEN];

	err = hw_id_get(hw_id_buf, ARRAY_SIZE(hw_id_buf));

	if (err) {
		LOG_ERR("Failed to retrieve device ID");
		return err;
	}

	strncpy(client_id_buf, hw_id_buf, sizeof(client_id_buf) - 1);

	/* Explicitly null terminate client_id_buf to be sure that we carry a
	 * null terminated buffer after strncpy().
	 */
	client_id_buf[sizeof(client_id_buf) - 1] = '\0';
#else
	snprintf(client_id_buf, sizeof(client_id_buf), "%s",
		 CONFIG_CLOUD_CLIENT_ID);
#endif

	err = aws_iot_init(aws_iot_event_handler);
	if (err) {
		LOG_ERR("aws_iot_init, error: %d", err);
		return err;
	}

	/* Populate cloud specific endpoint topics */
	err = populate_app_endpoint_topics();
	if (err) {
		LOG_ERR("populate_app_endpoint_topics, error: %d", err);
		return err;
	}

	LOG_DBG("********************************************");
	LOG_DBG(" The Asset Tracker v2 has started");
	LOG_DBG(" Version:     %s",
		CONFIG_ASSET_TRACKER_V2_APP_VERSION);
	LOG_DBG(" Client ID:   %s", client_id_buf);
	LOG_DBG(" Cloud:       %s", "AWS IoT");
	LOG_DBG(" Endpoint:    %s",
		CONFIG_AWS_IOT_BROKER_HOST_NAME);
	LOG_DBG("********************************************");

	wrapper_evt_handler = event_handler;

	return 0;
}

int cloud_wrap_connect(void)
{
	int err;

	struct aws_iot_config config = {
		.client_id = client_id_buf
	};

	err = aws_iot_connect(&config);
	if (err) {
		LOG_ERR("aws_iot_connect, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_disconnect(void)
{
	int err;

	err = aws_iot_disconnect();
	if (err) {
		LOG_ERR("aws_iot_disconnect, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_state_get(bool ack, uint32_t id)
{
	int err;

	struct aws_iot_data msg = {
		.ptr = REQUEST_SHADOW_DOCUMENT_STRING,
		.len = sizeof(REQUEST_SHADOW_DOCUMENT_STRING) - 1,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = AWS_IOT_SHADOW_TOPIC_GET
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_state_send(char *buf, size_t len, bool ack, uint32_t id)
{
	int err;

	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_data_send(char *buf, size_t len, bool ack, uint32_t id,
			 const struct lwm2m_obj_path path_list[])
{
	ARG_UNUSED(path_list);

	int err;
	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_batch_send(char *buf, size_t len, bool ack, uint32_t id)
{
	int err;

	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		/* <imei>/batch */
		.topic.str = pub_topics[APP_PUB_TOPIC_IDX_BATCH].topic.utf8,
		.topic.len = pub_topics[APP_PUB_TOPIC_IDX_BATCH].topic.size
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_ui_send(char *buf, size_t len, bool ack, uint32_t id,
		       const struct lwm2m_obj_path path_list[])
{
	ARG_UNUSED(path_list);

	int err;
	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		/* <imei>/messages */
		.topic.str = pub_topics[APP_PUB_TOPIC_IDX_UI].topic.utf8,
		.topic.len = pub_topics[APP_PUB_TOPIC_IDX_UI].topic.size
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_cloud_location_send(char *buf, size_t len, bool ack, uint32_t id)
{
	int err;
	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		/* <imei>/ncellmeas */
		.topic.str = pub_topics[APP_PUB_TOPIC_IDX_NEIGHBOR_CELLS].topic.utf8,
		.topic.len = pub_topics[APP_PUB_TOPIC_IDX_NEIGHBOR_CELLS].topic.size
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

bool cloud_wrap_cloud_location_response_wait(void)
{
	return false;
}

int cloud_wrap_agnss_request_send(char *buf, size_t len, bool ack, uint32_t id)
{
	int err;
	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		/* <imei>/agnss/get */
		.topic.str = pub_topics[APP_PUB_TOPIC_IDX_AGNSS].topic.utf8,
		.topic.len = pub_topics[APP_PUB_TOPIC_IDX_AGNSS].topic.size
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_pgps_request_send(char *buf, size_t len, bool ack, uint32_t id)
{
	int err;
	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		/* <imei>/pgps/get */
		.topic.str = pub_topics[APP_PUB_TOPIC_IDX_PGPS].topic.utf8,
		.topic.len = pub_topics[APP_PUB_TOPIC_IDX_PGPS].topic.size
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

int cloud_wrap_memfault_data_send(char *buf, size_t len, bool ack, uint32_t id)
{
	int err;
	struct aws_iot_data msg = {
		.ptr = buf,
		.len = len,
		.message_id = id,
		.qos = ack ? MQTT_QOS_1_AT_LEAST_ONCE : MQTT_QOS_0_AT_MOST_ONCE,
		/* <imei>/memfault */
		.topic.str = pub_topics[APP_PUB_TOPIC_IDX_MEMFAULT].topic.utf8,
		.topic.len = pub_topics[APP_PUB_TOPIC_IDX_MEMFAULT].topic.size
	};

	err = aws_iot_send(&msg);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
		return err;
	}

	return 0;
}

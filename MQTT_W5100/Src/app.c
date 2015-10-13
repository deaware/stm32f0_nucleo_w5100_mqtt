/**
  ******************************************************************************
  * File Name          : app.c
  * Description        : Foreground application function.
	* Author             : Burin Sapsiri <burin@deaware.com>
  ******************************************************************************
	*/

#include "app.h"
#include "app_sensor.h"
#include <string.h>
#include "socket.h"
#include "debug.h"
#include "mqtt-msg.h"

static const uint8_t target_host[4] = {192, 168, 10, 20};
static const uint16_t target_port = 1883;
static uint8_t buf[100];

#define MQTT_BUF_LEN				100
#define MQTT_TOPIC_MAX      32
#define MQTT_DATA_MAX       32
static uint8_t mqtt_buf[MQTT_BUF_LEN];
static uint8_t mqtt_is_connected = 0;
static mqtt_connection_t mqtt_conn;
static mqtt_connect_info_t connect_info;

#define MQTT_KEEP_ALIVE     30

static mqtt_message_t *connect_msg;


static void mqtt_init(void);
static void mqtt_connect(uint8_t *host, uint16_t port, char* cid);
static void mqtt_publish(char *topic, char *payload, uint16_t payload_length, int qos);
static void mqtt_subscribe(char* topic, int qos);
static void mqtt_ping(void);
static void mqtt_connected(void);
static void mqtt_disconnected(void);
static void mqtt_published(void);
static void mqtt_subscribed(void);
static void mqtt_received(char *topic, uint16_t topic_len, char *data, uint16_t data_len);

static void mqtt_init(void)
{
	mqtt_msg_init(&mqtt_conn, mqtt_buf, MQTT_BUF_LEN);
}

static void mqtt_connect(uint8_t *host, uint16_t port, char* cid)
{
	connect_info.client_id = cid;
	connect_info.username = 0;
	connect_info.password = 0;
	connect_info.will_topic = 0;
	connect_info.will_message = 0;
	connect_info.keepalive = MQTT_KEEP_ALIVE;
  connect_info.will_qos = 0;
  connect_info.will_retain = 0;
  connect_info.clean_session = 0;
	
	connect_msg = mqtt_msg_connect(&mqtt_conn, &connect_info);
	
	connect(APP_SOCKET_NO, host, port);
}

static void mqtt_publish(char *topic, char *payload, uint16_t payload_length, int qos)
{
	mqtt_message_t *msg_ptr;
	int8_t rslt = 0;
	
	if (qos < 0) {
		qos = 0;
	}
	
	/* Not implement for qos 1, qos 2 yet. */
	if (qos > 0) {
		qos = 0;
	}
	
	msg_ptr = mqtt_msg_publish(&mqtt_conn, topic, payload, payload_length, qos, 0, &mqtt_conn.message_id);
	
	rslt = send(APP_SOCKET_NO, msg_ptr->data, msg_ptr->length);
	
	DEBUG_PRINT("send : %d", rslt)
	
	if (qos == 0) {
		mqtt_published();
	}
}

static void mqtt_subscribe(char* topic, int qos)
{
	mqtt_message_t *msg_ptr;

	if (qos < 0) {
		qos = 0;
	}
	
	/* Not implement for qos 1, qos 2 yet. */
	if (qos > 0) {
		qos = 0;
	}
	
	msg_ptr = mqtt_msg_subscribe(&mqtt_conn, topic, qos, &mqtt_conn.message_id);
	
	send(APP_SOCKET_NO, msg_ptr->data, msg_ptr->length);
	
	if (qos == 0) {
		mqtt_subscribed();
	}
}

static void mqtt_ping(void)
{
	mqtt_message_t *msg_ptr;
	msg_ptr = mqtt_msg_pingreq(&mqtt_conn);
	
	send(APP_SOCKET_NO, msg_ptr->data, msg_ptr->length);
}

static void mqtt_connected(void)
{
	DEBUG_PRINT("MQTT connectted.")

	mqtt_subscribe("iot/deaware/sn0", 0);
	mqtt_is_connected = 1;
}

static void mqtt_disconnected(void)
{
	DEBUG_PRINT("MQTT disconnectted.")
	
	mqtt_is_connected = 0;
}

static void mqtt_published(void)
{
	DEBUG_PRINT("MQTT published.")
}

static void mqtt_subscribed(void)
{
	DEBUG_PRINT("MQTT subscribed.")
}

static void mqtt_received(char *topic, uint16_t topic_len, char *data, uint16_t data_len)
{
	strncpy((char*)buf, topic, topic_len);
	buf[topic_len] = '\0';
	
	strncpy((char*)buf + topic_len + 1, data, data_len);
	buf[topic_len + 1 + data_len] = '\0';
	
	DEBUG_PRINT("MQTT received from %s : %s", buf, (buf + topic_len + 1))
}

void app_connected(void)
{
	DEBUG_PRINT("Connected")
	
	send(APP_SOCKET_NO, connect_msg->data, connect_msg->length);
}

void app_disconnected(void)
{
	DEBUG_PRINT("Disconnected")
	
	mqtt_disconnected();
}

void app_received(uint8_t *data, uint16_t len)
{
	int msg_type = 0;
	char *topic_ptr;
	uint16_t topic_len = MQTT_TOPIC_MAX;
	char *data_ptr;
	uint16_t data_len = MQTT_DATA_MAX;
	
	DEBUG_PRINT("Received : %d", len)
	
	msg_type = mqtt_get_type(data);
	
	switch (msg_type) {
		case MQTT_MSG_TYPE_CONNECT:
			break;
		case MQTT_MSG_TYPE_CONNACK:
			if (data[2] == 0) {
				mqtt_connected();
			}
			break;
		case MQTT_MSG_TYPE_PUBLISH:
			topic_ptr = (char*)mqtt_get_publish_topic(data, &topic_len);
			data_ptr = (char*)mqtt_get_publish_data(data, &data_len);
			mqtt_received(topic_ptr, topic_len, data_ptr, data_len);
			break;
		case MQTT_MSG_TYPE_PUBACK:
			break;
		case MQTT_MSG_TYPE_PUBREC:
			break;
		case MQTT_MSG_TYPE_PUBREL:
			break;
		case MQTT_MSG_TYPE_PUBCOMP:
			break;
		case MQTT_MSG_TYPE_SUBSCRIBE:
			break;
		case MQTT_MSG_TYPE_SUBACK:
			if (data[4] != 0x80) {
				mqtt_subscribed();
			}
			break;
		case MQTT_MSG_TYPE_UNSUBSCRIBE:
			break;
		case MQTT_MSG_TYPE_UNSUBACK:
			break;
		case MQTT_MSG_TYPE_PINGREQ:
			break;
		case MQTT_MSG_TYPE_PINGRESP:
			break;
		case MQTT_MSG_TYPE_DISCONNECT:
			break;
		default:
			break;
	}
}

void app_sent(void)
{
	DEBUG_PRINT("Sent")
}

void app_init(void)
{
	uint8_t rtnval = 0;
	
	app_sensor_init();
	
	rtnval = socket(APP_SOCKET_NO, Sn_MR_TCP, 0, 0);
	if (rtnval != APP_SOCKET_NO) {
		ERR_PRINT("Cannot bind socket no. %d", APP_SOCKET_NO)
		while (1);
	}
	
	mqtt_init();
	mqtt_connect((uint8_t*)target_host, target_port, "MyCliesdfdfnt");
}

void app_tick(void)
{
	uint16_t len = 0;
	static uint32_t prev;
	static uint32_t prev2;
	uint32_t now;
	
	now = HAL_GetTick();
	
	if (now - prev > (MQTT_KEEP_ALIVE * 1000)) {
		prev = now;
		mqtt_ping();
	}
	
	
	if (now - prev2 > 1000) {
		prev2 = now;
		if (mqtt_is_connected) {
			len = sprintf((char*)buf, "%.2f", app_sensor_read_tmp());
			mqtt_publish("iot/deaware/tmp", (char*)buf, len, 0);
			
			len = sprintf((char*)buf, "%.2f", app_sensor_read_hum());
			mqtt_publish("iot/deaware/hum", (char*)buf, len, 0);
			
			len = sprintf((char*)buf, "%.3f", app_sensor_read_psr() / 10);
			mqtt_publish("iot/deaware/psr", (char*)buf, len, 0);
		}
	}
}

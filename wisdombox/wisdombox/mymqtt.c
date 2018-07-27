/* ------------------------ LWIP includes --------------------------------- */
#include "mymqtt.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/memp.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip.h"
#include "netif/ethernet.h"

#include <memory.h>
 void mqtt_do_connect(mqtt_client_t * client) {
	ip4_addr_t broker_ipaddr;
	struct mqtt_connect_client_info_t ci;
	err_t err;
	IP4_ADDR(& broker_ipaddr, 176, 122, 166, 83);
	/* Setup an empty client info structure */
	memset(& ci, 0, sizeof(ci));
	/* Minimal amount of information required is client identifier, so set it here */
	ci.client_id = "mymqtt";
	ci.client_user = "gaohongwei";
	ci.client_pass = "";
	/* Initiate client and connect to server, if this fails immediately an error code is returned
	  otherwise mqtt_connection_cb will be called with connection result after attempting
	  to establish a connection with the server.
	  For now MQTT version 3.1.1 is always used */
	err = mqtt_client_connect(client, & broker_ipaddr, MQTT_PORT, mqtt_connection_cb, 0, & ci);
	/* For now just print the result code if something goes wrong */
	if (err != ERR_OK) {
		printf("mqtt_connect return %d\n", err);
	}
}

 void mqtt_connection_cb(mqtt_client_t * client, void * arg, mqtt_connection_status_t status) {
	err_t err;
	if (status == MQTT_CONNECT_ACCEPTED) {
		printf("mqtt_connection_cb: Successfully connected\n");
		/* Setup callback for incoming publish requests */
		//ע�� ��Ϣ���� �ص����� �Լ���Ϣ���ݵ����Ĵ�����
		mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);
		/* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
		err = mqtt_subscribe(client, "subtopic", 1, mqtt_sub_request_cb, arg);
		if (err != ERR_OK) {
			printf("mqtt_subscribe return: %d\n", err);
		}
	}
	else {
		printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);
		/* Its more nice to be connected, so try to reconnect */
		mqtt_do_connect(client);
	}
}

/* The idea is to demultiplex topic and create some reference to be used in data callbacks
 Example here uses a global variable, better would be to use a member in arg
 If RAM and CPU budget allows it, the easiest implementation might be to just take a copy of
 the topic string and use it in mqtt_incoming_data_cb
*/
//���������� һ�� ���� ��Ϣͷ һ�� ���� ����� ����
//��Ϣ����֮���Ƚ������������ж� ��ʲô���� 
static int inpub_id;
 void mqtt_incoming_publish_cb(void * arg,
	const char * topic,
	u32_t tot_len) {
	printf("Incoming publish at topic %s with total length %u\n", topic, (unsigned int) tot_len);
	/* Decode topic string into a user defined reference */
	if (strcmp(topic, "print_payload") == 0) {
		inpub_id = 0;
	}
	else if (topic[0] == 'A') {
		/* All topics starting with 'A' might be handled at the same way */
		inpub_id = 1;
	}
	else {
		/* For all other topics */
		inpub_id = 2;
	}
}
//��Ϣ����֮�����ж����� �����Ǹ�������Ȼ���������Ǹ��ݲ�ͬ�����⣬���в�ͬ�Ĵ���
 void mqtt_incoming_data_cb(void * arg,
	const u8_t * data,
	u16_t len,
	u8_t flags) {
	printf("Incoming publish payload with length %d, flags %u\n", len, (unsigned int) flags);
	if (flags & MQTT_DATA_FLAG_LAST) {
		/* Last fragment of payload received (or whole part if payload fits receive buffer
		See MQTT_VAR_HEADER_BUFFER_LEN) */
		/* Call function or do action depending on reference, in this case inpub_id */
		if (inpub_id == 0) {
			/* Don't trust the publisher, check zero termination */
			if (data[len - 1] == 0) {
				printf("mqtt_incoming_data_cb: %s\n", (const char *) data);
			}
		}
		else if (inpub_id == 1) {
			/* Call an 'A' function... */
		}
		else {
			printf("mqtt_incoming_data_cb: Ignoring payload...\n");
		}
	}
	else {
		/* Handle fragmented payload, store in buffer, write to file or whatever */
	}
}
 void mqtt_sub_request_cb(void * arg, err_t result) {
	/* Just print the result code here for simplicity,
	normal behaviour would be to take some action if subscribe fails like
	notifying user, retry subscribe or disconnect from server */
	printf("Subscribe result: %d\n", result);
}

 void my_mqtt_publish(mqtt_client_t * client, void * arg) {
	const char * pub_payload = "abcd";//���͵���Ч����(����)
	err_t err;
	/*
  *    ���һ�Σ���һ����ᷢ����Ϣ��ʧ���ظ�����Ϣ���������ڵײ�TCP/IP���硣����<=1
    *    ����һ�Σ���һ�����ȷ����Ϣ�������Ϣ���ܻ��ظ�������>=1
    *    ֻ��һ�Σ�ȷ����Ϣֻ��һ�ε��������1����һЩҪ��Ƚ��ϸ�ļƷ�ϵͳ�У�����ʹ�ô˼���
  */
	u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
	u8_t retain = 0; /* No don't retain such crappy payload... */
	err = mqtt_publish(client, "test", pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
	if (err != ERR_OK) {
		printf("Publish err: %d\n", err);
	}
}

/* Called when publish is complete either with success or failure */
//������Ϣ��ɺ� �� �ص�����
 void mqtt_pub_request_cb(void * arg, err_t result) {
	if (result != ERR_OK) {
		printf("Publish result: %d\n", result);
	}
}
#ifndef __MYMQTT_H
#define __MYMQTT_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "lwip/apps/mqtt.h"



	 void my_mqtt_publish(mqtt_client_t * client, void * arg);
	 
	 void mqtt_sub_request_cb(void * arg, err_t result);
	 void mqtt_incoming_data_cb(void * arg,
		 const u8_t * data,
		 u16_t len,
		 u8_t flags);
	 void mqtt_incoming_publish_cb(void * arg,const char * topic,
		 u32_t tot_len);
	 void mqtt_connection_cb(mqtt_client_t * client, void * arg, mqtt_connection_status_t status);
	 void mqtt_do_connect(mqtt_client_t * client);
	 
 void mqtt_pub_request_cb(void * arg, err_t result);

#endif
	 
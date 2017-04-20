/**
 * \file
 *
 * \brief HTTP File Downloader Example.
 *
 * Copyright (c) 2016-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define DETKIN
//#define HOME
// #define HAL

/** Wi-Fi AP Settings. */
#ifdef HAL
#define MAIN_WLAN_SSID                       "hp-setup-1" /**< Destination SSID */
#define MAIN_WLAN_AUTH                       M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                        "WelcomeHal" /**< Password for Destination SSID */
#endif

#ifdef DETKIN
#define MAIN_WLAN_SSID                       "HP-setup-south" /**< Destination SSID */
#define MAIN_WLAN_AUTH                       M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                        "detkinlabsouth101999" /**< Password for Destination SSID */
#endif

#ifdef HOME
#define MAIN_WLAN_SSID                       "black&yellow" /**< Destination SSID */
#define MAIN_WLAN_AUTH                       M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                        "808s&heartbreak" /**< Password for Destination SSID */
#endif
/** IP address parsing. */
#define IPV4_BYTE(val, index)                ((val >> (index * 8)) & 0xFF)

/** Content URI for download. */
#define BINARY_FILE_URL                   "https://www.seas.upenn.edu/~versia/ESE680_AppCode.bin"
#define CRC_FILE_URL                      "https://www.seas.upenn.edu/~versia/ESE680_AppCode.crc"

/** Maximum size for packet buffer. */
#define MAIN_BUFFER_MAX_SIZE                 (4*1024)
/** Maximum file name length. */
#define MAIN_MAX_FILE_NAME_LENGTH            (250)
/** Maximum file extension length. */
#define MAIN_MAX_FILE_EXT_LENGTH             (8)
/** Output format with '0'. */
#define MAIN_ZERO_FMT(SZ)                    (SZ == 4) ? "%04d" : (SZ == 3) ? "%03d" : (SZ == 2) ? "%02d" : "%d"

typedef enum {
	NOT_READY = 0,
	STORAGE_READY = 0x01, /*!< Storage is ready. */
	WIFI_CONNECTED = 0x02, /*!< Wi-Fi is connected. */
	GET_REQUESTED = 0x04, /*!< GET request is sent. */
	DOWNLOADING = 0x08, /*!< Running to download. */
	COMPLETED = 0x10, /*!< Download completed. */
	CANCELED = 0x20 /*!< Download canceled. */
} download_state;

/* Max size of UART buffer. */
#define MAIN_CHAT_BUFFER_SIZE 64

/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 128

/* Limitation of user name. */
#define MAIN_CHAT_USER_NAME_SIZE 64

/* Limitation of password. */
#define MAIN_CHAT_PASSWORD_SIZE 64

/* Chat MQTT topic. */
#define MAIN_CHAT_TOPIC "ese680/main/"
#define SENSOR_TOPIC    "ese680/sensor/"
#define ACTUATOR_TOPIC  "ese680/actuator/"

#define MQTT_PORT       16625

/*
 * A MQTT broker server which was connected.
 * m2m.eclipse.org is public MQTT broker.
 */
static const char main_mqtt_broker[] = "m11.cloudmqtt.com";

#define LEFT_BUTTON_PIN		PIN_PA09
#define LED_PIN				PIN_PA17  
#define LED2_PIN			PIN_PA22  

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */

/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <errno.h>
#include <asf.h>
#include <board.h>
#include "main.h"
#include "stdio_serial.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "iot/http/http_client.h"
#define LED0_ACTIVE               false
#define LED0_INACTIVE             !LED0_ACTIVE
// MEMORY MAP
// Code goest to ~ 0x3500 budget until 0x0FFF
// BOOT Status 4100 -> 41FF
// APP 5000 -> TOP
#define BOOT_STATUS_ADDR 0x4100 //boot status address in nvm
#define BASE_CODE_ADDR 0x5000 //nvm application start address
#define TOP_CODE_ADDR 0x8000  //nvm application end address
#define FLASH_FIRMWARE_HEADER_ADDR 0x2000  //address of header for fw1 in external flash
#define BASE_FLASH_CODE_ADDR 0x3000  //start address of fw 1 in external flash
#define TOP_FLASH_CODE_ADDR 0x6000   //start address of fw2 in in external flash
#define FLASH_TOP_FIRMWARE_HEADER_ADDR  0x5000  //address of header for fw2 in external flash
#define INTEGRITY_CHECK 0xdead
#define APP_START_ADDR 0x5000 //start address of application code in nvm (pointer to jump to in the application code)
typedef struct bs {
	uint32_t integrity_check;
	uint8_t signature[3];
	int8_t executing_image;
	int8_t downlaoded_image;
	uint8_t arr[NVMCTRL_PAGE_SIZE-9];
}boot_status;

typedef struct fh {
  uint32_t crc;
  uint32_t size;
  uint32_t sw_version;
  uint32_t hw_version;
} firmware_header;

firmware_header download_header;

// FLASH CHIP CONFIG
#define FLASH_SPI_SERCOM          SERCOM1
#define FLASH_SPI_CLOCK_SPEED     12000000 ///< Hz
#define FLASH_SPI_CLK_SOURCE      GCLK_GENERATOR_0


#define SERIALFLASH_SPI_MODULE      SERCOM1
#define SERIALFLASH_SPI_MUX_SETTING SPI_SIGNAL_MUX_SETTING_E
#define SERIALFLASH_SPI_PINMUX_PAD0 PINMUX_PA16C_SERCOM1_PAD0 /// MISO
#define SERIALFLASH_SPI_PINMUX_PAD1 PINMUX_UNUSED
#define SERIALFLASH_SPI_PINMUX_PAD2 PINMUX_PA18C_SERCOM1_PAD2 /// MOSI
#define SERIALFLASH_SPI_PINMUX_PAD3 PINMUX_PA19C_SERCOM1_PAD3 /// SCK
#define SERIALFLASH_SPI_CS          PIN_PA07                  /// CS


//! Select the SPI module AT25DFx is connected to
#define AT25DFX_SPI                 SERIALFLASH_SPI_MODULE

/** AT25DFx device type */
#define AT25DFX_MEM_TYPE            AT25DFX_081A

#define AT25DFX_SPI_PINMUX_SETTING  SERIALFLASH_SPI_MUX_SETTING
#define AT25DFX_SPI_PINMUX_PAD0     SERIALFLASH_SPI_PINMUX_PAD0
#define AT25DFX_SPI_PINMUX_PAD1     SERIALFLASH_SPI_PINMUX_PAD1
#define AT25DFX_SPI_PINMUX_PAD2     SERIALFLASH_SPI_PINMUX_PAD2
#define AT25DFX_SPI_PINMUX_PAD3     SERIALFLASH_SPI_PINMUX_PAD3

#define AT25DFX_CS                  SERIALFLASH_SPI_CS

//! SPI master speed in Hz.
#define AT25DFX_CLOCK_SPEED         1200000  /// 12000000



//! [buffers]
#define AT25DFX_BUFFER_SIZE  (10)

//! [buffers]

//! [driver_instances]
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
//! [driver_instances]


// END OF FLASH CHIP CONFIG

// have we downloaded CRC
static int crc_status = 0;
// 0 - Need to download
// 1 - File does not exist
// 2 - Done downloading
// File download processing state.
static download_state down_state = NOT_READY;
// instance of uart module
struct usart_module usart_instance;
// instance of timer module
struct sw_timer_module swt_module_inst;
// Instance of HTTP client module.
struct http_client_module http_client_module_inst;
/** Http content length. */
static uint32_t http_file_size = 0;
/** Receiving content length. */
static uint32_t received_file_size = 0;

// Overflow buffer for if we get less thank 4KB
char http_overflow[MAIN_BUFFER_MAX_SIZE];
int http_overflow_length = 0;
int http_page_num = 1;

void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;
	stdio_serial_init(&usart_instance, SERCOM4, &config_usart);
	usart_enable(&usart_instance);
}

static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

/**
 * \brief Initialize download state to not ready.
 */
static void init_state(void)
{
	down_state = NOT_READY;
}

/**
 * \brief Clear state parameter at download processing state.
 * \param[in] mask Check download_state.
 */
static void clear_state(download_state mask)
{
	down_state &= ~mask;
}

/**
 * \brief Add state parameter at download processing state.
 * \param[in] mask Check download_state.
 */
static void add_state(download_state mask)
{
	down_state |= mask;
}

/**
 * \brief File download processing state check.
 * \param[in] mask Check download_state.
 * \return true if this state is set, false otherwise.
 */

static inline bool is_state_set(download_state mask)
{
	return ((down_state & mask) != 0);
}
/**
 * \brief Store received packet to file.
 * \param[in] data Packet data.
 * \param[in] length Packet data length.
 */
static void store_file_packet(char *data, uint32_t length)
{

	//FRESULT ret;
	if ((data == NULL) || (length < 1)) {
		printf("store_file_packet: empty data.\r\n");
		return;
	}

	if (!is_state_set(DOWNLOADING)) {
		received_file_size = 0;
		http_page_num = 0;
		add_state(DOWNLOADING);
	}

  if (crc_status == 0) {
    printf("Saving CRC");
    // downloading crc
    memcpy(&download_header, data, length);
    clear_state(DOWNLOADING);
    add_state(COMPLETED);
    return;
  }

	if (data != NULL) {
		if (length < MAIN_BUFFER_MAX_SIZE && http_overflow_length == 0) {
			printf("nothing in save buffer, filling it now");
			// nothing in our overflow buffer, and not enough data for full buffer
			memcpy(http_overflow, data, length);
			http_overflow_length = length;
		} else if (http_overflow_length != 0) {
				printf("using save buffer");
				// not enough data for full buffer, and there is data in our overflow buffer
				if (length + http_overflow_length >= MAIN_BUFFER_MAX_SIZE) {
					// would overflow the overflow buffer with the new data
					int copy_size = (MAIN_BUFFER_MAX_SIZE - http_overflow_length);
					memcpy(http_overflow + http_overflow_length, data, copy_size); // fill remaining space in buffer
					at25dfx_chip_write_buffer(&at25dfx_chip,  (http_page_num * MAIN_BUFFER_MAX_SIZE) + 0x10000, http_overflow, MAIN_BUFFER_MAX_SIZE);
					memset(http_overflow, 0, MAIN_BUFFER_MAX_SIZE);
					if (length + http_overflow_length > MAIN_BUFFER_MAX_SIZE){
						memcpy(http_overflow, data + copy_size, length - copy_size);
						http_overflow_length = (length - copy_size);
					} else {
						http_overflow_length = 0;
					}
					http_page_num++;
				} else {
					printf("sup");
					// need to add to the end of the http_overflow buffer, because we didn't get enough data to overflow again
					memcpy(http_overflow + http_overflow_length, data, length); // fill remaining space in buffer
					http_overflow_length += length;
				}
		} else {
			printf("full buffer");
			// we received a full buffer
			at25dfx_chip_write_buffer(&at25dfx_chip, (http_page_num * MAIN_BUFFER_MAX_SIZE) + 0x10000, data, MAIN_BUFFER_MAX_SIZE);
			http_page_num++;
		}
		unsigned int wsize = length;
		//ret = f_write(&file_object, (const void *)data, length, &wsize);
		//if (ret != FR_OK) {
			//f_close(&file_object);
			//add_state(CANCELED);
			//printf("store_file_packet: file write error, download canceled.\r\n");
			//return;
		//}

		received_file_size += wsize;
		printf("store_file_packet: received[%lu], file size[%lu]\r\n", (unsigned long)received_file_size, (unsigned long)http_file_size);
		if (received_file_size >= http_file_size) {
			//f_close(&file_object);
			printf("store_file_packet: file downloaded successfully.\r\n");
			port_pin_set_output_level(PIN_PA23, false);
			add_state(COMPLETED);
			// http_page_num = 1; // Reset page number to 1
			return;
		}
	}
}

/**
 * \brief Start file download via HTTP connection.
 */
static void start_download(void)
{
	if (!is_state_set(STORAGE_READY)) {
		printf("start_download: MMC storage not ready.\r\n");
		return;
	}

	if (!is_state_set(WIFI_CONNECTED)) {
		printf("start_download: Wi-Fi is not connected.\r\n");
		return;
	}

	if (is_state_set(GET_REQUESTED)) {
		printf("start_download: request is sent already.\r\n");
		return;
	}

	if (is_state_set(DOWNLOADING)) {
		printf("start_download: running download already.\r\n");
		return;
	}

	/* Send the HTTP request. */
  if (crc_status == 0) {
    printf("start_download: sending HTTP request...\r\n");
    http_client_send_request(&http_client_module_inst, CRC_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
  } else if (crc_status == 1) {
    printf("CRC_FILE not found on server\r\n");
    return;
  } else {
    printf("Downloading binary file\r\n");
    http_client_send_request(&http_client_module_inst, BINARY_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
  }
}

/**
 * \brief Callback of the HTTP client.
 *
 * \param[in]  module_inst     Module instance of HTTP client module.
 * \param[in]  type            Type of event.
 * \param[in]  data            Data structure of the event. \refer http_client_data
 */
static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
	switch (type) {
	case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
		printf("http_client_callback: HTTP client socket connected.\r\n");
		break;

	case HTTP_CLIENT_CALLBACK_REQUESTED:
		printf("http_client_callback: request completed.\r\n");
		add_state(GET_REQUESTED);
		break;

	case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
		printf("http_client_callback: received response %u data size %u\r\n",
				(unsigned int)data->recv_response.response_code,
				(unsigned int)data->recv_response.content_length);
		if ((unsigned int)data->recv_response.response_code == 200) {
			http_file_size = data->recv_response.content_length;
			received_file_size = 0;
		}
		else {
			add_state(CANCELED);
			crc_status = 1;
			return;
		}
		if (data->recv_response.content_length <= MAIN_BUFFER_MAX_SIZE) {
			store_file_packet(data->recv_response.content, data->recv_response.content_length);
			add_state(COMPLETED);
		}
		break;

	case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
		store_file_packet(data->recv_chunked_data.data, data->recv_chunked_data.length);
		if (data->recv_chunked_data.is_complete) {
			add_state(COMPLETED);
		}

		break;

	case HTTP_CLIENT_CALLBACK_DISCONNECTED:
		printf("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

		/* If disconnect reason is equal to -ECONNRESET(-104),
		 * It means the server has closed the connection (timeout).
		 * This is normal operation.
		 */
		if (data->disconnected.reason == -EAGAIN) {
			/* Server has not responded. Retry immediately. */
			if (is_state_set(DOWNLOADING)) {
				//TODO f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			start_download();
		}

		break;
	}
}
/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			clear_state(WIFI_CONNECTED);
			if (is_state_set(DOWNLOADING)) {
				//f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		add_state(WIFI_CONNECTED);
		start_download();
		break;
	}

	default:
		break;
	}
}

/**
 * \brief Callback to get the data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	http_client_socket_event_handler(sock, u8Msg, pvMsg);
}

/**
 * \brief Callback for the gethostbyname function (DNS Resolution callback).
 * \param[in] pu8DomainName Domain name of the host.
 * \param[in] u32ServerIP Server IPv4 address encoded in NW byte order format. If it is Zero, then the DNS resolution failed.
 */
static void resolve_cb(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", pu8DomainName,
			(int)IPV4_BYTE(u32ServerIP, 0), (int)IPV4_BYTE(u32ServerIP, 1),
			(int)IPV4_BYTE(u32ServerIP, 2), (int)IPV4_BYTE(u32ServerIP, 3));
	http_client_socket_resolve_handler(pu8DomainName, u32ServerIP);
}

static void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = MAIN_BUFFER_MAX_SIZE;
	httpc_conf.timer_inst = &swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		printf("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
		while (1) {
			} /* Loop forever. */
		}

		http_client_register_callback(&http_client_module_inst, http_client_callback);
}
//! [init_function]
static void at25dfx_init(void)
{
  //! [config_instances]
  struct at25dfx_chip_config at25dfx_chip_config;
  struct spi_config at25dfx_spi_config;
  //! [config_instances]

  //! [spi_setup]
  at25dfx_spi_get_config_defaults(&at25dfx_spi_config);
  at25dfx_spi_config.mode_specific.master.baudrate = AT25DFX_CLOCK_SPEED;
  at25dfx_spi_config.mux_setting = AT25DFX_SPI_PINMUX_SETTING;
  at25dfx_spi_config.pinmux_pad0 = AT25DFX_SPI_PINMUX_PAD0;
  at25dfx_spi_config.pinmux_pad1 = AT25DFX_SPI_PINMUX_PAD1;
  at25dfx_spi_config.pinmux_pad2 = AT25DFX_SPI_PINMUX_PAD2;
  at25dfx_spi_config.pinmux_pad3 = AT25DFX_SPI_PINMUX_PAD3;

  spi_init(&at25dfx_spi, AT25DFX_SPI, &at25dfx_spi_config);
  spi_enable(&at25dfx_spi);

  //! [spi_setup]

  //! [chip_setup]
  at25dfx_chip_config.type = AT25DFX_MEM_TYPE;
  at25dfx_chip_config.cs_pin = AT25DFX_CS;

  at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_chip_config);

  //! [chip_setup]
}
//! [init_function]


void flash_startup() {
	at25dfx_chip_wake(&at25dfx_chip);

	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		printf("Flash chip unresponsive\r\n");
	}
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
	at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);
}

void flash_shutdown() {
	 at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
	 at25dfx_chip_sleep(&at25dfx_chip);
}

void configure_nvm(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
}

void run_app() {
  printf ("Running LOCAL application\r\n");

  // infinite loop
  while (1);
}

int check_version_number() {
    // read the flash and check if the crc's match
    firmware_header flash_fh;
    at25dfx_chip_read_buffer(&at25dfx_chip, 0x0, &flash_fh, sizeof(firmware_header));
    return (download_header.crc == flash_fh.crc);
}

int main (void)
{
	tstrWifiInitParam param;
	int8_t ret;
	// init board
	system_init();
	// init UART
	configure_usart();
	//printf ("online code\r\n");
	//while(1);
	//while (1) {
	//	printf("online code\r\n");
	//}

	//usart_reset(&usart_instance);
	//system_reset();
	// init state
	init_state();
	 // init storage
	at25dfx_init();
	add_state(STORAGE_READY);
	flash_startup();
	//Set up NVM
	configure_nvm();
	// init delay
	delay_init();
	//enable interrupts
	system_interrupt_enable_global();
	// init timer
	configure_timer();
	// initialize http client service
	configure_http_client();
	// Initialize the BSP.
	nm_bsp_init();
	// Initialize Wi-Fi parameters structure.
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	//TODO DELETE ME
	at25dfx_chip_erase_block(&at25dfx_chip, 0x00000, AT25DFX_BLOCK_SIZE_64KB); // erase 64k block for firmware


	// Initialize Wi-Fi driver with data and status callbacks.
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error! (res %d)\r\n", ret);
		while (1) {
		}
	}
	// Initialize socket module.
	socketInit();
	// Register socket callback function.
	registerSocketCallback(socket_cb, resolve_cb);

	// Connect to router.
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

  // wait for crc to get downloaded
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
	}
  clear_state(COMPLETED);
  clear_state(CANCELED);
  clear_state(DOWNLOADING);
  clear_state(GET_REQUESTED);
  if (crc_status == 1) {
    // crc_status file does not exits
    // skip to run the app since it does not exist
	printf("Could not get crc from server");
    run_app();
  } else if (check_version_number()) {
    // if the versions match then we can simply run the app
    run_app();
  } 
  at25dfx_chip_erase_block(&at25dfx_chip, 0x00000, AT25DFX_BLOCK_SIZE_64KB); // erase bottom  64k block for crc header
  at25dfx_chip_write_buffer(&at25dfx_chip, 0x00000, &download_header, sizeof(download_header)); // save crc to flash
  
  // DEBUG
  firmware_header debug_header;
  at25dfx_chip_read_buffer(&at25dfx_chip,0x00000, &debug_header, sizeof(download_header));
  if (debug_header.crc == download_header.crc) {
	  printf("harder written to flash is right");
  } else{
	  printf("harder written to flash is right");
  }
  
  // END DEBUG
  crc_status = 2; // completed downloading crc, move on to http

	// cointune on to download the firmware and reset
	at25dfx_chip_erase_block(&at25dfx_chip, 0x10000, AT25DFX_BLOCK_SIZE_64KB); // erase 64k block for firmware
	start_download();
	
	// wait for firmware to get downloaded
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
	}
	
	if (!is_state_set(COMPLETED) && is_state_set(CANCELED)) {
		printf("Firmware could not be downloaded");
		run_app();
	}
	
	if (http_overflow_length != 0){
		at25dfx_chip_write_buffer(&at25dfx_chip, (http_page_num * MAIN_BUFFER_MAX_SIZE) + 0x10000, http_overflow, http_overflow_length);
	}
	printf("%d\r\n", http_page_num);
	int num_pages_to_read = http_page_num;
	http_page_num = 1;
	printf("main: please unplug the SD/MMC card.\r\n");
	printf("main: done.\r\n");
	//at25dfx_chip_read_buffer(&at25dfx_chip, http_page_num * 0x10000, http_overflow, http_overflow_length);
	//printf ("%s\r\n", &http_overflow[0]);
	flash_shutdown();

	// update boot status
	enum status_code error_code;
	boot_status status;
	do
  {
      error_code = nvm_read_buffer(
              BOOT_STATUS_ADDR,
              (void *) &status, NVMCTRL_PAGE_SIZE);
  } while (error_code == STATUS_BUSY);
	status.downlaoded_image = 1;
	do
	{
		error_code = nvm_erase_row(
		BOOT_STATUS_ADDR);
	} while (error_code == STATUS_BUSY);
	do
	{
		error_code = nvm_write_buffer(
		BOOT_STATUS_ADDR,
		(void *) &status, NVMCTRL_PAGE_SIZE);
	} while (error_code == STATUS_BUSY);

	usart_reset(&usart_instance);
	system_reset();
	while (1) {
	} /* Loop forever. */

	return 0;

}

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
#include <asf.h>




#include <board.h>
#include "at25dfx.h"


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

static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
//! [buffers]

//! [driver_instances]
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
//! [driver_instances]

// MEMORY MAP
// Code goest to ~ 0x3500 budget until 0x0FFF
// BOOT Status 4100 -> 41FF
// APP 5000 -> TOP
#define BOOT_STATUS_ADDR 0x4100 //boot status address in nvm
int BASE_CODE_ADDR = 0x5000; //nvm application start address
int TOP_CODE_ADDR =  0x8000;  //nvm application end address
#define FLASH_FIRMWARE_HEADER_ADDR 0x0000  //address of header for fw1 in external flash
#define BASE_FLASH_CODE_ADDR 0x10000  //start address of fw 1 in external flash
#define GOLDEN_FLASH_CODE_ADDR 0x20000 // where we store the initial code that was flashed on to the uC
#define INTEGRITY_CHECK 0xdead
#define APP_START_ADDR 0x5000 //start address of application code in nvm (pointer to jump to in the application code)
typedef struct bs {
	uint32_t integrity_check;
	uint8_t signature[3];
	int8_t executing_image;
	int8_t downlaoded_image;	
	uint8_t arr[NVMCTRL_PAGE_SIZE-9]  
}boot_status;

boot_status default_boot_status = { .integrity_check = INTEGRITY_CHECK, .signature = {-1,-1,-1}, .executing_image = 1, .downlaoded_image = -1 };
	
typedef struct fh {
	uint32_t crc;
	uint32_t size;
	uint32_t sw_version;
	uint32_t hw_version;
} firmware_header;


void configure_wdt(void)
{
 /* Create a new configuration structure for the Watchdog settings and fill
 * with the default module settings. */
 struct wdt_conf config_wdt;
 wdt_get_config_defaults(&config_wdt);
 /* Set the Watchdog configuration settings */
 config_wdt.enable = false;
 //config_wdt.timeout_period = WDT_PERIOD_2048CLK;
 //config_wdt.always_on = false;
 /* Initialize and enable the Watchdog with the user settings */
 wdt_set_config(&config_wdt);
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


void configure_nvm(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
}
struct usart_module usart_instance;

void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	#ifdef XPLAINED_BOARD
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;
	stdio_serial_init(&usart_instance, SERCOM4, &config_usart);
	#endif
	#ifndef XPLAINED_BOARD
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PA20D_SERCOM3_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PA21D_SERCOM3_PAD3;
	stdio_serial_init(&usart_instance, SERCOM3, &config_usart);
	#endif
	
	usart_enable(&usart_instance);
}

int main (void)
{
	enum status_code error_code;
	//Set up NVM
	configure_nvm();
	system_init();
	delay_init();
	system_interrupt_enable_global();
	
	configure_usart();
	printf("BOOTLADER CODE RUNNING!!\r\n");
	

	////TODO system clock
	//// Initialization, there are no firmwares and no boot_status
	////Disable watchdog timer
	//configure_wdt();
	
	boot_status status;

	// read out existing boot status
	do
    {
        error_code = nvm_read_buffer(
                BOOT_STATUS_ADDR,
                (void *) &status, NVMCTRL_PAGE_SIZE);
    } while (error_code == STATUS_BUSY);
	if (status.integrity_check != INTEGRITY_CHECK) {
		// boot_status is not valid;
		status = default_boot_status;
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
		
		// save the current code that exists into the "golden image"
		char arr [NVMCTRL_PAGE_SIZE] = {0};	
		// wake up flash chip
		at25dfx_init();
		// Need to copy program from external flash to nvm
		at25dfx_chip_wake(&at25dfx_chip);
		if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
			// Handle missing or non-responsive device
			printf("device not present");
		} //end if
		at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
		at25dfx_chip_set_sector_protect(&at25dfx_chip, GOLDEN_FLASH_CODE_ADDR, false);
		at25dfx_chip_erase_block(&at25dfx_chip, GOLDEN_FLASH_CODE_ADDR, AT25DFX_BLOCK_SIZE_64KB); // erase 64k block for golden firmware
		int nvm_addr = BASE_CODE_ADDR;
		for (int addr = GOLDEN_FLASH_CODE_ADDR; addr < (GOLDEN_FLASH_CODE_ADDR + 0x10000); addr += NVMCTRL_PAGE_SIZE) {
			// read code out of nvm and write to flash
			do
			{
				error_code = nvm_read_buffer(
				nvm_addr,
				(void *) arr, NVMCTRL_PAGE_SIZE);
			} while (error_code == STATUS_BUSY);
			at25dfx_chip_write_buffer(&at25dfx_chip, addr, arr, sizeof(arr));	
			nvm_addr += NVMCTRL_PAGE_SIZE;
		}
		// Done with writing to flash
		at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
		at25dfx_chip_set_sector_protect(&at25dfx_chip, GOLDEN_FLASH_CODE_ADDR, true);
		at25dfx_chip_sleep(&at25dfx_chip);
		printf("BOOTLOADER: Set default boot status and saved golden image\r\n");
		usart_reset(&usart_instance);
		NVIC_SystemReset();
	} 
	else if (status.downlaoded_image != -1)
	{			
			printf("BOOTLADER we have new code to load\r\n");
			at25dfx_init();	
			// Need to copy program from external flash to nvm
			at25dfx_chip_wake(&at25dfx_chip);
			if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
				// Handle missing or non-responsive device
				printf("device not present");
			} //end if
			at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
			at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);
			// read firmware header
			firmware_header flash_header;
			at25dfx_chip_read_buffer(&at25dfx_chip, FLASH_FIRMWARE_HEADER_ADDR, (void *) &flash_header, sizeof (firmware_header));
			
			// Clear out application code that is currently in nvm
			char arr [64] = {0};
				
			for (int addr = BASE_CODE_ADDR; addr < BASE_CODE_ADDR + flash_header.size; addr += 256) {
				//nvm_write_buffer(addr, (void *) arr, sizeof (char) * 64);
				do 
				{
					error_code = nvm_erase_row(addr);
				} while (error_code != STATUS_OK);
				char arr2[256];
				do
				{
					nvm_read_buffer(addr, (void *) arr2, sizeof(arr2));
				} while (error_code != STATUS_OK);
				for (int i = 0; i < 256; i++) {
					printf("%x", arr2[i]);
				}
				
			} //end of for loop
			
			int flash_addr = BASE_FLASH_CODE_ADDR;
			for (int addr = BASE_CODE_ADDR; addr < BASE_CODE_ADDR + flash_header.size; addr += 64) {
				at25dfx_chip_read_buffer(&at25dfx_chip, flash_addr, (void *) arr, sizeof (char) * 64);
				do 
				{
					error_code = nvm_write_buffer(addr, (void *) arr, sizeof (char) * 64);  
				} while (error_code == STATUS_BUSY);
				
				flash_addr += 64;
			} //end for
			printf("BOOTLADER reset bootstatus 2\r\n");
			// TODO FIX ME
			status = default_boot_status;
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
			printf("BOOTLOADER: Firmware moved from flash to nvm.\r\n");
			//usart_reset(&usart_instance); WE SHOULD GO ON AND CHECK THE HEADER
			//NVIC_SystemReset();
			
			
			// Done with reading from flash
			at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
			at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, true);
			at25dfx_chip_sleep(&at25dfx_chip);
			
			// compare with calculated CRC
			uint32_t crc;
			//dsu_crc32_init();
			// if (STATUS_OK == dsu_crc32_cal(BASE_CODE_ADDR, flash_header.size, &crc))
			if (STATUS_OK != crc32_calculate(BASE_CODE_ADDR, flash_header.size, &crc)) {
				printf("BOOTLOADER: CRC failed\r\n");
			}
			printf("CRC atmel: %x CRC me: %x\r\n", crc, flash_header.crc);
			if (crc == flash_header.crc) { // TODO change to false for crc testing
				// GOOD
				
				//image is correctly loaded into nvm. Update the boot status
				status = default_boot_status;
				status.executing_image = flash_header.sw_version;
				status.signature[0] = flash_header.crc;
				do
				{
					error_code = nvm_erase_row(
					BOOT_STATUS_ADDR);
				} while (error_code == STATUS_BUSY);
				char arr2[256];
				do 
				{
					nvm_read_buffer(BOOT_STATUS_ADDR, (void *) arr2, sizeof(arr2));
				} while (error_code != STATUS_OK);
				for (int i = 0; i < 256; i++) {
					printf("%x", arr2[i]);
				}
					
				do
				{
					error_code = nvm_write_buffer(
					BOOT_STATUS_ADDR,
					(void *) &status, NVMCTRL_PAGE_SIZE);
				} while (error_code == STATUS_BUSY);
				usart_reset(&usart_instance);
				NVIC_SystemReset();
			} else {
				printf("BOOTLOADER: firmware download failed, loading golden image\r\n");
				// Need to copy program from external flash to nvm
				at25dfx_chip_wake(&at25dfx_chip);
				if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
					// Handle missing or non-responsive device
					printf("device not present");
				} //end if
				at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
				at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);
				

				char arr [64] = {0};
				// Clear out application code that is currently in nvm				
				for (int addr = BASE_CODE_ADDR; addr < (BASE_CODE_ADDR + (64 * 1024)); addr += 256) { // TODO DEBUG ME
					//nvm_write_buffer(addr, (void *) arr, sizeof (char) * 64);
					do
					{
						error_code = nvm_erase_row(addr);
					} while (error_code != STATUS_OK);
					char arr2[256];
					do 
					{
						nvm_read_buffer(addr, (void *) arr2, sizeof(arr2));
					} while (error_code != STATUS_OK);
					for (int i = 0; i < 256; i++) {
						printf("%x", arr2[i]);
					}
					
				} //end of for loop
				
				int flash_addr = GOLDEN_FLASH_CODE_ADDR;
				for (int addr = BASE_CODE_ADDR; addr < BASE_CODE_ADDR + (64 *1024); addr += 64) {
					at25dfx_chip_read_buffer(&at25dfx_chip, flash_addr, (void *) arr, sizeof (char) * 64);
					do
					{
						error_code = nvm_write_buffer(addr, (void *) arr, sizeof (char) * 64);
					} while (error_code != STATUS_OK);
					char arr2[64];
					do
					{
						nvm_read_buffer(addr, (void *) arr2, sizeof(arr2));
					} while (error_code != STATUS_OK);
					for (int i = 0; i < 64; i++) {
						if(arr[i] != arr2[i]) {
							printf ("DONT MATCH %d \r\n", i);
						}
					}
					
					flash_addr += 64;
				} //end for
				usart_reset(&usart_instance);
				NVIC_SystemReset();
			}
		} //end of status = downloaded_image
		 else if (status.executing_image != -1)
		  {
			printf("BOOTLOADER: Jumping to application code.\r\n");
		    usart_reset(&usart_instance);
			//at25dfx_chip_sleep(&at25dfx_chip);
			// we are running an application
			/* Pointer to the Application Section */
			void (*application_code_entry)(void);
			/* Rebase the Stack Pointer */
			__set_MSP(*(uint32_t *) APP_START_ADDR);
			/* Rebase the vector table base address */
			SCB->VTOR = ((uint32_t) APP_START_ADDR & SCB_VTOR_TBLOFF_Msk);
			/* Load the Reset Handler address of the application */
			application_code_entry = (void (*)(void))(unsigned *)(*(unsigned *) (APP_START_ADDR+ 4));
			/* Jump to user Reset Handler in the application */
			application_code_entry();

		}//end of application code
		else
		{
			/* do nothing*/
		}
		printf ("Welp");
		// infinite loop
		while(1);
}

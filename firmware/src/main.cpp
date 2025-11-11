#include <cstring>
#include <harp_c_app.h>
#include <harp_synchronizer.h>
#include <core_registers.h>
#include <reg_types.h>
#include <config.h>
#include <cstdio> // for printf
#include "hardware/flash.h"
#ifdef DEBUG
    #include <pico/stdlib.h> // for uart printing
    #include <cstdio> // for printf
#endif

#define DEBUG_HARP_MSG_IN

#define FLASH_TARGET_OFFSET (256 * 1024)  // Start writing at 256KB  = 
#define SECTOR_SIZE FLASH_SECTOR_SIZE     // 4096 bytes
#define BUF_SIZE 64
// Define your constants and buffers
#define TOTAL_DATA_SIZE 102400 // This is the total number of bytes
#define TOTAL_16BIT_WORDS (TOTAL_DATA_SIZE / 2) // Total number of 16-bit words
// Temporary buffer for each incoming USB packet (64 bytes)
uint8_t rx_packet_buffer[BUF_SIZE];
// Main buffer to store the complete 16-bit data
uint16_t rx_data_buffer[TOTAL_16BIT_WORDS];
bool write_to_flash = false;
// Keep track of how much data we've received
static uint32_t bytes_received = 0;
static uint16_t words_received = 0;
uint32_t count;
// Buffers for data transfer
uint8_t read_data_buffer[100];

// Create device name array.
const uint16_t who_am_i = 1234;
const uint8_t hw_version_major = 0;
const uint8_t hw_version_minor = 0;
const uint8_t assembly_version = 0;
const uint8_t harp_version_major = 0;
const uint8_t harp_version_minor = 0;
const uint8_t fw_version_major = 0;
const uint8_t fw_version_minor = 0;
const uint16_t serial_number = 0xCAFE;

// Harp App Register Setup.
const size_t reg_count = 17;

void write_any_channel(msg_t& msg);
void erase_flash_memory(msg_t& msg);
void specific_any_waveform(msg_t& msg);

// Function to call the flash programming
// Note: The flash programming function still works with 8-bit bytes
static void call_flash_range_program(void *params) {
    uintptr_t *p = (uintptr_t *)params;
    flash_range_program(p[0], (const uint8_t*)p[1], TOTAL_DATA_SIZE);
}
// Define register contents.
#pragma pack(push, 1)
struct app_regs_t
{
    volatile uint8_t start_channel_1;  // app register 0
    volatile uint8_t start_channel_2;
    volatile uint8_t start_channel_3; 
    volatile uint8_t start_channel_4;

    volatile uint8_t stop_channel_1;
    volatile uint8_t stop_channel_2;
    volatile uint8_t stop_channel_3; 
    volatile uint8_t stop_channel_4;
    
    volatile uint32_t update_rate_1;
    volatile uint32_t update_rate_2;
    volatile uint32_t update_rate_3; 
    volatile uint32_t update_rate_4; 

    volatile uint32_t samples_wave_1;
    volatile uint32_t samples_wave_2;
    volatile uint32_t samples_wave_3; 
    volatile uint32_t samples_wave_4; 

    volatile uint8_t erase_flash;
} app_regs;
#pragma pack(pop)

// Define register "specs."
RegSpecs app_reg_specs[reg_count]
{
    {(uint8_t*)&app_regs.start_channel_1, sizeof(app_regs.start_channel_1), U8},
    {(uint8_t*)&app_regs.start_channel_2, sizeof(app_regs.start_channel_2), U8},
    {(uint8_t*)&app_regs.start_channel_3, sizeof(app_regs.start_channel_3), U8},
    {(uint8_t*)&app_regs.start_channel_4, sizeof(app_regs.start_channel_4), U8},
    {(uint8_t*)&app_regs.stop_channel_1, sizeof(app_regs.stop_channel_1), U8},
    {(uint8_t*)&app_regs.stop_channel_2, sizeof(app_regs.stop_channel_2), U8},
    {(uint8_t*)&app_regs.stop_channel_3, sizeof(app_regs.stop_channel_3), U8},
    {(uint8_t*)&app_regs.stop_channel_4, sizeof(app_regs.stop_channel_4), U8},
    {(uint8_t*)&app_regs.update_rate_1, sizeof(app_regs.update_rate_1), U32},
    {(uint8_t*)&app_regs.update_rate_2, sizeof(app_regs.update_rate_2), U32},
    {(uint8_t*)&app_regs.update_rate_3, sizeof(app_regs.update_rate_3), U32},
    {(uint8_t*)&app_regs.update_rate_4, sizeof(app_regs.update_rate_4), U32},
    {(uint8_t*)&app_regs.samples_wave_1, sizeof(app_regs.samples_wave_1), U32},
    {(uint8_t*)&app_regs.samples_wave_2, sizeof(app_regs.samples_wave_2), U32},
    {(uint8_t*)&app_regs.samples_wave_3, sizeof(app_regs.samples_wave_3), U32},
    {(uint8_t*)&app_regs.samples_wave_4, sizeof(app_regs.samples_wave_4), U32},
    {(uint8_t*)&app_regs.erase_flash, sizeof(app_regs.erase_flash), U8}
};

// Define register read-and-write handler functions.
RegFnPair reg_handler_fns[reg_count]
{
    {&HarpCore::read_reg_generic, write_any_channel},
    {&HarpCore::read_reg_generic, write_any_channel},
    {&HarpCore::read_reg_generic, write_any_channel},
    {&HarpCore::read_reg_generic, write_any_channel},
    {&HarpCore::read_reg_generic, write_any_channel},
    {&HarpCore::read_reg_generic, write_any_channel},
    {&HarpCore::read_reg_generic, write_any_channel},
    {&HarpCore::read_reg_generic, write_any_channel},
    
    {&HarpCore::read_reg_generic, specific_any_waveform},
    {&HarpCore::read_reg_generic, specific_any_waveform},
    {&HarpCore::read_reg_generic, specific_any_waveform},
    {&HarpCore::read_reg_generic, specific_any_waveform},
    {&HarpCore::read_reg_generic, specific_any_waveform},
    {&HarpCore::read_reg_generic, specific_any_waveform},
    {&HarpCore::read_reg_generic, specific_any_waveform},
    {&HarpCore::read_reg_generic, specific_any_waveform},

    {&HarpCore::read_reg_generic, erase_flash_memory}
};

void write_any_channel(msg_t& msg)
{
    HarpCore::copy_msg_payload_to_register(msg);
    uint8_t channel_index = msg.header.address - APP_REG_START_ADDRESS;
}

void specific_any_waveform(msg_t& msg)
{
    HarpCore::copy_msg_payload_to_register(msg);
}

void erase_flash_memory(msg_t& msg)
{
    HarpCore::copy_msg_payload_to_register(msg);
    if(app_regs.erase_flash == 1)
    {
        printf("Erasing Flash Memory !\r\n");
        uint32_t ints = save_and_disable_interrupts();
        // for PI PICO RP2530
        // we need to ereas (4 Mbyte - 256 Kbyte) = 3932160 byte 
        // (write it in term of Sector_Size as follow  960*4*1024 = 3932160 bytes = 3840 Kbyte)
        // flash_range_erase(FLASH_TARGET_OFFSET  , 960*SECTOR_SIZE);
        // for PI PICO RP2040
        // we need to ereas (2 Mbyte - 256 Kbyte) = 1835008 byte 
        // (write it in term of Sector_Size as follow  448*4*1024 = 1835008 bytes = 1792 Kbyte)
        flash_range_erase(FLASH_TARGET_OFFSET  , 448*SECTOR_SIZE);
        // Restore interrupts
        restore_interrupts(ints);
        printf("Done Erasing Flash Memory !\r\n");
    }
}

void app_reset()
{
    app_regs.start_channel_1 = 0;
    app_regs.start_channel_2 = 0;
    app_regs.start_channel_3 = 0;
    app_regs.start_channel_4 = 0;

    app_regs.stop_channel_1 = 0;
    app_regs.stop_channel_2 = 0;
    app_regs.stop_channel_3 = 0;
    app_regs.stop_channel_4 = 0;

    app_regs.update_rate_1 = 0;
    app_regs.update_rate_2 = 0;
    app_regs.update_rate_3 = 0;
    app_regs.update_rate_4 = 0;

    app_regs.samples_wave_1 = 0;
    app_regs.samples_wave_2 = 0;
    app_regs.samples_wave_3 = 0;
    app_regs.samples_wave_4 = 0;

    app_regs.erase_flash = 0;

}

void update_app_state()
{
    // update here!
    printf("Hello, from an Quad DAC!\r\n");
    // If app registers update their states outside the read/write handler
    // functions, update them here.
    // (Called inside run() function.)
}
#ifndef LED_DELAY_MS
#define LED_DELAY_MS 100
#endif

#ifndef PICO_DEFAULT_LED_PIN
#warning blink_simple example requires a board with a regular LED
#endif

// Initialize the GPIO for the LED
void pico_led_init(void) {
#ifdef PICO_DEFAULT_LED_PIN
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
}

void gpio_callback(uint gpio, uint32_t events)
{
    uint32_t gpio_state = gpio_get_all();
    /*app_regs.di_state = 0;
    app_regs.di_state |= (gpio_state & 0xC) >> 2;
    app_regs.di_state |= (gpio_state & 0x7000) >> 10;
    */
    HarpCore::send_harp_reply(EVENT, APP_REG_START_ADDRESS);
}

void configure_gpio(void)
{
    gpio_set_irq_callback(gpio_callback);
}

// Turn the LED on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#endif
}
// Create Harp App.
HarpCApp& app = HarpCApp::init(HARP_DEVICE_ID,
                               HW_VERSION_MAJOR, HW_VERSION_MINOR,
                               HW_ASSEMBLY_VERSION,
                               HARP_VERSION_MAJOR, HARP_VERSION_MINOR,
                               FW_VERSION_MAJOR, FW_VERSION_MINOR,
                               UNUSED_SERIAL_NUMBER, "Quad DAC",
                               (uint8_t*)GIT_HASH,
                               &app_regs, app_reg_specs,
                               reg_handler_fns, reg_count, update_app_state,
                               app_reset);

// Core0 main.
int main()
{
    pico_led_init();
// Init Synchronizer.
    HarpSynchronizer::init(uart1, HARP_SYNC_RX_PIN);
    app.set_synchronizer(&HarpSynchronizer::instance());
    configure_gpio();
    #ifdef DEBUG
    stdio_uart_init_full(uart0, 921600, UART_TX_PIN, -1); // use uart1 tx only.
    printf("Hello, from Quad DAC!\r\n");
#endif
    while(true)
    {
        printf("Hello, from Quad DAC!\r\n");

        if (tud_vendor_available()) {
            uint32_t count = tud_vendor_read(rx_packet_buffer, sizeof(rx_packet_buffer));

            // Ensure the data size is an even number for 16-bit conversion
            if (count % 2 != 0) {
                printf("Error: Received odd number of bytes, 16-bit data expected.\n");
                bytes_received = 0;
                words_received = 0;
                continue;
            }

            // Check if we have enough space in our main buffer (in bytes)
            if (bytes_received + count <= TOTAL_DATA_SIZE) {
                // Manually convert 8-bit bytes into 16-bit words
                for (uint32_t i = 0; i < count; i += 2) {
                    // Combine two 8-bit bytes into a single 16-bit word
                    rx_data_buffer[words_received] = (uint16_t)rx_packet_buffer[i] | ((uint16_t)rx_packet_buffer[i + 1] << 8);
                    words_received++;
                }
                bytes_received += count;

                if (bytes_received == TOTAL_DATA_SIZE) {
                    write_to_flash = true;
                    printf("Full 16-bit data array received, preparing to write to flash.\n");
                }
            } else {
                printf("Error: Buffer overflow. Data dropped.\n");
                bytes_received = 0;
                words_received = 0;
            }
        }

        if (write_to_flash) {
            uintptr_t params[] = {FLASH_TARGET_OFFSET, (uintptr_t)rx_data_buffer};
            uint32_t rc;

            printf("Writing to flash... DO NOT unplug.\n");
            // Cast the 16-bit buffer to a void pointer for the flash function
            rc = flash_safe_execute(call_flash_range_program, params, TOTAL_DATA_SIZE);

            if (rc == PICO_OK) {
                printf("Flash write completed successfully.\n");
                const uint8_t *flash_read_address = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

                memcpy(read_data_buffer, flash_read_address, 100);

                printf("Read for 16-bit data...\n");
            } else {
                printf("Flash write failed with error code: %d\n", rc);
            }
            
            bytes_received = 0;
            words_received = 0;
            write_to_flash = false;
        }
        app.run();
    }
}

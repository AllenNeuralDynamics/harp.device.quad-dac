#include <cstring>
#include <harp_c_app.h>
#include <harp_synchronizer.h>
#include <core_registers.h>
#include <reg_types.h>
#include <config.h>
#include <cstdio> // for printf
#ifdef DEBUG
    #include <pico/stdlib.h> // for uart printing
    #include <cstdio> // for printf
#endif

#define DEBUG_HARP_MSG_IN


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
const size_t reg_count = 8;

void write_any_channel(msg_t& msg);
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
    {(uint8_t*)&app_regs.stop_channel_4, sizeof(app_regs.stop_channel_4), U8}
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
    {&HarpCore::read_reg_generic, write_any_channel}
};

void write_any_channel(msg_t& msg)
{
    HarpCore::copy_msg_payload_to_register(msg);
    uint8_t channel_index = msg.header.address - APP_REG_START_ADDRESS;
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
        app.run();
    }
}

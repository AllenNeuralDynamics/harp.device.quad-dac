#include <cstring>
#include <harp_c_app.h>
#include <harp_synchronizer.h>
#include <core_registers.h>
#include <reg_types.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <pico/util/queue.h>
#include <pico/stdlib.h>

// Create device name array.
const uint16_t who_am_i = 123;
const uint8_t hw_version_major = 1;
const uint8_t hw_version_minor = 0;
const uint8_t assembly_version = 0;
const uint8_t harp_version_major = 2;
const uint8_t harp_version_minor = 0;
const uint8_t fw_version_major = 0;
const uint8_t fw_version_minor = 1;
const uint16_t serial_number = 0x0;

// Harp App setup.
const uint32_t DO0_PIN = 15;
const uint32_t DI_MASK = 0x700C;
const uint32_t DO_MASK = 0xFF << DO0_PIN;
const uint32_t AI0_PIN = 26;
const uint32_t AI1_PIN = 27;
const uint32_t AI2_PIN = 28;
const uint32_t AI_MASK = 0x7;

// Harp App state.
bool events_active = false;

// Repeating timers for pulse control
const size_t pulse_train_count = 256;



struct pulse_train_t
{
    struct repeating_timer timer;
    uint8_t output_mask;
    uint32_t pulse_width_us;
    uint32_t pulse_period_us;
    uint32_t pulse_count;
};
pulse_train_t pulse_train_timers[pulse_train_count];

// Repeating timer and buffers for ADC sampling using 
// Pointer to an address is required for the reinitialization DMA channel.
uint16_t adc_vals[3] = {0, 0, 0};
uint16_t* data_ptr[1] = {adc_vals};
struct repeating_timer adc_timer;
const int32_t adc_period_us = 4000;
const int32_t adc_callback_delay_us = 80000;
int adc_sample_channel;
int adc_ctrl_channel;
static queue_t adc_queue;

// Define queue item contents
#pragma pack(push, 1)
struct adc_queue_item_t
{
    uint64_t timestamp;
    uint16_t analog_data[3];
};
#pragma pack(pop)
adc_queue_item_t adc_queue_current;

// Harp App Register Setup.
const size_t reg_count = 8;

// Define register contents.
#pragma pack(push, 1)
struct app_regs_t
{
    volatile uint8_t di_state;
    volatile uint8_t do_set;
    volatile uint8_t do_clear;
    volatile uint8_t do_toggle;
    volatile uint8_t do_state;
    volatile uint32_t start_pulse_train[4];
    volatile uint8_t stop_pulse_train;
    volatile uint16_t analog_data[3];
} app_regs;
#pragma pack(pop)

// Define register "specs."
RegSpecs app_reg_specs[reg_count]
{
    {(uint8_t*)&app_regs.di_state, sizeof(app_regs.di_state), U8},  // address 32
    {(uint8_t*)&app_regs.do_set, sizeof(app_regs.do_set), U8},   // address 33
    {(uint8_t*)&app_regs.do_clear, sizeof(app_regs.do_clear), U8},  // 34
    {(uint8_t*)&app_regs.do_toggle, sizeof(app_regs.do_toggle), U8}, // 35
    {(uint8_t*)&app_regs.do_state, sizeof(app_regs.do_state), U8},  // 36
    {(uint8_t*)&app_regs.start_pulse_train, sizeof(app_regs.start_pulse_train), U32},
    {(uint8_t*)&app_regs.stop_pulse_train, sizeof(app_regs.stop_pulse_train), U8},
    {(uint8_t*)&app_regs.analog_data, sizeof(app_regs.analog_data), U16}
};

void gpio_callback(uint gpio, uint32_t events)
{
    uint32_t gpio_state = gpio_get_all();
    app_regs.di_state = 0;
    app_regs.di_state |= (gpio_state & 0xC) >> 2;
    app_regs.di_state |= (gpio_state & 0x7000) >> 10;
    HarpCore::send_harp_reply(EVENT, APP_REG_START_ADDRESS);
}

void write_do_set(msg_t &msg)
{
    gpio_put(25,true);
    sleep_ms(100);
    HarpCore::copy_msg_payload_to_register(msg);
    gpio_set_mask(app_regs.do_set << DO0_PIN);
    HarpCore::send_harp_reply(WRITE, msg.header.address);
}

void write_do_clear(msg_t &msg)
{
    gpio_put(25,false);
    sleep_ms(100);
    HarpCore::copy_msg_payload_to_register(msg);
    gpio_clr_mask(app_regs.do_clear << DO0_PIN);
    HarpCore::send_harp_reply(WRITE, msg.header.address);
}

void write_do_toggle(msg_t &msg)
{
    gpio_put(25,true);
    sleep_ms(100);
    HarpCore::copy_msg_payload_to_register(msg);
    gpio_xor_mask(app_regs.do_toggle << DO0_PIN);
    HarpCore::send_harp_reply(WRITE, msg.header.address);
}

void write_do_state(msg_t &msg)
{
    gpio_put(25,false);
    sleep_ms(100);
    HarpCore::copy_msg_payload_to_register(msg);
    gpio_put_masked(DO_MASK, app_regs.do_state << DO0_PIN);
    HarpCore::send_harp_reply(WRITE, msg.header.address);
}

int64_t pulse_callback(alarm_id_t id, void *user_data)
{
    pulse_train_t *pulse_train = (pulse_train_t *)user_data;
    app_regs.do_clear = pulse_train->output_mask;
    gpio_clr_mask(pulse_train->output_mask << DO0_PIN);

    // Emit stop notifications for pulse and pulse train
    uint64_t harp_time_us = HarpCore::harp_time_us_64();
    HarpCore::send_harp_reply(EVENT, APP_REG_START_ADDRESS + 2, harp_time_us);
    if (pulse_train->timer.delay_us == 0)
    {
        // Mark timer as cancelled if pulse train stops
        pulse_train->timer.alarm_id = 0;
        app_regs.stop_pulse_train = pulse_train->output_mask;
        HarpCore::send_harp_reply(EVENT, APP_REG_START_ADDRESS + 6, harp_time_us);
    }
    return 0;
}

bool pulse_train_callback(repeating_timer_t *rt)
{
    pulse_train_t *pulse_train = (pulse_train_t *)rt->user_data;

    // Configure the repeating timer delay following the first pulse
    pulse_train->timer.delay_us = -((int64_t)pulse_train->pulse_period_us);

    // Stop pulse train if positive counter falls to zero;
    // counters which started zero or negative repeat indefinitely
    if (pulse_train->pulse_count > 0 && --pulse_train->pulse_count == 0)
    {
        pulse_train->timer.delay_us = 0;
    }

    // For every pulse in the pulse train, arm an alarm matching the pulse width
    add_alarm_in_us(pulse_train->pulse_width_us, pulse_callback, pulse_train, true);

    gpio_set_mask(pulse_train->output_mask << DO0_PIN);
    HarpCore::send_harp_reply(EVENT, APP_REG_START_ADDRESS + 1);
    return pulse_train->timer.delay_us != 0;
}

void write_start_pulse_train(msg_t& msg)
{
    HarpCore::copy_msg_payload_to_register(msg);

    // Configure pulse train parameters
    uint8_t output_mask = (uint8_t)((app_regs.start_pulse_train[0] & 0xFF));
    pulse_train_t *pulse_train = &pulse_train_timers[output_mask];
    pulse_train->output_mask = output_mask;
    pulse_train->pulse_width_us = app_regs.start_pulse_train[1];
    pulse_train->pulse_period_us = app_regs.start_pulse_train[2];
    pulse_train->pulse_count = app_regs.start_pulse_train[3];
    
    // Cancel any existing timer
    if (cancel_repeating_timer(&pulse_train->timer))
    {
        app_regs.stop_pulse_train = output_mask;
        HarpCore::send_harp_reply(EVENT, APP_REG_START_ADDRESS + 6);
    }

    // Arm repeating timer and immediately arm the first pulse
    HarpCore::send_harp_reply(WRITE, msg.header.address);
    add_repeating_timer_us(0, pulse_train_callback, pulse_train, &pulse_train->timer);
}

void write_stop_pulse_train(msg_t& msg)
{
    HarpCore::copy_msg_payload_to_register(msg);

    uint8_t output_mask = (uint8_t)((app_regs.start_pulse_train[0] & 0xFF));
    cancel_repeating_timer(&pulse_train_timers[output_mask].timer);

    HarpCore::send_harp_reply(WRITE, msg.header.address);
}

bool adc_callback(repeating_timer_t *rt)
{
    if (!HarpCore::events_enabled())
        return false;

    rt->delay_us = -adc_period_us;
    
    // Mask the values to 12 bits (0xFFF) to ensure only valid ADC bits are used
    adc_queue_item_t item;
    item.timestamp = HarpCore::harp_time_us_64();
    item.analog_data[0] = adc_vals[0] & 0xFFF;
    item.analog_data[1] = adc_vals[1] & 0xFFF;
    item.analog_data[2] = adc_vals[2] & 0xFFF;
    queue_add_blocking(&adc_queue, &item);
    return true;
}

// Define register read-and-write handler functions.
RegFnPair reg_handler_fns[reg_count]
{
    {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
    {&HarpCore::read_reg_generic, &write_do_set},
    {&HarpCore::read_reg_generic, &write_do_clear},
    {&HarpCore::read_reg_generic, &write_do_toggle},
    {&HarpCore::read_reg_generic, &write_do_state},
    {&HarpCore::read_reg_generic, &write_start_pulse_train},
    {&HarpCore::read_reg_generic, &write_stop_pulse_train},
    {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error}
};

void app_reset()
{
    app_regs.di_state = 0;
    app_regs.do_set = 0;
    app_regs.do_clear = 0;
    app_regs.do_toggle = 0;
    app_regs.do_state = 0;
    app_regs.start_pulse_train[0] = 0;
    app_regs.start_pulse_train[1] = 0;
    app_regs.start_pulse_train[2] = 0;
    app_regs.start_pulse_train[3] = 0;
    app_regs.stop_pulse_train = 0;
    app_regs.analog_data[0] = 0;
    app_regs.analog_data[1] = 0;
    app_regs.analog_data[2] = 0;
}

void configure_gpio(void)
{
    gpio_init_mask(DO_MASK | DI_MASK);
    gpio_set_dir_out_masked(DO_MASK);
    gpio_set_dir_in_masked(DI_MASK);
    gpio_clr_mask(DO_MASK);

    gpio_set_irq_callback(gpio_callback);
    gpio_set_irq_enabled(2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(3, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(12, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(13, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(14, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
}

void enable_gpio(bool enabled)
{
    irq_set_enabled(IO_IRQ_BANK0, enabled);
}

void configure_adc(void)
{
    adc_gpio_init(AI0_PIN);
    adc_gpio_init(AI1_PIN);
    adc_gpio_init(AI2_PIN);

    adc_init();
    adc_set_clkdiv(0); // Run conversion back-to-back at full speed.
    adc_set_round_robin(AI_MASK); // Enable round-robin sampling of all 3 inputs.
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false    // We won't byte-shift since we will be using the full ADC bit-depth.
    );

    // Get two open DMA channels.
    // adc_sample_channel will sample the adc, paced by DREQ_ADC and chain to adc_ctrl_channel.
    // adc_ctrl_channel will reconfigure & retrigger adc_sample_channel when it finishes.
    adc_sample_channel = dma_claim_unused_channel(true);
    adc_ctrl_channel = dma_claim_unused_channel(true);
    dma_channel_config sample_config = dma_channel_get_default_config(adc_sample_channel);
    dma_channel_config ctrl_config = dma_channel_get_default_config(adc_ctrl_channel);

    // Setup Sample Channel.
    channel_config_set_transfer_data_size(&sample_config, DMA_SIZE_16);
    channel_config_set_read_increment(&sample_config, false); // read from adc FIFO reg.
    channel_config_set_write_increment(&sample_config, true);
    channel_config_set_irq_quiet(&sample_config, true);
    channel_config_set_dreq(&sample_config, DREQ_ADC); // pace data according to ADC
    channel_config_set_chain_to(&sample_config, adc_ctrl_channel);
    channel_config_set_enable(&sample_config, true);

    // Apply adc_sample_channel configuration.
    dma_channel_configure(
        adc_sample_channel, // Channel to be configured
        &sample_config,
        nullptr,            // write (dst) address will be loaded by adc_ctrl_channel.
        &adc_hw->fifo,      // read (source) address. Does not change.
        count_of(adc_vals), // Number of word transfers.
        false               // Don't Start immediately.
    );

    // Setup Reconfiguration Channel
    // This channel will Write the starting address to the write address
    // "trigger" register, which will restart the DMA Sample Channel.
    channel_config_set_transfer_data_size(&ctrl_config, DMA_SIZE_32);
    channel_config_set_read_increment(&ctrl_config, false); // read a single uint32.
    channel_config_set_write_increment(&ctrl_config, false);
    channel_config_set_irq_quiet(&ctrl_config, true);
    channel_config_set_dreq(&ctrl_config, DREQ_FORCE); // Go as fast as possible.
    channel_config_set_enable(&ctrl_config, true);

    // Apply reconfig channel configuration.
    dma_channel_configure(
        adc_ctrl_channel,  // Channel to be configured
        &ctrl_config,
        &dma_hw->ch[adc_sample_channel].al2_write_addr_trig, // dst address. Retrigger on write.
        data_ptr,      // Read (src) address is a single array with the starting address.
        1,             // Number of word transfers.
        false          // Don't Start immediately.
    );

    // Configure queue for storing sample data and avoid concurrency in
    // outbound message buffers, i.e. avoid sending reply in timer callback.
    queue_init(&adc_queue, sizeof(adc_queue_item_t), 2);
}

void enable_adc_events()
{
    // Set starting ADC channel for round-robin mode.
    adc_select_input(0);

    // Start free-running ADC and DMA transfer
    dma_channel_start(adc_ctrl_channel);
    adc_run(true);

    // Setup repeating timer for reporting values back to the host.
    add_repeating_timer_us(-adc_callback_delay_us, adc_callback, NULL, &adc_timer);
}

void disable_adc_events()
{
    // Ensure both DMA channels are fully stopped
    // Note: loop is needed since dma_channel_abort does not wait for CHAN_ABORT
    // https://github.com/raspberrypi/pico-sdk/issues/923
    while (dma_channel_is_busy(adc_ctrl_channel) || dma_channel_is_busy(adc_sample_channel)) {
        dma_channel_abort(adc_ctrl_channel);
        dma_channel_abort(adc_sample_channel);
    }

    // Stop the ADC and drain the FIFO.
    adc_run(false);
    adc_fifo_drain();
}

void cancel_pulse_timers()
{
    // Cancel any pulse train timer which might be still running
    for (size_t i = 0; i < pulse_train_count; i++)
    {
        cancel_repeating_timer(&pulse_train_timers[i].timer);
    }
}

void update_app_state()
{
    // Enable or disable asynchronous register updates depending on app state
    if (!events_active && HarpCore::events_enabled())
    {
        // enable events
        enable_gpio(true);
        enable_adc_events();
        events_active = true;
    }
    else if (events_active && !HarpCore::events_enabled())
    {
        // disable events
        enable_gpio(false);
        disable_adc_events();
        cancel_pulse_timers();
        events_active = false;
    }

    if (events_active && queue_try_remove(&adc_queue, &adc_queue_current))
    {
        app_regs.analog_data[0] = adc_queue_current.analog_data[0];
        app_regs.analog_data[1] = adc_queue_current.analog_data[1];
        app_regs.analog_data[2] = adc_queue_current.analog_data[2];
        HarpCore::send_harp_reply(EVENT, APP_REG_START_ADDRESS + 7, adc_queue_current.timestamp);
    }
}

// Create Harp App.
HarpCApp& app = HarpCApp::init(who_am_i, hw_version_major, hw_version_minor,
                               assembly_version,
                               harp_version_major, harp_version_minor,
                               fw_version_major, fw_version_minor,
                               serial_number, "Quad-DAC",
                               (const uint8_t*)GIT_HASH, // in CMakeLists.txt.
                               &app_regs, app_reg_specs,
                               reg_handler_fns, reg_count, update_app_state,
                               app_reset);

// Core0 main.
int main()
{
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    sleep_ms(100);
    // Init Synchronizer.
    HarpSynchronizer& sync = HarpSynchronizer::init(uart1, 5);
    app.set_synchronizer(&sync);
    configure_gpio();
    configure_adc();
    
    while(true)
    {
        app.run();
    }
}

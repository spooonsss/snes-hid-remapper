/*

cd ~/Downloads/
cd hidrdd-1.1.27
../Regina396w64/rexx.exe  rd.rex  -d -f snes.h -o snes.txt


cd Documents/hid-remapper/firmware
bash ~/bin/diffuse.sh  -m



cd Documents/hid-remapper/
cd firmware/build
make



cd Documents\hid-remapper\firmware\build
copy remapper_snes.uf2 d:\

*/


#include "descriptor_parser.h"
#include "interval_override.h"
#include "remapper.h"
#include "tick.h"
#include "interval_override.h"

#include "hardware/pio.h"
#include "snes_shift_register.pio.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"

#include <stdio.h>

#include "pico/stdlib.h"

// #define SERIAL_DEBUG_UART uart0
// #define SERIAL_DEBUG_TX_PIN 12

const static uint CLK_PIN = 2;
const static uint LATCH_PIN = CLK_PIN + 1; // aka "parallel/serial control"
const static uint DATA_PIN = LATCH_PIN + 1;


#define FAKE_VID 0x0001
#define FAKE_PID 0x0001
#define FAKE_INTERFACE 0x0101


static PIO pio;
static uint sm = 3;


// typedef struct
// {
//                                                      // No REPORT ID byte
//                                                      // Collection: CA:GamePad
//   uint8_t  BTN_GamePadButton1 : 1;                   // Usage 0x00090001: Button 1 Primary/trigger, Value = 0 to 1
//   uint8_t  BTN_GamePadButton2 : 1;                   // Usage 0x00090002: Button 2 Secondary, Value = 0 to 1
//   uint8_t  BTN_GamePadButton3 : 1;                   // Usage 0x00090003: Button 3 Tertiary, Value = 0 to 1
//   uint8_t  BTN_GamePadButton4 : 1;                   // Usage 0x00090004: Button 4, Value = 0 to 1
//   uint8_t  BTN_GamePadButton5 : 1;                   // Usage 0x00090005: Button 5, Value = 0 to 1
//   uint8_t  BTN_GamePadButton6 : 1;                   // Usage 0x00090006: Button 6, Value = 0 to 1
//   uint8_t  BTN_GamePadButton7 : 1;                   // Usage 0x00090007: Button 7, Value = 0 to 1
//   uint8_t  BTN_GamePadButton8 : 1;                   // Usage 0x00090008: Button 8, Value = 0 to 1
//                                                      // Collection: CA:GamePad CP:Pointer
//   int8_t   GD_GamePadPointerY : 2;                   // Usage 0x00010031: Y, Value = -1 to 1
//   int8_t   GD_GamePadPointerX : 2;                   // Usage 0x00010030: X, Value = -1 to 1
//                                                      // Collection: CA:GamePad
//   uint8_t  GD_GamePadHatSwitch : 4;                  // Usage 0x00010039: Hat switch, Value = 0 to 7, Physical = Value x 45 in degrees
// } report_t;

typedef struct 
{
    uint8_t b1;
    uint8_t b2;
} report_t;

report_t report = { .b1 = 0, .b2 = 0 };
report_t last_report = { .b1 = 0xFF, .b2 = 0xFF };


const uint8_t fake_descriptor[] = {
0x05, 0x01, //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
0x09, 0x05, //     (LOCAL)  USAGE              0x00010005 Game Pad (Application Collection)  
0xA1, 0x01, //     (MAIN)   COLLECTION         0x01 Application (Usage=0x00010005: Page=Generic Desktop Page, Usage=Game Pad, Type=Application Collection)
0x05, 0x09, //       (GLOBAL) USAGE_PAGE         0x0009 Button Page 
0x19, 0x01, //       (LOCAL)  USAGE_MINIMUM      0x00090001 Button 1 Primary/trigger (Selector, On/Off Control, Momentary Control, or One Shot Control)  
0x29, 0x08, //       (LOCAL)  USAGE_MAXIMUM      0x00090008 Button 8 (Selector, On/Off Control, Momentary Control, or One Shot Control)  
0x14, //          (GLOBAL) LOGICAL_MINIMUM    (0)  
0x25, 0x01, //       (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
0x95, 0x08, //       (GLOBAL) REPORT_COUNT       0x08 (8) Number of fields  
0x75, 0x01, //       (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
0x81, 0x02, //       (MAIN)   INPUT              0x00000002 (8 fields x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 



0x05, 0x01, //       (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
0x09, 0x01, //       (LOCAL)  USAGE              0x00010001 Pointer (Physical Collection)  
//0x05, 0x01, //       (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
0xA1, 0x00, //       (MAIN)   COLLECTION         0x00 Physical (Usage=0x00010001: Page=Generic Desktop Page, Usage=Pointer, Type=Physical Collection)
0x09, 0x31, //         (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
0x09, 0x30, //         (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
0x15, 0xFF, //           (GLOBAL) LOGICAL_MINIMUM    0xFF (-1)  
//0x25, 0x01, //       (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
0x75, 0x02, //         (GLOBAL) REPORT_SIZE        0x02 (2) Number of bits per field  
0x95, 0x02, //         (GLOBAL) REPORT_COUNT       0x02 (2) Number of fields  
0x81, 0x02, //         (MAIN)   INPUT              0x00000002 (2 fields x 2 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
0xC0, //          (MAIN)   END_COLLECTION     Physical 



//0x05, 0x01,       //    (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
0x09, 0x39,       //    (LOCAL)  USAGE              0x00010039 Hat switch (Dynamic Value)  
0x14,             // (GLOBAL) LOGICAL_MINIMUM    (0)
0x25, 0x07,       //    (GLOBAL) LOGICAL_MAXIMUM    0x07 (7)  
0x34,             // (GLOBAL) PHYSICAL_MINIMUM   (0)  
0x46, 0x3B, 0x01,  //       (GLOBAL) PHYSICAL_MAXIMUM   0x013B (315)  
0x65, 0x14,       //   (GLOBAL) UNIT               0x14 Rotation in degrees [1° units] (4=System=English Rotation, 1=Rotation=Degrees)  
0x75, 0x04,       //    (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field  
0x95, 0x01,       //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
0x81, 0x42,       //    (MAIN)   INPUT              0x00000042 (1 field x 4 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 1=Null 0=NonVolatile 0=Bitmap 


//0x95, 0x01, //       (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
//0x75, 0x04, //       (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field  
//0x81, 0x01, //       (MAIN)   INPUT              0x00000001 (1 field x 4 bits) 1=Constant 0=Array 0=Absolute 
0xC0          //  (MAIN)   END_COLLECTION     Application  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=315) UNIT(0x00000014,EXP=0)
};


void extra_init() {
#ifdef SERIAL_DEBUG_UART
    gpio_set_function(SERIAL_DEBUG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SERIAL_DEBUG_TX_PIN + 1, GPIO_FUNC_UART);
    // gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    uart_init(SERIAL_DEBUG_UART, 115200);
    uart_set_hw_flow(SERIAL_DEBUG_UART, false, false);
    uart_set_translate_crlf(SERIAL_DEBUG_UART, false);
    uart_set_format(SERIAL_DEBUG_UART, 8, 1, UART_PARITY_NONE);

    uart_puts(SERIAL_DEBUG_UART, "RP2040 SNES init\n");
#endif

    pio = pio1;
    uint offset = pio_add_program(pio, &snes_shift_register_program);
    // printf("Loaded program at %d\n", offset);

    snes_shift_register_program_init(pio, sm, offset, 2);

    pio_gpio_init(pio, LATCH_PIN);
    pio_gpio_init(pio, CLK_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, CLK_PIN, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_PIN, 1, false);

    pio_sm_config c = snes_shift_register_program_get_default_config(offset);

    sm_config_set_set_pins(&c, CLK_PIN, 2);
    sm_config_set_out_pins(&c, CLK_PIN, 2);
    sm_config_set_in_pins(&c, DATA_PIN);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
    sm_config_set_in_shift(&c, true, false, 32);

    sm_config_set_clkdiv_int_frac(&c, 125, 0); // set 1MHz clock; PICO clock is 125MHz

    pio_sm_init(pio, sm, offset, &c);

    pio_sm_set_enabled(pio, sm, true);

    parse_descriptor(FAKE_VID, FAKE_PID, fake_descriptor, sizeof(fake_descriptor), FAKE_INTERFACE, 1);
}

uint32_t get_gpio_valid_pins_mask() {
    return GPIO_VALID_PINS_BASE & ~(
#ifdef PICO_DEFAULT_UART_TX_PIN
                                      (1 << PICO_DEFAULT_UART_TX_PIN) |
#endif
#ifdef PICO_DEFAULT_UART_RX_PIN
                                      (1 << PICO_DEFAULT_UART_RX_PIN) |
#endif
#ifdef SERIAL_DEBUG_UART
                                      (1 << SERIAL_DEBUG_TX_PIN) |
#endif
                                      (1 << CLK_PIN) | (LATCH_PIN << 3) | (DATA_PIN << 4)
                                      );

//                                              (1 << PICO_DEFAULT_LED_PIN));

}


#ifdef SERIAL_DEBUG_UART
static uint32_t loop_count = 0;
static uint32_t report_count = 0;
#endif

uint64_t prev_report = 0;
uint64_t last_debug_time = 0;
void read_report(bool* new_report, bool* tick) {
    *tick = get_and_clear_tick_pending();
    *new_report = false;

    // <select class="form-select" id="interval_override_dropdown">
    //     <option value="0">don't override</option>
    //     <option value="16">62.5 Hz</option>
    //     <option value="8">125 Hz</option>
    //     <option value="4">250 Hz</option>
    //     <option value="2">500 Hz</option>
    //     <option value="1">1000 Hz</option>
    // </select>

    uint64_t io = interval_override;
    uint64_t interval = io == 0 ? 8 : io;


    //if (*tick) {
    //if (now - prev_report >= interval * (uint64_t)1000 * 20) {
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        io_ro_32 *rxfifo_shift = (io_ro_32 *)&pio->rxf[sm];
        uint32_t buttons = (uint32_t)*rxfifo_shift;

        // `buttons` high two bytes:
        // 1 (LSB)         B
        // 2               Y
        // 3               Select
        // 4               Start
        // 5               Up on joypad
        // 6               Down on joypad
        // 7               Left on joypad
        // 8               Right on joypad
        // 9               A
        // 10              X
        // 11              L
        // 12              R
        // 13 -
        // 14 -
        // 15 -
        // 16 (MSB) -

        // report.b1 = ((buttons >> 8 >> 4) & 0x0F) | (buttons & 0xF0);
        report.b1 = ((buttons >> 20) & 0xF0) | ((buttons >> 16) & 0x0F);
        report.b1 ^= 0xFF; // active low
        //report.b1 = 1;
        uint8_t dpad_lookup[] = {
            0b10,
            0b11,
            0b01,
            0b00,
        };
        uint8_t y_dpad = ((buttons >> 20) & 0x03);
        uint8_t x_dpad = ((buttons >> 22) & 0x03);
        uint8_t dpad = dpad_lookup[y_dpad] | (dpad_lookup[x_dpad] << 2);

        uint8_t hat_lookup[] = {
            0xF,
            0,
            4,
            0xF,
            6, // 4
            7, 
            5,
            0xF,
            2, // 8
            1,
            3,
            0xF,
            0xF, // 12
            0xF,
            0xF,
            0xF,
        };
        report.b2 = dpad | (hat_lookup[((buttons >> 20) & 0x0F) ^ 0xF] << 4);

        // report.b1 ^= 0x0F;
        // report.b2 ^= 0x01;
        // report.b2 ^= 0x10;
        if (report.b1 != last_report.b1 || report.b2 != last_report.b2) {
            last_report = report;
            handle_received_report((const uint8_t*) &report, sizeof(report), FAKE_INTERFACE);
            *new_report = true;
            // prev_report = now;
        }
#ifdef SERIAL_DEBUG_UART
        report_count++;
#endif
    }

#ifdef SERIAL_DEBUG_UART
    uint64_t now = time_us_64();
//    if (loop_count++ == 10000) {
    if (now - last_debug_time > 1000 * 15) {
        last_debug_time = now;
        char buf[256];
        sprintf(buf, "reports %lu\r\n", report_count); // 64 or 65; about 4266 samples from the controller per second
        uart_puts(SERIAL_DEBUG_UART, buf);
        loop_count = report_count = 0;
    }
#endif
}

void interval_override_updated() {
}

void flash_b_side() {
}

void queue_out_report(uint16_t interface, uint8_t report_id, const uint8_t* buffer, uint8_t len) {
}

void queue_set_feature_report(uint16_t interface, uint8_t report_id, const uint8_t* buffer, uint8_t len) {
}

void queue_get_feature_report(uint16_t interface, uint8_t report_id, uint8_t len) {
}

void send_out_report() {
}

void sof_callback() {
    set_tick_pending();
}

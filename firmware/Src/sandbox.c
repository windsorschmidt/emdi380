#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "cmsis_os.h"
#include "gfx.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

// Support OpenOCD's thread awareness
const int __attribute__((used)) uxTopUsedPriority = configMAX_PRIORITIES - 1;

static const uint8_t encoder_mapping[] = {11, 14, 10, 15, 6, 3, 7, 2,
                                          12, 13, 9, 16, 5, 4, 8, 1};

// TODO improve this a lot
void delay_us(uint32_t n) {
    volatile uint32_t q;
    for(uint32_t i=0; i<n; i++) {
        for(q=0; q<16; q++);
    }
}

static void reset_lcd() {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    delay_us(40);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    delay_us(40);
}

static void set_lcd_data_to_inputs() {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
}

static void set_lcd_data_to_outputs() {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
}

static void set_keyboard_to_output() {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void set_keyboard_to_floating() {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void pause() {
    static volatile int i;
    for(i = 0; i < 15; i++);
}

// LCD Pin mapping:
// D0 = PD14
// D1 = PD15
// D2 = PD0
// D3 = PD1
// D4 = PE7
// D5 = PE8
// D6 = PE9
// D7 = PE10

static uint8_t get_lcd_data() {
    uint16_t gpiod;
    uint16_t gpioe;
    gpiod = GPIOD->IDR;
    gpioe = GPIOE->IDR;
    return ((gpiod & (GPIO_PIN_14 | GPIO_PIN_15)) >> 14) |
        ((gpiod & (GPIO_PIN_0 | GPIO_PIN_1)) << 2) |
           ((gpioe & (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) >> 3);
}

static void put_lcd_data(uint8_t data) {
    HAL_GPIO_WritePin(GPIOD, ((data & 0x03) << 14) | ((data & 0x0C) >> 2), GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, (((~data) & 0x03) << 14) | (((~data) & 0x0C) >> 2), GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, ((data & 0xF0) << 3), GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, (((~data) & 0xF0) << 3), GPIO_PIN_RESET);
}

void lcd_backlight_on() { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); }
void lcd_backlight_off() { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); }

void lcd_assert_cs() {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

void lcd_deassert_cs() {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
}

void lcd_write(uint8_t byte) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
    put_lcd_data(byte);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
}

void lcd_write16(uint16_t data) {
    lcd_write(data >> 8);
    lcd_write(data & 0xFF);
}

void lcd_write_command(uint8_t byte) {
    GPIOD->BSRR = GPIO_PIN_5 << 16; // ~WR goes low
    GPIOD->BSRR = GPIO_PIN_12 << 16; // D/~C goes C
    put_lcd_data(byte);
    //pause();
    GPIOD->BSRR = GPIO_PIN_5; // ~WR goes high
    GPIOD->BSRR = GPIO_PIN_12; // D/~C goes D
    //pause();
}

uint8_t lcd_read() {
    uint8_t data;
    set_lcd_data_to_inputs();
    GPIOD->BSRR = GPIO_PIN_4 << 16; // ~RD goes low
    pause();
    data = get_lcd_data();
    GPIOD->BSRR = GPIO_PIN_4; // ~RD goes high
    pause();
    set_lcd_data_to_outputs();
    return data;
}

uint32_t read_keyboard() {
    uint32_t keyboard = 0;
    set_keyboard_to_output();
    set_lcd_data_to_inputs();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // KS1
    pause();
    keyboard |= get_lcd_data();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // KS1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // KS2
    pause();
    keyboard |= (get_lcd_data() << 8);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // KS2
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); // KS3
    pause();
    keyboard |= ((uint32_t)get_lcd_data() << 16);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // KS3
    set_keyboard_to_floating();
    set_lcd_data_to_outputs();
    return keyboard;
}

void green_led_on() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
}

void green_led_off() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
}

void red_led_on() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
}

void red_led_off() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
}

uint8_t get_ptt() {
    return !(GPIOE->IDR & GPIO_PIN_11);
}

uint8_t get_encoder() {
    uint16_t gpioe = GPIOE->IDR;
    uint16_t gpiob = GPIOB->IDR;
    return encoder_mapping[((gpioe & (GPIO_PIN_14 | GPIO_PIN_15)) >> 14) |
                           ((gpiob & (GPIO_PIN_10 | GPIO_PIN_11)) >> 8)];
}

static void set_addresses(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    lcd_write_command(0x2A); // CASET
    lcd_write16(x);
    lcd_write16(x + w - 1);
    lcd_write_command(0x2B); // PASET
    lcd_write16(y);
    lcd_write16(y + h - 1);
    lcd_write_command(0x2C); // RAMWR
}

void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    lcd_assert_cs();
    set_addresses(x, y, w, h);
    for(uint16_t i=0; i<h; i++) {
        for(uint16_t j=0; j<w; j++) {
            lcd_write16(color);
        }
    }
    lcd_deassert_cs();
}

void lcd_on() {
    lcd_assert_cs();
    lcd_write_command(0x11); // Sleep out
    //delay_us(120000);
    lcd_write_command(0x3A); // PIXFMT
    lcd_write(0x55); // 16-bit color 565RGB
    lcd_write_command(0x36); // MAC
    lcd_write(0x70); // Sets proper orientation
    lcd_write_command(0x29); // Disp on
    lcd_deassert_cs();
    lcd_fill(0, 0, 160, 128, 0x0000);
}

void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t * data) {
    lcd_assert_cs();
    set_addresses(x, y, w, h);
    for(uint16_t i=0; i<h; i++) {
        for(uint16_t j=0; j<w; j++) {
            lcd_write16(*data++);
        }
    }
    lcd_deassert_cs();
}

void init() {
    green_led_off();
    red_led_off();
    set_lcd_data_to_outputs();
    set_keyboard_to_floating();
    reset_lcd();
    lcd_backlight_off();

    // LCD control lines
    // ~CS  = PD6
    // D/~C = PD12
    // ~RD  = PD4
    // ~WR  = PD5
    // ~RST = PD13
}

// FreeRTOS idle task, defined __weak in auto-generated main.c
void DefaultTask(void const * argument) {
    (void) argument;

    uint8_t buf[] = "test";

    gfxInit(); // after FreeRTOS starts we can initialize uGFX
    init();
    MX_USB_DEVICE_Init();
    osDelay(1000);
    while(1) {
        CDC_Transmit_FS(buf, 4);
        GPIOE->BSRR = (1<<0); // green LED ON
        osDelay(500);
        GPIOE->BSRR = (1<<16); // green LED off
        osDelay(500);
        lcd_fill(10, 10, 40, 40, 0);
    }
}

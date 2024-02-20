/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * This example uses OTAA to join the LoRaWAN network and then sends the
 * internal temperature sensors value up as an uplink message periodically
 * and the first byte of any uplink messages received controls the boards
 * built-in LED.
 */

#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "pico/stdlib.h"
#include "pico/lorawan.h"
// #include "tusb.h"

// edit with LoRaWAN Node Region and OTAA settings
#include "config.h"

// pin configuration for SX12xx radio module
const struct lorawan_sx12xx_settings sx12xx_settings = {
    .spi = {
        .inst = spi1,
        .mosi = 11,
        .miso = 12,
        .sck = 10,
        .nss = 3},
    .reset = 15,
    .busy = 2,
    .dio1 = 20};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui = LORAWAN_DEVICE_EUI,
    .app_eui = LORAWAN_APP_EUI,
    .app_key = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK};

// variables for receiving data
int LORA_RX_LEN = 0;
uint8_t LORA_RX_BUFFER[242];
uint8_t LORA_RX_PORT = 0;
#define LORA_RX_TIMEOUT_MS 10000 // Default 30000

#include "hardware/uart.h"
#define UART_ID uart0
#define UART_BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_MAX_BUFFER 47

void _init_LED();
void _set_led_time(bool led_on, uint8_t time_sec);

void _internal_temperature_init();
float _internal_temperature_get();

void _uart_init();
uint8_t _uart_get(char data[]);
void _uart_send(const void *data, uint8_t data_len);

void _lorawan_init();
void _lorawan_join();
void _lorawan_send(const void *data, uint8_t data_len);

void _inf_send_temperature_data_via_uart();
void _inf_send_temperature_data_via_lorawan();
void _inf_send_uart_data_via_uart();
void _inf_send_uart_data_via_lorawan();
#define _INF_SLEEP_MS 1000

void _print_data(const void *data, uint8_t data_len);

int main(void)
{
    // initialize stdio
    stdio_init_all();

    // _inf_send_temperature_data_via_uart();
    _inf_send_uart_data_via_lorawan();

    return 0;
}

// Continuously send UART data via LoRaWAN
void _inf_send_uart_data_via_lorawan()
{
    _uart_init();
    _lorawan_init();
    _lorawan_join();
    char data[UART_MAX_BUFFER];
    uint8_t data_len;
    while (1)
    {
        data_len = _uart_get(data);
        printf("%s", data);
        _lorawan_send(data, data_len);
        sleep_ms(_INF_SLEEP_MS);
    }
}

// Continuously send UART data via UART
void _inf_send_uart_data_via_uart()
{
    _uart_init();
    char data[UART_MAX_BUFFER];
    uint8_t data_len;
    while (1)
    {
        data_len = _uart_get(data);
        _uart_send(data, data_len);
        sleep_ms(_INF_SLEEP_MS);
    }
}

// Continuously Send internal temperature data via LoRaWAN
void _inf_send_temperature_data_via_lorawan(){
    _internal_temperature_init();
    _uart_init();
    _lorawan_init();
    _lorawan_join();
    printf("Sending Internal Temperature: \n");
    while(1){
        int8_t temperature_byte = _internal_temperature_get();
        printf("%d'C (0x%02x) \n", temperature_byte, temperature_byte);
        _lorawan_send(&temperature_byte, sizeof(temperature_byte));
        sleep_ms(_INF_SLEEP_MS);
    }
}

// Continuously Send internal temperature data via UART
void _inf_send_temperature_data_via_uart(){
    _internal_temperature_init();
    _uart_init();
    printf("Sending Internal Temperature: \n");
    while(1){
        int8_t temperature_byte = _internal_temperature_get();
        printf("%d 'C (0x%02x)", temperature_byte, temperature_byte);
        _uart_send(&temperature_byte, sizeof(temperature_byte));
        sleep_ms(_INF_SLEEP_MS);
    }
}

// Initialize LED
void _init_LED(){
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

// Set LED output for time_sec seconds
void _set_led_time(bool led_on, uint8_t time_sec)
{
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    sleep_ms(time_sec*1000);
}

// Initialize UART
void _uart_init()
{
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Get UART data
uint8_t _uart_get(char data[])
{
    uint8_t data_len = 0;
    while (1)
    {
        if (uart_is_readable(uart0) && data_len < UART_MAX_BUFFER-1)
        {
            data[data_len++] = uart_getc(uart0);
        }else{
            break;
        }
    }
    data[data_len] = '\0';
    return data_len;
}

// Send data via UART
void _uart_send(const void *data, uint8_t data_len)
{
    if (data_len > 0)
    {
        printf("Sending data via UART ... ");
        uart_puts(UART_ID, data);
        printf("success!\n");
    }
    else
    {
        printf("No UART data available\n");
    }
}

// Initialze internal temperature to read via ADC
void _internal_temperature_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

// Get pico internal temperature
float _internal_temperature_get()
{
    const float v_ref = 3.3;

    // read the ADC
    uint16_t adc_raw = adc_read();

    // convert the raw ADC value to a voltage
    float adc_voltage = adc_raw * v_ref / 4095.0f;

    // convert voltage to temperature
    float adc_temperature = 27.0 - ((adc_voltage - 0.706) / 0.001721);

    return adc_temperature;
}

// Initialize the LoRaWAN stack
void _lorawan_init()
{
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_otaa(&sx12xx_settings, LORAWAN_REGION, &otaa_settings) < 0)
    {
        printf("failed!!!\n");
        while (1)
        {
            tight_loop_contents();
        }
    }
    else
    {
        printf("success!\n");
    }
}

// Initiate LoRaWAN join process
void _lorawan_join()
{
    printf("Joining LoRaWAN network ... ");
    lorawan_join();

    while (!lorawan_is_joined())
    {
        lorawan_process_timeout_ms(1000);
        printf(".");
    }
    printf("success!\n");
}

// Send data via LoRaWAN
void _lorawan_send(const void *data, uint8_t data_len)
{
    if(data_len > 0)
    {
        printf("Sending data via LoRaWAN ... ");
        if (lorawan_send_unconfirmed(data, data_len, 2) < 0)
        {
            printf("failed!!!\n");
        }
        else
        {
            printf("success!\n");
        }

        // wait for up to LORA_RX_TIMEOUT_MS milliseconds for an event
        if (lorawan_process_timeout_ms(LORA_RX_TIMEOUT_MS) == 0)
        {
            // check if a downlink message was received
            LORA_RX_LEN = lorawan_receive(LORA_RX_BUFFER, sizeof(LORA_RX_BUFFER), &LORA_RX_PORT);
            if (LORA_RX_LEN > -1)
            {
                printf("received a %d byte message on port %d: ", LORA_RX_LEN, LORA_RX_PORT);

                for (int i = 0; i < LORA_RX_LEN; i++)
                {
                    printf("%02x", LORA_RX_BUFFER[i]);
                }
                printf("\n");

                // the first byte of the received message controls the on board LED
                gpio_put(PICO_DEFAULT_LED_PIN, LORA_RX_BUFFER[0]);
            }
        }
    }
    else
    {
        printf("No UART data available\n");
    }
}

// Print data
void _print_data(const void *data, uint8_t data_len)
{
    uint8_t *c = (uint8_t *)data;
    printf("Printing data: \n");
    for (uint8_t i = 0; i < data_len; i++)
    {
        printf("0x%02x ", c[i]);
    }
    printf("\n");
}

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

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "pico/multicore.h"
#include "pico/util/queue.h"

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

void _adc_setup();
void _adc_irh();
void _adc_dma1_setup();
void _adc_dma2_setup();
void _adc_dma_irh();

void _trig_dma_setup();
void _trig_dma_irh();

void core1();

void test();

// maximum frequency of the input signal
#define fm 100
// sampling factor n implies 2^n sampling rate
#define SAMPLING_FACTOR 5
// sampling rate needed
#define fs (fm << SAMPLING_FACTOR)
// ADC clock time
#define CLOCK_DIV (48000000 / fs)
// adc buffer sample size
#define ADC_SAMPLES_N 8192
uint8_t ADC_BUFFER[2][ADC_SAMPLES_N];
// uint8_t ADC_NEXT_BUFFER_INDEX = 1;
// uint8_t (*ADC_NEXT_BUFFER_PTR)[] = ADC_BUFFER[1];
uint adc_dma1_chn, trig_dma_chn, adc_dma2_chn;
dma_channel_config adc_dma1_cfg, trig_dma_cfg, adc_dma2_cfg;
uint8_t tmp_buffer;

void _adc_setup()
{

    // Initialize PIN 26 for ADC
    // disable digital functions for the pin
    adc_gpio_init(26);

    // Initialize ADC, reset and enable clock (48MHz)
    adc_init();

    // select input for ADC MUX (0...3 are PINs 26...29, 4 is internal temperature)
    adc_select_input(0);

    adc_fifo_setup(
        true,  // write each completed conversion to FIFO
        true,  // enable DMA Data Request (DREQ)
        1,     // number of samples OR DREQ_THRESHOLD
        false, // error bit
        true   // shift each sample to 8 bits
    );

    adc_fifo_drain();

    // adc_irq_set_enabled(true);
    // irq_set_exclusive_handler(ADC_IRQ_FIFO, _adc_irh);
    // irq_set_enabled(ADC_IRQ_FIFO, true);

    // ADC is triggered every CLOCK_DIV+1 cycles, it takes 96 cycles for 1 conversion
    // Cycles are wrt to 48MHz ADC clock
    // CLOCK_DIV set to 0 implies free running mode
    // set any value > 96 for slowing down the ADC sampling rate
    adc_set_clkdiv(CLOCK_DIV);

    return;
}

// uint adc_sum = 0;

uint adc_cnt = 0;

void _adc_irh()
{
    // uint a = adc_fifo_get();
    // adc_sum += a;
    adc_cnt++;
    // printf("%d,",a);
    return;
}

void _adc_dma1_setup()
{

    // ADC DMA config
    adc_dma1_cfg = dma_channel_get_default_config(adc_dma1_chn);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&adc_dma1_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&adc_dma1_cfg, false);
    channel_config_set_write_increment(&adc_dma1_cfg, false);
    channel_config_set_chain_to(&adc_dma1_cfg, adc_dma2_chn);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&adc_dma1_cfg, DREQ_ADC);

    dma_channel_configure(
        adc_dma1_chn,   // DMA channel
        &adc_dma1_cfg,  // DMA configuration
        &tmp_buffer,    // initial write address
        &adc_hw->fifo,  // initial read address
        1,              // transfer count
        false           // start immediate trigger
    );

    dma_channel_set_irq0_enabled(adc_dma1_chn, true);

    return;
}

void _adc_dma2_setup()
{

    // ADC DMA config
    adc_dma2_cfg = dma_channel_get_default_config(adc_dma2_chn);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&adc_dma2_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&adc_dma2_cfg, false);
    channel_config_set_write_increment(&adc_dma2_cfg, false);
    channel_config_set_chain_to(&adc_dma2_cfg, adc_dma1_chn);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&adc_dma2_cfg, DREQ_ADC);

    dma_channel_configure(
        adc_dma2_chn,   // DMA channel
        &adc_dma2_cfg,  // DMA configuration
        &tmp_buffer,    // initial write address
        &adc_hw->fifo,  // initial read address
        1,              // transfer count
        false           // start immediate trigger
    );

    dma_channel_set_irq0_enabled(adc_dma2_chn, true);

    return;
}

void _init_setup()
{

    // ADC SETUP
    _adc_setup();
    sleep_ms(1000);

    // DMA SETUP
    adc_dma1_chn = dma_claim_unused_channel(true);
    // trig_dma_chn = dma_claim_unused_channel(true);
    adc_dma2_chn = dma_claim_unused_channel(true);

    _adc_dma1_setup();
    // _trig_dma_setup();
    _adc_dma2_setup();

    irq_set_exclusive_handler(DMA_IRQ_0, _adc_dma_irh);
    irq_set_enabled(DMA_IRQ_0, true);
}



// void _trig_dma_setup()
// {

//     // TRIG DMA config
//     trig_dma_cfg = dma_channel_get_default_config(trig_dma_chn);

//     // Reading from constant address, writing to incrementing byte addresses
//     channel_config_set_transfer_data_size(&trig_dma_cfg, DMA_SIZE_32);
//     channel_config_set_read_increment(&trig_dma_cfg, false);
//     channel_config_set_write_increment(&trig_dma_cfg, false);
//     channel_config_set_chain_to(&trig_dma_cfg, adc_dma2_chn);

//     dma_channel_configure(
//         trig_dma_chn,                        // DMA channel
//         &trig_dma_cfg,                       // DMA configuration
//         &dma_hw->ch[adc_dma2_chn].write_addr, // initial write address
//         &ADC_NEXT_BUFFER_PTR,                // initial read address
//         1,                                   // transfer count
//         false                                // start immediate trigger
//     );

//     dma_channel_set_irq0_enabled(trig_dma_chn, true);
//     // irq_set_exclusive_handler(DMA_IRQ_0, _trig_dma_irh);
//     // irq_set_enabled(DMA_IRQ_0, true);

//     return;
// }

volatile bool trig = true;
uint dma_txed = 0;

// void _trig_dma_irh()
// {
//     // channel_config_set_enable(&trig_dma_cfg, false);
//     adc_run(false);
//     // channel_config_set_chain_to(&trig_dma_cfg, trig_dma_chn);
//     dma_hw->ints0 = (1u << trig_dma_chn);
//     // printf("c=%d,s=%d\n", adc_cnt, adc_sum);
//     // adc_cnt=0; adc_sum = 0;
//     // printf("adc_cnt=%d\n", adc_cnt);
//     // adc_cnt=0;
//     ADC_NEXT_BUFFER_INDEX ^= 1;
//     ADC_NEXT_BUFFER_PTR = ADC_BUFFER[ADC_NEXT_BUFFER_INDEX];
//     dma_txed++;
//     trig = !trig;
//     // channel_config_set_enable(&trig_dma_cfg, false);

//     // printf("I: %d\n", ADC_NEXT_BUFFER_INDEX);
// }

#define window ( 1 << SAMPLING_FACTOR ) //  = fs/fm
#define qlen ( window >> 1 )  // = (fs/fm)/2
uint8_t q[qlen];
#define threshold 120
uint8_t peakv = 0;
unsigned int qi = 0;
int peaki = -1;
bool peakb = false;

bool checkPeak(uint8_t v)
{
    peakb = false;

    // if current exiting is peak
    if (peaki == qi)
    {
        peakb = true;
        peakv = q[qi];
        peaki = -1;
    }

    q[qi] = v;

    // // if no peak and v more than threshold
    // if(peaki == -1 && v > threshold){
    //     peaki = qi;
    // }

    // // if peak exists and new peak has come
    // else if(peaki != -1 && v > q[peaki]){
    //     peaki = qi;
    // }

    if (peaki == -1 ? v > threshold : v > q[peaki])
    {
        peaki = qi;
    }

    qi = (qi + 1) & (qlen - 1);

    return peakb;
}

uint trig_cnt = 0;
uint buffer_i = 0;

queue_t queue;
const uint8_t zero = 0;

void _adc_dma_irh()
{
    dma_channel_acknowledge_irq0(adc_dma1_chn);
    dma_channel_acknowledge_irq0(adc_dma2_chn);
    // if (buffer_i < ADC_SAMPLES_N)
    {
        // ADC_BUFFER[0][buffer_i] = tmp_buffer;
        if (checkPeak(tmp_buffer))
        {
            // ADC_BUFFER[1][buffer_i] = peakv;
            queue_try_add(&queue, &peakv);
        }
        else
        {
            // ADC_BUFFER[1][buffer_i] = 100;
            queue_try_add(&queue, &zero);
        }
        // buffer_i++;
    }
    // dma_txed++;
    // trig = !trig;
}

void core1()
{

    printf("core 1 started \n");

    printf("setting up adc dma\n");
    // _adc_dma1_setup();
    _init_setup();
    printf("adc dma setup done\n");

    printf("starting adc & dma\n");
    adc_run(true);
    dma_channel_start(adc_dma1_chn);
    printf("adc & dma started\n");

    unsigned long long loop_count = 0;

    while (true)
    {
        // if(dma_txed){
        //     int id = ADC_NEXT_BUFFER_INDEX;
        //     printf("BD %d:\n", id);
        //     uint sum = 0;
        //     for(int i=0; i<ADC_SAMPLES_N; i++){
        //         sum += ADC_BUFFER[id][i];
        //         // printf("%d,",ADC_BUFFER[id][i]);
        //     }
        //     printf("sum: %d\n",sum);
        //     dma_txed = false;
        // }
        // if(dma_txed>=ADC_SAMPLES_N){
        //     printf("\nADC BUFFER 0\n");
        //     for(int i=0; i<ADC_SAMPLES_N; i++){
        //         printf("%d,",ADC_BUFFER[0][i]);
        //     }
        //     // printf("\nADC BUFFER 1\n");
        //     // for(int i=0; i<ADC_SAMPLES_N; i++){
        //     //     printf("%d,",ADC_BUFFER[1][i]);
        //     // }
        //     printf("\n");
        // }
        // while(trig){
        //     loop_count++;
        // }
        // printf("lc0:%llu\n",loop_count);
        // loop_count = 0;

        // while(!trig){
        //     loop_count++;
        // }
        // printf("lc1:%llu\n",loop_count);
        // loop_count = 0;
        tight_loop_contents();
    }
}

bool _send_data(char* data, uint8_t data_len){

    data[data_len] = '\0';
    // uart_puts(UART_ID, data);
    // _uart_send(data, data_len);
    _lorawan_send(data, data_len);

}

int main(void)
{
    // initialize stdio
    stdio_init_all();
    _uart_init();
    _lorawan_init();
    _lorawan_join();

    for (int i = 0; i < ADC_SAMPLES_N; i++)
    {
        ADC_BUFFER[0][i] = 0;
        ADC_BUFFER[1][i] = 0;
    }

    queue_init(&queue, 1, window);

    sleep_ms(5000);
    printf("core 1 launching in ");
    for (int i = 5; i > 0; i--)
    {
        printf("%d ", i);
        sleep_ms(1000);
    }
    printf("\n");
    multicore_launch_core1(core1);
    // _inf_send_temperature_data_via_uart();
    // _inf_send_uart_data_via_lorawan();

    // store queue value in v
    uint8_t v;
    int buff_i = 0;
    // zeros count in queue
    uint8_t zs = 0;
    // zeros count threshold
    const uint8_t zth = window * 1.5;
    const uint8_t ldlen_max = 32;
    uint8_t ldata[ldlen_max+1];
    uint8_t ldlen = 0;
    // count of zero peaks
    uint8_t czs = 0;
    // zero peaks count threshold
    const uint8_t czsth = ldlen_max / 3;
    while (true)
    {
        // if (buff_i >= ADC_SAMPLES_N)
        // {
        //     printf("\nADC SAMPLES 0\n");
        //     for (int i = 0; i < ADC_SAMPLES_N; i++)
        //     {
        //         printf("%d,", ADC_BUFFER[0][i]);
        //     }
        //     printf("\nADC SAMPLES 1\n");
        //     for (int i = 0; i < ADC_SAMPLES_N; i++)
        //     {
        //         printf("%d,", ADC_BUFFER[1][i]);
        //     }
        //     while (true)
        //     {
        //         tight_loop_contents();
        //     }
        // }
        if (queue_try_remove(&queue, &v))
        {
            // ADC_BUFFER[1][buff_i++] = v;
            if (v != 0)
            {
                ldata[ldlen++] = v;
                zs = 0;
                czs = 0;
                if(ldlen == ldlen_max){
                    _send_data(ldata, ldlen);
                    ldlen = 0;
                }
            }
            else
            {
                if (ldlen > 0)
                {
                    zs++;
                    if (zs >= zth)
                    {
                        ldata[ldlen++] = 0;
                        zs -= window;
                        czs++;
                        if (czs == czsth || ldlen == ldlen_max)
                        {
                            _send_data(ldata, ldlen-czs);
                            ldlen = 0;
                            zs = 0;
                            czs = 0;
                        }
                    }
                }
            }
        }
    }

    return 0;
}

void test()
{
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
void _inf_send_temperature_data_via_lorawan()
{
    _internal_temperature_init();
    _uart_init();
    _lorawan_init();
    _lorawan_join();
    printf("Sending Internal Temperature: \n");
    while (1)
    {
        int8_t temperature_byte = _internal_temperature_get();
        printf("%d'C (0x%02x) \n", temperature_byte, temperature_byte);
        _lorawan_send(&temperature_byte, sizeof(temperature_byte));
        sleep_ms(_INF_SLEEP_MS);
    }
}

// Continuously Send internal temperature data via UART
void _inf_send_temperature_data_via_uart()
{
    _internal_temperature_init();
    _uart_init();
    printf("Sending Internal Temperature: \n");
    while (1)
    {
        int8_t temperature_byte = _internal_temperature_get();
        printf("%d 'C (0x%02x)", temperature_byte, temperature_byte);
        _uart_send(&temperature_byte, sizeof(temperature_byte));
        sleep_ms(_INF_SLEEP_MS);
    }
}

// Initialize LED
void _init_LED()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

// Set LED output for time_sec seconds
void _set_led_time(bool led_on, uint8_t time_sec)
{
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    sleep_ms(time_sec * 1000);
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
        if (uart_is_readable(uart0) && data_len < UART_MAX_BUFFER - 1)
        {
            data[data_len++] = uart_getc(uart0);
        }
        else
        {
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
    if (data_len > 0)
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

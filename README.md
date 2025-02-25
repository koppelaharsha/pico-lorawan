# pico-lorawan
Enable LoRaWAN communications on [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/?variant=raspberry-pi-pico-h) using [Semtech SX1262 radio module](https://www.waveshare.com/wiki/Pico-LoRa-SX1262).

Based on the Semtech's [LoRaWAN end-device stack implementation and example projects](https://github.com/Lora-net/LoRaMac-node).

## Hardware

 * [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/?variant=raspberry-pi-pico-h)
 * [WaveShare SX1262 868M](https://www.waveshare.com/wiki/Pico-LoRa-SX1262)
 * [USB-UART](https://www.waveshare.com/wiki/PL2303_USB_UART_Board_(type_A)_V2)


### Default Pinout

| Raspberry Pi Pico | WaveShare SX1262 |
| ----------------- | -------------- |
| VSYS | VCC |
| GND | GND |
| GPIO 10 | SCK |
| GPIO 11 | MOSI |
| GPIO 12 | MISO |
| GPIO 15 | RESET |
| GPIO 3 | NSS / CS |
| GPIO 2 | BUSY |
| GPIO 20 | DIO1 |

| Raspberry Pi Pico | USB UART |
| ----------------- | -------------- |
| VSYS (PIN 39) | VCC |
| GND (PIN 38) | GND |
| GPIO 0 | RXD |
| GPIO 1 | TXD |

GPIO pins are configurable in examples or API.
USB-UART jumper set to 5V.

## Examples

See [examples](examples/) folder.

There is a `config.h` file to your ABP or OTAA node configuration for each example.

## Cloning

```sh
git clone --recurse-submodules https://github.com/koppelaharsha/pico-lorawan.git 
```

## Building

1. [Set up the Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
- For windows, installing via setup file is recommended
2. Setup SDK
- For windows
    - Open `Pico Developer Command Prompt` to set the path
    - Open vscode from the same terminal
3. Build the project
4. Copy example `.uf2` to Pico when in BOOT mode.

## Acknowledgements

Thanks to [siuwah](https://github.com/siuwahzhong) for contributing to [the original repository](https://github.com/ArmDeveloperEcosystem/lorawan-library-for-pico) to make it work for SX1262 module.


# FreeRTOS ADC DMA UART Logger

> Timer-triggered ADC sampling on an STM32G431, with circular DMA, FreeRTOS task notifications and queues, and UART packet output for a Python host decoder.

## Overview

This project acquires analog samples on an STM32G431 (NUCLEO-G431RB target). TIM2 generates a 1 kHz trigger for ADC1, and DMA transfers 16-bit results into an eight-element circular buffer. FreeRTOS separates sample handling from serial output: the DMA interrupt notifies an acquisition task, which copies each completed half-buffer into a queue; a UART task removes packets from that queue and transmits them.

Each packet contains four consecutive readings from the single analog input on PA0. The current firmware does not scan four independent ADC channels. The firmware has been build-verified; hardware testing is pending.

## Data flow

```text
PA0 analog input
      |
      v
TIM2 TRGO at 1 kHz --> ADC1 channel 1 --> DMA1 Channel 1 circular buffer [8]
                                                  |
                                      half/full transfer interrupt
                                                  |
                                    FreeRTOS task notification
                                                  |
                                  Acquisition task (priority 3)
                                  copies four readings to queue
                                                  |
                                     Packet queue (8 packets)
                                                  |
                                           UART task (priority 2)
                                                  |
                                            USART1 TX, PA9
                                                  |
                                             Host decode.py
```

### Firmware sequence

1. TIM2 uses `PSC=15` and `ARR=999` with the 16 MHz HSI clock, producing a 1 kHz update/TRGO.
2. ADC1 converts PA0 (ADC channel 1) on each TIM2 trigger. ADC12 uses SYSCLK as its kernel clock.
3. DMA1 Channel 1 writes each result into an eight-element, 16-bit circular buffer.
4. On half-transfer and transfer-complete events, the DMA ISR clears the flag and notifies the acquisition task. It does not format or transmit data inside the ISR.
5. The acquisition task copies the ready four-sample region into a packet and sends it to a FreeRTOS queue. If the queue is full, the packet is dropped and a counter is incremented.
6. The UART task is the only task that writes packet bytes to USART1.

The two DMA notifications arrive alternately every four samples, or every 4 ms at 1 kHz. This produces up to 250 packets per second.

## Packet format

Each packet is 10 bytes:

| Byte(s) | Value |
|---|---|
| 0 | Sync byte `0x23` (`#`) |
| 1–8 | Four unsigned 16-bit samples, little-endian |
| 9 | XOR of payload bytes 1–8 |

The decoder prints these values as `S0` through `S3`; they are four successive samples from PA0, not four separate channels.

## Hardware

| Pin/peripheral | Use |
|---|---|
| PA0 | ADC1 channel 1 analog input, 0–3.3 V |
| PA9 | USART1 TX; connect to USB-UART adapter RX |
| PA10 | USART1 RX pin configured by firmware; unused by this logger |
| GND | Common ground between the board and USB-UART adapter |
| TIM2 | 1 kHz update event routed to ADC1 as TRGO |
| DMA1 Channel 1 | ADC1 circular transfers, half/full interrupts |
| USART1 | Nominal 115200 baud, transmit-only application path |

**Nucleo VCP note:** The project’s `.ioc` file maps the onboard ST-LINK virtual COM port to LPUART1 on PA2/PA3, but this firmware outputs through USART1 on PA9. For the current firmware, connect an external USB-UART adapter to PA9 and GND. Use a 3.3 V-compatible adapter.

## RTOS configuration

- FreeRTOS kernel with the GCC ARM Cortex-M4F port.
- 1 kHz RTOS tick.
- Acquisition task: priority 3; waits on DMA task notifications.
- UART task: priority 2; waits on a queue of eight packets.
- DMA interrupt priority is set to the FreeRTOS syscall-safe priority.
- FreeRTOS heap: 8 KiB using `heap_4.c`.

## Build and flash

The project uses the included Makefile and requires `arm-none-eabi-gcc`, GNU Make, and OpenOCD (for flashing).

```sh
make
```

The build outputs are `build/Testtt.elf`, `build/Testtt.hex`, and `build/Testtt.bin`. To flash with OpenOCD and an ST-Link:

```sh
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program build/Testtt.elf verify reset exit"
```

If the ARM GCC tools are not on `PATH`, pass their `bin` directory through the Makefile’s `GCC_PATH` variable:

```sh
make GCC_PATH="/path/to/arm-none-eabi/bin"
```

## Host decoder

Install the Python serial dependency and run the included decoder:

```sh
python -m pip install pyserial
python decode.py
```

Edit the `COM4` port in `decode.py` to match the serial adapter on your computer. Use 115200 baud. The decoder searches for the sync byte, reads the fixed-size packet, checks the XOR checksum, and prints valid sample groups.

Example:

```text
Listening on COM4 at 115200 baud...

S0:  2048  S1:  2051  S2:  2047  S3:  2050
```

## Project structure

```text
Testtt/
├── Core/
│   ├── Inc/
│   │   └── FreeRTOSConfig.h
│   └── Src/
│       ├── main.c                 # ADC/DMA setup and FreeRTOS tasks
│       └── stm32g4xx_it.c         # DMA and RTOS exception handlers
├── Drivers/                       # STM32 CMSIS and HAL device support
├── Middlewares/Third_Party/FreeRTOS/Source/
│   ├── include/
│   └── portable/GCC/ARM_CM4F/     # FreeRTOS Cortex-M4F port
├── decode.py                      # Host-side serial packet decoder
├── Makefile
├── STM32G431XX_FLASH.ld
└── startup_stm32g431xx.s
```

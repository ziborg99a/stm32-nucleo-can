# STM32 Nucleo CAN Bus Project

This is a minimal C++ starter project for a CAN bus application on an STM32 Nucleo board using PlatformIO and the STM32Cube HAL.

## Selected board
- NUCLEO-F446RE

## Features
- HAL-based CAN1 configuration
- CAN transmit example
- CAN receive monitor over the Nucleo virtual COM port
- Ready for extension to a real bus application

## Requirements
- PlatformIO Core or VS Code PlatformIO extension
- STM32 Nucleo board
- ST-LINK programmer/debugger

## Project structure
```text
stm32-nucleo-can/
├── platformio.ini
├── README.md
└── src/
    ├── main.cpp
    ├── canbus.hpp
    └── canbus.cpp
```

## Build
```bash
cd stm32-nucleo-can
pio run
```

## Run tests
The host-native Unity tests cover hardware-independent CAN frame validation:
```bash
cd stm32-nucleo-can
/home/x/.platformio/penv/bin/pio test -e native
```

## Upload
```bash
cd stm32-nucleo-can
pio run --target upload
```

## Monitor serial output
```bash
cd stm32-nucleo-can
pio device monitor
```

## Notes
- The board uses `CAN1` on pins `PA11` and `PA12`.
- CAN frames received from FIFO 0 are printed on `USART2` (`PA2`/`PA3`) at 115200 baud.
- Connect the CAN transceiver to the bus using the proper terminators and wiring.
- This project is intentionally simple and meant as a starting point for real CAN applications.

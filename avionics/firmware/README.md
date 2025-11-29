# SRAD Flight Computer Firmware

## Hardware Configuration

| Component | Model | Interface | Pins |
|-----------|-------|-----------|------|
| MCU | STM32F429ZIT6 | - | - |
| Barometer | MS5611 | I2C / SPI | Configurable |
| IMU | MPU-9250 | I2C / SPI | Configurable |
| GPS | NEO-7M | UART | TX/RX |
| LoRa | E32-433T30D | UART | TX/RX/M0/M1/AUX |
| Flash | W25Q40 | SPI | CS/CLK/MOSI/MISO |

## Firmware Structure

```
firmware/
├── Core/
│   ├── Inc/
│   │   └── main.h
│   └── Src/
│       └── main.c
├── Drivers/
│   ├── MS5611/          # Barometer driver
│   ├── MPU9250/         # IMU driver
│   ├── NEO7M/           # GPS driver
│   ├── E32_LoRa/        # LoRa driver
│   └── W25Qxx/          # Flash driver
└── README.md
```

## Build Instructions

1. Open project in STM32CubeIDE
2. Configure pins in .ioc file
3. Build and flash via ST-Link

## Pin Configuration (Example)

Configure these in STM32CubeMX:

### I2C1 (Sensors)
- PB6: I2C1_SCL
- PB7: I2C1_SDA

### UART1 (GPS)
- PA9: USART1_TX
- PA10: USART1_RX

### UART2 (LoRa)
- PA2: USART2_TX
- PA3: USART2_RX

### SPI1 (Flash)
- PA5: SPI1_SCK
- PA6: SPI1_MISO
- PA7: SPI1_MOSI
- PA4: Flash_CS (GPIO)

### Pyro Channels
- PC0: PYRO1 (Drogue)
- PC1: PYRO2 (Main)

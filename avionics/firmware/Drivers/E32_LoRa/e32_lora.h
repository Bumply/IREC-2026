/**
 * @file e32_lora.h
 * @brief EBYTE E32-433T30D LoRa Module Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * E32-433T30D is a 433MHz LoRa wireless module with:
 * - Frequency: 410-441 MHz
 * - Power: 30dBm (1W) max
 * - Range: Up to 8km (LOS)
 * - Interface: UART (TTL)
 * - Baud rates: 1200 to 115200
 * 
 * Pin Configuration:
 * - M0, M1: Mode selection (GPIO outputs)
 * - AUX: Status indicator (GPIO input)
 * - TXD, RXD: UART interface
 * - VCC: 3.3-5.2V
 */

#ifndef E32_LORA_H
#define E32_LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * CONFIGURATION
 *============================================================================*/

/* Maximum packet size */
#define E32_MAX_PACKET_SIZE     58      /* LoRa packet limit */

/* TX/RX buffer sizes */
#define E32_TX_BUFFER_SIZE      256
#define E32_RX_BUFFER_SIZE      256

/* Timeouts */
#define E32_AUX_TIMEOUT_MS      1000
#define E32_CONFIG_TIMEOUT_MS   500

/*============================================================================
 * OPERATING MODES
 *============================================================================*/

typedef enum {
    E32_MODE_NORMAL = 0,        /* M0=0, M1=0: Normal operation (TX/RX) */
    E32_MODE_WAKEUP = 1,        /* M0=1, M1=0: Wake-up mode */
    E32_MODE_POWER_SAVE = 2,    /* M0=0, M1=1: Power saving mode */
    E32_MODE_SLEEP = 3          /* M0=1, M1=1: Sleep/configuration mode */
} E32_Mode_t;

/*============================================================================
 * CONFIGURATION PARAMETERS
 *============================================================================*/

typedef enum {
    E32_UART_PARITY_8N1 = 0b00,     /* 8N1 (default) */
    E32_UART_PARITY_8O1 = 0b01,     /* 8O1 */
    E32_UART_PARITY_8E1 = 0b10      /* 8E1 */
} E32_UartParity_t;

typedef enum {
    E32_UART_BAUD_1200 = 0b000,
    E32_UART_BAUD_2400 = 0b001,
    E32_UART_BAUD_4800 = 0b010,
    E32_UART_BAUD_9600 = 0b011,     /* Default */
    E32_UART_BAUD_19200 = 0b100,
    E32_UART_BAUD_38400 = 0b101,
    E32_UART_BAUD_57600 = 0b110,
    E32_UART_BAUD_115200 = 0b111
} E32_UartBaud_t;

typedef enum {
    E32_AIR_RATE_300 = 0b000,       /* 300 bps */
    E32_AIR_RATE_1200 = 0b001,      /* 1200 bps */
    E32_AIR_RATE_2400 = 0b010,      /* 2400 bps (default) */
    E32_AIR_RATE_4800 = 0b011,      /* 4800 bps */
    E32_AIR_RATE_9600 = 0b100,      /* 9600 bps */
    E32_AIR_RATE_19200 = 0b101      /* 19200 bps */
} E32_AirRate_t;

typedef enum {
    E32_TX_POWER_30DBM = 0b00,      /* 30 dBm (1W) - Default */
    E32_TX_POWER_27DBM = 0b01,      /* 27 dBm (500mW) */
    E32_TX_POWER_24DBM = 0b10,      /* 24 dBm (250mW) */
    E32_TX_POWER_21DBM = 0b11       /* 21 dBm (125mW) */
} E32_TxPower_t;

typedef enum {
    E32_FEC_OFF = 0,                /* FEC disabled */
    E32_FEC_ON = 1                  /* FEC enabled (default) */
} E32_FEC_t;

typedef enum {
    E32_WAKEUP_250MS = 0b000,       /* 250ms */
    E32_WAKEUP_500MS = 0b001,       /* 500ms */
    E32_WAKEUP_750MS = 0b010,       /* 750ms */
    E32_WAKEUP_1000MS = 0b011,      /* 1000ms */
    E32_WAKEUP_1250MS = 0b100,      /* 1250ms */
    E32_WAKEUP_1500MS = 0b101,      /* 1500ms */
    E32_WAKEUP_1750MS = 0b110,      /* 1750ms */
    E32_WAKEUP_2000MS = 0b111       /* 2000ms */
} E32_WakeupTime_t;

typedef enum {
    E32_IO_OPEN_COLLECTOR = 0,      /* TXD/AUX open collector */
    E32_IO_PUSH_PULL = 1            /* TXD/AUX push-pull (default) */
} E32_IO_Mode_t;

typedef enum {
    E32_TX_MODE_TRANSPARENT = 0,    /* Transparent transmission */
    E32_TX_MODE_FIXED = 1           /* Fixed transmission (address mode) */
} E32_TxMode_t;

/*============================================================================
 * STATUS/ERROR CODES
 *============================================================================*/

typedef enum {
    E32_OK = 0,
    E32_ERROR = -1,
    E32_ERROR_TIMEOUT = -2,
    E32_ERROR_BUSY = -3,
    E32_ERROR_PARAM = -4,
    E32_ERROR_NOT_INIT = -5
} E32_Status_t;

/*============================================================================
 * CONFIGURATION STRUCTURE
 *============================================================================*/

typedef struct {
    uint16_t address;               /* Module address (0x0000-0xFFFF) */
    uint8_t channel;                /* Operating channel (0-31) */
    E32_UartParity_t uart_parity;
    E32_UartBaud_t uart_baud;
    E32_AirRate_t air_rate;
    E32_TxPower_t tx_power;
    E32_FEC_t fec;
    E32_WakeupTime_t wakeup_time;
    E32_IO_Mode_t io_mode;
    E32_TxMode_t tx_mode;
} E32_Config_t;

/*============================================================================
 * HANDLE STRUCTURE
 *============================================================================*/

typedef struct {
    /* UART handle */
    UART_HandleTypeDef *huart;
    
    /* GPIO pins */
    GPIO_TypeDef *m0_port;
    uint16_t m0_pin;
    GPIO_TypeDef *m1_port;
    uint16_t m1_pin;
    GPIO_TypeDef *aux_port;
    uint16_t aux_pin;
    
    /* Current mode */
    E32_Mode_t current_mode;
    
    /* Configuration */
    E32_Config_t config;
    
    /* RX buffer */
    uint8_t rx_buffer[E32_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint16_t rx_count;
    
    /* Status flags */
    volatile bool tx_busy;
    volatile bool rx_available;
    
    /* Statistics */
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t tx_errors;
    uint32_t rx_errors;
    
    /* Initialization status */
    bool initialized;
    
} E32_Handle_t;

/*============================================================================
 * RECEIVED PACKET STRUCTURE (for fixed mode)
 *============================================================================*/

typedef struct {
    uint16_t source_address;        /* Source module address */
    uint8_t channel;                /* Source channel */
    uint8_t *data;                  /* Packet data */
    uint8_t length;                 /* Data length */
    int8_t rssi;                    /* RSSI if available */
} E32_Packet_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize E32 LoRa module
 * @param dev Pointer to E32 handle
 * @param huart Pointer to UART handle
 * @param m0_port M0 GPIO port
 * @param m0_pin M0 GPIO pin
 * @param m1_port M1 GPIO port
 * @param m1_pin M1 GPIO pin
 * @param aux_port AUX GPIO port
 * @param aux_pin AUX GPIO pin
 * @return E32_OK on success
 */
E32_Status_t E32_Init(E32_Handle_t *dev, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *m0_port, uint16_t m0_pin,
                      GPIO_TypeDef *m1_port, uint16_t m1_pin,
                      GPIO_TypeDef *aux_port, uint16_t aux_pin);

/**
 * @brief Start receiving data (enables UART interrupt)
 * @param dev Pointer to E32 handle
 * @return E32_OK on success
 */
E32_Status_t E32_StartReceiving(E32_Handle_t *dev);

/**
 * @brief Set operating mode
 * @param dev Pointer to E32 handle
 * @param mode Operating mode
 * @return E32_OK on success
 */
E32_Status_t E32_SetMode(E32_Handle_t *dev, E32_Mode_t mode);

/**
 * @brief Get current operating mode
 * @param dev Pointer to E32 handle
 * @return Current mode
 */
E32_Mode_t E32_GetMode(E32_Handle_t *dev);

/**
 * @brief Wait for AUX pin to go high (module ready)
 * @param dev Pointer to E32 handle
 * @param timeout_ms Timeout in milliseconds
 * @return E32_OK if ready, E32_ERROR_TIMEOUT if timeout
 */
E32_Status_t E32_WaitAuxHigh(E32_Handle_t *dev, uint32_t timeout_ms);

/**
 * @brief Read module configuration
 * @param dev Pointer to E32 handle
 * @return E32_OK on success
 */
E32_Status_t E32_ReadConfig(E32_Handle_t *dev);

/**
 * @brief Write module configuration
 * @param dev Pointer to E32 handle
 * @param save_to_eeprom true to save permanently
 * @return E32_OK on success
 */
E32_Status_t E32_WriteConfig(E32_Handle_t *dev, bool save_to_eeprom);

/**
 * @brief Set module address
 * @param dev Pointer to E32 handle
 * @param address Module address (0x0000-0xFFFF)
 * @return E32_OK on success
 */
E32_Status_t E32_SetAddress(E32_Handle_t *dev, uint16_t address);

/**
 * @brief Set operating channel
 * @param dev Pointer to E32 handle
 * @param channel Channel number (0-31)
 * @return E32_OK on success
 */
E32_Status_t E32_SetChannel(E32_Handle_t *dev, uint8_t channel);

/**
 * @brief Set TX power
 * @param dev Pointer to E32 handle
 * @param power TX power level
 * @return E32_OK on success
 */
E32_Status_t E32_SetTxPower(E32_Handle_t *dev, E32_TxPower_t power);

/**
 * @brief Set air data rate
 * @param dev Pointer to E32 handle
 * @param rate Air data rate
 * @return E32_OK on success
 */
E32_Status_t E32_SetAirRate(E32_Handle_t *dev, E32_AirRate_t rate);

/**
 * @brief Send data (transparent mode)
 * @param dev Pointer to E32 handle
 * @param data Data to send
 * @param length Data length
 * @return E32_OK on success
 */
E32_Status_t E32_Send(E32_Handle_t *dev, const uint8_t *data, uint8_t length);

/**
 * @brief Send data to specific address (fixed mode)
 * @param dev Pointer to E32 handle
 * @param address Destination address
 * @param channel Destination channel
 * @param data Data to send
 * @param length Data length
 * @return E32_OK on success
 */
E32_Status_t E32_SendTo(E32_Handle_t *dev, uint16_t address, uint8_t channel,
                        const uint8_t *data, uint8_t length);

/**
 * @brief Broadcast data to all modules on channel (fixed mode)
 * @param dev Pointer to E32 handle
 * @param data Data to send
 * @param length Data length
 * @return E32_OK on success
 */
E32_Status_t E32_Broadcast(E32_Handle_t *dev, const uint8_t *data, uint8_t length);

/**
 * @brief Check if data is available to read
 * @param dev Pointer to E32 handle
 * @return Number of bytes available
 */
uint16_t E32_Available(E32_Handle_t *dev);

/**
 * @brief Read received data
 * @param dev Pointer to E32 handle
 * @param buffer Buffer to store data
 * @param max_length Maximum bytes to read
 * @return Number of bytes read
 */
uint16_t E32_Read(E32_Handle_t *dev, uint8_t *buffer, uint16_t max_length);

/**
 * @brief Read single byte
 * @param dev Pointer to E32 handle
 * @param byte Pointer to store byte
 * @return E32_OK if byte read, E32_ERROR if no data
 */
E32_Status_t E32_ReadByte(E32_Handle_t *dev, uint8_t *byte);

/**
 * @brief Flush RX buffer
 * @param dev Pointer to E32 handle
 */
void E32_FlushRX(E32_Handle_t *dev);

/**
 * @brief Process received byte (call from UART RX callback)
 * @param dev Pointer to E32 handle
 * @param byte Received byte
 */
void E32_ProcessByte(E32_Handle_t *dev, uint8_t byte);

/**
 * @brief Check if module is busy (AUX low)
 * @param dev Pointer to E32 handle
 * @return true if busy
 */
bool E32_IsBusy(E32_Handle_t *dev);

/**
 * @brief Reset module to factory defaults
 * @param dev Pointer to E32 handle
 * @return E32_OK on success
 */
E32_Status_t E32_Reset(E32_Handle_t *dev);

/**
 * @brief Read module version information
 * @param dev Pointer to E32 handle
 * @param version Buffer to store version (at least 4 bytes)
 * @return E32_OK on success
 */
E32_Status_t E32_ReadVersion(E32_Handle_t *dev, uint8_t *version);

/**
 * @brief Get frequency in MHz for given channel
 * @param channel Channel number (0-31)
 * @return Frequency in MHz
 */
float E32_GetFrequency(uint8_t channel);

/**
 * @brief Get UART baud rate for given setting
 * @param baud Baud rate setting
 * @return Actual baud rate
 */
uint32_t E32_GetBaudRate(E32_UartBaud_t baud);

#ifdef __cplusplus
}
#endif

#endif /* E32_LORA_H */

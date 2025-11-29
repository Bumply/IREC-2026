/**
 * @file w25q.h
 * @brief Winbond W25Q Flash Memory Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * W25Q40CLSNIG - 4Mbit (512KB) SPI Flash
 * - Page size: 256 bytes
 * - Sector size: 4KB (smallest erasable unit)
 * - Block size: 64KB
 * - Total: 128 sectors, 8 blocks
 * 
 * Used for flight data logging during flight.
 */

#ifndef W25Q_H
#define W25Q_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * FLASH SPECIFICATIONS (W25Q40)
 *============================================================================*/

#define W25Q_PAGE_SIZE          256
#define W25Q_SECTOR_SIZE        4096        /* 4 KB */
#define W25Q_BLOCK_SIZE         65536       /* 64 KB */
#define W25Q_TOTAL_SIZE         524288      /* 512 KB (4 Mbit) */

#define W25Q_NUM_PAGES          2048
#define W25Q_NUM_SECTORS        128
#define W25Q_NUM_BLOCKS         8

/*============================================================================
 * COMMAND CODES
 *============================================================================*/

#define W25Q_CMD_WRITE_ENABLE       0x06
#define W25Q_CMD_WRITE_DISABLE      0x04
#define W25Q_CMD_READ_STATUS_1      0x05
#define W25Q_CMD_READ_STATUS_2      0x35
#define W25Q_CMD_WRITE_STATUS       0x01
#define W25Q_CMD_READ_DATA          0x03
#define W25Q_CMD_FAST_READ          0x0B
#define W25Q_CMD_PAGE_PROGRAM       0x02
#define W25Q_CMD_SECTOR_ERASE       0x20    /* 4KB */
#define W25Q_CMD_BLOCK_ERASE_32K    0x52    /* 32KB */
#define W25Q_CMD_BLOCK_ERASE_64K    0xD8    /* 64KB */
#define W25Q_CMD_CHIP_ERASE         0xC7
#define W25Q_CMD_POWER_DOWN         0xB9
#define W25Q_CMD_RELEASE_POWER      0xAB
#define W25Q_CMD_DEVICE_ID          0xAB
#define W25Q_CMD_JEDEC_ID           0x9F
#define W25Q_CMD_UNIQUE_ID          0x4B

/*============================================================================
 * STATUS REGISTER BITS
 *============================================================================*/

#define W25Q_STATUS_BUSY            0x01
#define W25Q_STATUS_WEL             0x02    /* Write Enable Latch */

/*============================================================================
 * JEDEC ID
 *============================================================================*/

#define W25Q_MANUFACTURER_ID        0xEF    /* Winbond */
#define W25Q40_DEVICE_ID            0x4013  /* W25Q40 */

/*============================================================================
 * HANDLE STRUCTURE
 *============================================================================*/

typedef struct {
    /* SPI handle */
    SPI_HandleTypeDef *hspi;
    
    /* Chip select GPIO */
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    
    /* Device info */
    uint8_t manufacturer_id;
    uint16_t device_id;
    uint32_t capacity;          /* In bytes */
    
    /* Current write address for logging */
    uint32_t write_address;
    
    /* Page buffer for write operations */
    uint8_t page_buffer[W25Q_PAGE_SIZE];
    uint16_t buffer_index;
    
    /* Status */
    bool initialized;
    
} W25Q_Handle_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize W25Q flash
 * @param dev Pointer to W25Q handle
 * @param hspi Pointer to SPI handle
 * @param cs_port Chip select GPIO port
 * @param cs_pin Chip select GPIO pin
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_Init(W25Q_Handle_t *dev, SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief Read JEDEC ID
 * @param dev Pointer to W25Q handle
 * @param manufacturer Pointer to store manufacturer ID
 * @param device Pointer to store device ID
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_ReadID(W25Q_Handle_t *dev, uint8_t *manufacturer, uint16_t *device);

/**
 * @brief Read data from flash
 * @param dev Pointer to W25Q handle
 * @param address Start address
 * @param buffer Buffer to store data
 * @param length Number of bytes to read
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_Read(W25Q_Handle_t *dev, uint32_t address, uint8_t *buffer, uint32_t length);

/**
 * @brief Write data to flash (handles page boundaries)
 * @param dev Pointer to W25Q handle
 * @param address Start address
 * @param data Data to write
 * @param length Number of bytes to write
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_Write(W25Q_Handle_t *dev, uint32_t address, const uint8_t *data, uint32_t length);

/**
 * @brief Program a single page (max 256 bytes)
 * @param dev Pointer to W25Q handle
 * @param address Page-aligned address
 * @param data Data to write
 * @param length Number of bytes (max 256)
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_PageProgram(W25Q_Handle_t *dev, uint32_t address, const uint8_t *data, uint16_t length);

/**
 * @brief Erase a 4KB sector
 * @param dev Pointer to W25Q handle
 * @param address Any address within the sector
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_EraseSector(W25Q_Handle_t *dev, uint32_t address);

/**
 * @brief Erase a 64KB block
 * @param dev Pointer to W25Q handle
 * @param address Any address within the block
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_EraseBlock(W25Q_Handle_t *dev, uint32_t address);

/**
 * @brief Erase entire chip
 * @param dev Pointer to W25Q handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_EraseChip(W25Q_Handle_t *dev);

/**
 * @brief Check if flash is busy
 * @param dev Pointer to W25Q handle
 * @return true if busy
 */
bool W25Q_IsBusy(W25Q_Handle_t *dev);

/**
 * @brief Wait for flash to be ready
 * @param dev Pointer to W25Q handle
 * @param timeout_ms Timeout in milliseconds
 * @return HAL_OK if ready, HAL_TIMEOUT if timeout
 */
HAL_StatusTypeDef W25Q_WaitReady(W25Q_Handle_t *dev, uint32_t timeout_ms);

/**
 * @brief Enter power-down mode
 * @param dev Pointer to W25Q handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_PowerDown(W25Q_Handle_t *dev);

/**
 * @brief Wake from power-down mode
 * @param dev Pointer to W25Q handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_WakeUp(W25Q_Handle_t *dev);

/*============================================================================
 * DATA LOGGING FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize logging (set write pointer)
 * @param dev Pointer to W25Q handle
 * @param start_address Starting address for logging
 */
void W25Q_LogInit(W25Q_Handle_t *dev, uint32_t start_address);

/**
 * @brief Append data to log buffer
 * @param dev Pointer to W25Q handle
 * @param data Data to append
 * @param length Number of bytes
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_LogAppend(W25Q_Handle_t *dev, const uint8_t *data, uint16_t length);

/**
 * @brief Flush log buffer to flash
 * @param dev Pointer to W25Q handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Q_LogFlush(W25Q_Handle_t *dev);

/**
 * @brief Get current log write address
 * @param dev Pointer to W25Q handle
 * @return Current write address
 */
uint32_t W25Q_LogGetAddress(W25Q_Handle_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* W25Q_H */

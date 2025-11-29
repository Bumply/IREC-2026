/**
 * @file w25q.c
 * @brief Winbond W25Q Flash Memory Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 */

#include "w25q.h"
#include <string.h>

/*============================================================================
 * PRIVATE DEFINES
 *============================================================================*/

#define W25Q_SPI_TIMEOUT    100
#define W25Q_WRITE_TIMEOUT  100
#define W25Q_ERASE_TIMEOUT  5000

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

static void W25Q_CS_Low(W25Q_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void W25Q_CS_High(W25Q_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef W25Q_WriteEnable(W25Q_Handle_t *dev)
{
    uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
    
    W25Q_CS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    return status;
}

static uint8_t W25Q_ReadStatus(W25Q_Handle_t *dev)
{
    uint8_t cmd = W25Q_CMD_READ_STATUS_1;
    uint8_t status = 0xFF;
    
    W25Q_CS_Low(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, W25Q_SPI_TIMEOUT);
    HAL_SPI_Receive(dev->hspi, &status, 1, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    return status;
}

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

HAL_StatusTypeDef W25Q_Init(W25Q_Handle_t *dev, SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (dev == NULL || hspi == NULL) {
        return HAL_ERROR;
    }
    
    /* Clear handle */
    memset(dev, 0, sizeof(W25Q_Handle_t));
    
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    
    /* CS high (deselected) */
    W25Q_CS_High(dev);
    
    /* Wake up if in power-down */
    W25Q_WakeUp(dev);
    HAL_Delay(1);
    
    /* Read JEDEC ID to verify communication */
    HAL_StatusTypeDef status = W25Q_ReadID(dev, &dev->manufacturer_id, &dev->device_id);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Verify manufacturer ID */
    if (dev->manufacturer_id != W25Q_MANUFACTURER_ID) {
        return HAL_ERROR;
    }
    
    /* Determine capacity from device ID */
    switch (dev->device_id & 0xFF) {
        case 0x12: dev->capacity = 256 * 1024; break;   /* W25Q20 - 2Mbit */
        case 0x13: dev->capacity = 512 * 1024; break;   /* W25Q40 - 4Mbit */
        case 0x14: dev->capacity = 1024 * 1024; break;  /* W25Q80 - 8Mbit */
        case 0x15: dev->capacity = 2048 * 1024; break;  /* W25Q16 - 16Mbit */
        case 0x16: dev->capacity = 4096 * 1024; break;  /* W25Q32 - 32Mbit */
        case 0x17: dev->capacity = 8192 * 1024; break;  /* W25Q64 - 64Mbit */
        case 0x18: dev->capacity = 16384 * 1024; break; /* W25Q128 - 128Mbit */
        default: dev->capacity = W25Q_TOTAL_SIZE; break;
    }
    
    dev->write_address = 0;
    dev->buffer_index = 0;
    dev->initialized = true;
    
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_ReadID(W25Q_Handle_t *dev, uint8_t *manufacturer, uint16_t *device)
{
    uint8_t cmd = W25Q_CMD_JEDEC_ID;
    uint8_t id[3];
    
    W25Q_CS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, W25Q_SPI_TIMEOUT);
    if (status != HAL_OK) {
        W25Q_CS_High(dev);
        return status;
    }
    
    status = HAL_SPI_Receive(dev->hspi, id, 3, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    if (status == HAL_OK) {
        if (manufacturer) *manufacturer = id[0];
        if (device) *device = ((uint16_t)id[1] << 8) | id[2];
    }
    
    return status;
}

HAL_StatusTypeDef W25Q_Read(W25Q_Handle_t *dev, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (dev == NULL || buffer == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    if (address + length > dev->capacity) {
        return HAL_ERROR;
    }
    
    uint8_t cmd[4];
    cmd[0] = W25Q_CMD_READ_DATA;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    
    W25Q_CS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, cmd, 4, W25Q_SPI_TIMEOUT);
    if (status != HAL_OK) {
        W25Q_CS_High(dev);
        return status;
    }
    
    status = HAL_SPI_Receive(dev->hspi, buffer, length, W25Q_SPI_TIMEOUT + (length / 100));
    W25Q_CS_High(dev);
    
    return status;
}

HAL_StatusTypeDef W25Q_Write(W25Q_Handle_t *dev, uint32_t address, const uint8_t *data, uint32_t length)
{
    if (dev == NULL || data == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    if (address + length > dev->capacity) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status;
    uint32_t bytes_written = 0;
    
    while (bytes_written < length) {
        /* Calculate bytes to write in this page */
        uint32_t page_offset = (address + bytes_written) % W25Q_PAGE_SIZE;
        uint32_t bytes_remaining = length - bytes_written;
        uint32_t bytes_to_write = W25Q_PAGE_SIZE - page_offset;
        
        if (bytes_to_write > bytes_remaining) {
            bytes_to_write = bytes_remaining;
        }
        
        /* Program the page */
        status = W25Q_PageProgram(dev, address + bytes_written, 
                                  data + bytes_written, bytes_to_write);
        if (status != HAL_OK) {
            return status;
        }
        
        bytes_written += bytes_to_write;
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_PageProgram(W25Q_Handle_t *dev, uint32_t address, const uint8_t *data, uint16_t length)
{
    if (dev == NULL || data == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    if (length > W25Q_PAGE_SIZE) {
        length = W25Q_PAGE_SIZE;
    }
    
    /* Wait for any previous operation */
    HAL_StatusTypeDef status = W25Q_WaitReady(dev, W25Q_WRITE_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(dev);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Send page program command */
    uint8_t cmd[4];
    cmd[0] = W25Q_CMD_PAGE_PROGRAM;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    
    W25Q_CS_Low(dev);
    status = HAL_SPI_Transmit(dev->hspi, cmd, 4, W25Q_SPI_TIMEOUT);
    if (status != HAL_OK) {
        W25Q_CS_High(dev);
        return status;
    }
    
    status = HAL_SPI_Transmit(dev->hspi, (uint8_t*)data, length, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    if (status != HAL_OK) {
        return status;
    }
    
    /* Wait for programming to complete */
    return W25Q_WaitReady(dev, W25Q_WRITE_TIMEOUT);
}

HAL_StatusTypeDef W25Q_EraseSector(W25Q_Handle_t *dev, uint32_t address)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* Align to sector boundary */
    address = address & ~(W25Q_SECTOR_SIZE - 1);
    
    /* Wait for ready */
    HAL_StatusTypeDef status = W25Q_WaitReady(dev, W25Q_WRITE_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(dev);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Send sector erase command */
    uint8_t cmd[4];
    cmd[0] = W25Q_CMD_SECTOR_ERASE;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    
    W25Q_CS_Low(dev);
    status = HAL_SPI_Transmit(dev->hspi, cmd, 4, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    if (status != HAL_OK) {
        return status;
    }
    
    /* Wait for erase to complete (typ. 45ms, max 400ms) */
    return W25Q_WaitReady(dev, W25Q_ERASE_TIMEOUT);
}

HAL_StatusTypeDef W25Q_EraseBlock(W25Q_Handle_t *dev, uint32_t address)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* Align to block boundary */
    address = address & ~(W25Q_BLOCK_SIZE - 1);
    
    /* Wait for ready */
    HAL_StatusTypeDef status = W25Q_WaitReady(dev, W25Q_WRITE_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(dev);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Send block erase command */
    uint8_t cmd[4];
    cmd[0] = W25Q_CMD_BLOCK_ERASE_64K;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    
    W25Q_CS_Low(dev);
    status = HAL_SPI_Transmit(dev->hspi, cmd, 4, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    if (status != HAL_OK) {
        return status;
    }
    
    /* Wait for erase to complete (typ. 150ms, max 2000ms) */
    return W25Q_WaitReady(dev, W25Q_ERASE_TIMEOUT);
}

HAL_StatusTypeDef W25Q_EraseChip(W25Q_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* Wait for ready */
    HAL_StatusTypeDef status = W25Q_WaitReady(dev, W25Q_WRITE_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(dev);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Send chip erase command */
    uint8_t cmd = W25Q_CMD_CHIP_ERASE;
    
    W25Q_CS_Low(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    if (status != HAL_OK) {
        return status;
    }
    
    /* Wait for erase to complete (typ. 3s, max 10s for 4Mbit) */
    return W25Q_WaitReady(dev, 15000);
}

bool W25Q_IsBusy(W25Q_Handle_t *dev)
{
    return (W25Q_ReadStatus(dev) & W25Q_STATUS_BUSY) != 0;
}

HAL_StatusTypeDef W25Q_WaitReady(W25Q_Handle_t *dev, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    
    while (W25Q_IsBusy(dev)) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_PowerDown(W25Q_Handle_t *dev)
{
    uint8_t cmd = W25Q_CMD_POWER_DOWN;
    
    W25Q_CS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    return status;
}

HAL_StatusTypeDef W25Q_WakeUp(W25Q_Handle_t *dev)
{
    uint8_t cmd = W25Q_CMD_RELEASE_POWER;
    
    W25Q_CS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, W25Q_SPI_TIMEOUT);
    W25Q_CS_High(dev);
    
    /* tRES1 = 3us */
    HAL_Delay(1);
    
    return status;
}

/*============================================================================
 * DATA LOGGING FUNCTIONS
 *============================================================================*/

void W25Q_LogInit(W25Q_Handle_t *dev, uint32_t start_address)
{
    if (dev == NULL) return;
    
    dev->write_address = start_address;
    dev->buffer_index = 0;
    memset(dev->page_buffer, 0xFF, W25Q_PAGE_SIZE);
}

HAL_StatusTypeDef W25Q_LogAppend(W25Q_Handle_t *dev, const uint8_t *data, uint16_t length)
{
    if (dev == NULL || data == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_OK;
    
    for (uint16_t i = 0; i < length; i++) {
        dev->page_buffer[dev->buffer_index++] = data[i];
        
        /* Flush when buffer is full */
        if (dev->buffer_index >= W25Q_PAGE_SIZE) {
            status = W25Q_LogFlush(dev);
            if (status != HAL_OK) {
                return status;
            }
        }
    }
    
    return status;
}

HAL_StatusTypeDef W25Q_LogFlush(W25Q_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    if (dev->buffer_index == 0) {
        return HAL_OK;  /* Nothing to flush */
    }
    
    /* Check if we need to erase sector */
    if ((dev->write_address % W25Q_SECTOR_SIZE) == 0) {
        HAL_StatusTypeDef status = W25Q_EraseSector(dev, dev->write_address);
        if (status != HAL_OK) {
            return status;
        }
    }
    
    /* Write the page */
    HAL_StatusTypeDef status = W25Q_PageProgram(dev, dev->write_address, 
                                                 dev->page_buffer, dev->buffer_index);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Update address */
    dev->write_address += dev->buffer_index;
    
    /* Reset buffer */
    dev->buffer_index = 0;
    memset(dev->page_buffer, 0xFF, W25Q_PAGE_SIZE);
    
    return HAL_OK;
}

uint32_t W25Q_LogGetAddress(W25Q_Handle_t *dev)
{
    if (dev == NULL) return 0;
    return dev->write_address + dev->buffer_index;
}

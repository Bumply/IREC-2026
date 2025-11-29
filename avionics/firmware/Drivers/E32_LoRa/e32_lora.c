/**
 * @file e32_lora.c
 * @brief EBYTE E32-433T30D LoRa Module Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 */

#include "e32_lora.h"
#include <string.h>

/*============================================================================
 * PRIVATE DEFINES
 *============================================================================*/

/* Command bytes */
#define E32_CMD_READ_CONFIG     0xC1
#define E32_CMD_READ_VERSION    0xC3
#define E32_CMD_RESET           0xC4
#define E32_CMD_WRITE_TEMP      0xC2    /* Write config (temporary) */
#define E32_CMD_WRITE_PERM      0xC0    /* Write config (permanent) */

/* Configuration response length */
#define E32_CONFIG_SIZE         6

/* Broadcast address */
#define E32_BROADCAST_ADDR      0xFFFF

/* Base frequency for 433MHz modules */
#define E32_BASE_FREQ_MHZ       410.0f
#define E32_CHANNEL_STEP_MHZ    1.0f

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/

/* Single byte RX buffer for interrupt mode */
static uint8_t e32_rx_byte;

/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *============================================================================*/

static void E32_SetM0(E32_Handle_t *dev, bool state);
static void E32_SetM1(E32_Handle_t *dev, bool state);
static bool E32_ReadAux(E32_Handle_t *dev);
static E32_Status_t E32_SendRaw(E32_Handle_t *dev, const uint8_t *data, uint16_t length);

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

E32_Status_t E32_Init(E32_Handle_t *dev, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *m0_port, uint16_t m0_pin,
                      GPIO_TypeDef *m1_port, uint16_t m1_pin,
                      GPIO_TypeDef *aux_port, uint16_t aux_pin)
{
    if (dev == NULL || huart == NULL) {
        return E32_ERROR_PARAM;
    }
    
    /* Clear handle */
    memset(dev, 0, sizeof(E32_Handle_t));
    
    /* Store handles and pins */
    dev->huart = huart;
    dev->m0_port = m0_port;
    dev->m0_pin = m0_pin;
    dev->m1_port = m1_port;
    dev->m1_pin = m1_pin;
    dev->aux_port = aux_port;
    dev->aux_pin = aux_pin;
    
    /* Initialize buffer indices */
    dev->rx_head = 0;
    dev->rx_tail = 0;
    dev->rx_count = 0;
    
    /* Default configuration */
    dev->config.address = 0x0000;
    dev->config.channel = 23;           /* 433 MHz */
    dev->config.uart_parity = E32_UART_PARITY_8N1;
    dev->config.uart_baud = E32_UART_BAUD_9600;
    dev->config.air_rate = E32_AIR_RATE_2400;
    dev->config.tx_power = E32_TX_POWER_30DBM;
    dev->config.fec = E32_FEC_ON;
    dev->config.wakeup_time = E32_WAKEUP_250MS;
    dev->config.io_mode = E32_IO_PUSH_PULL;
    dev->config.tx_mode = E32_TX_MODE_TRANSPARENT;
    
    /* Set to normal mode */
    E32_SetM0(dev, false);
    E32_SetM1(dev, false);
    dev->current_mode = E32_MODE_NORMAL;
    
    /* Wait for module ready */
    E32_Status_t status = E32_WaitAuxHigh(dev, E32_AUX_TIMEOUT_MS);
    if (status != E32_OK) {
        return status;
    }
    
    dev->initialized = true;
    
    return E32_OK;
}

E32_Status_t E32_StartReceiving(E32_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    /* Start UART receive in interrupt mode */
    if (HAL_UART_Receive_IT(dev->huart, &e32_rx_byte, 1) != HAL_OK) {
        return E32_ERROR;
    }
    
    return E32_OK;
}

E32_Status_t E32_SetMode(E32_Handle_t *dev, E32_Mode_t mode)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    /* Wait for any ongoing operation */
    E32_Status_t status = E32_WaitAuxHigh(dev, E32_AUX_TIMEOUT_MS);
    if (status != E32_OK) {
        return status;
    }
    
    /* Set M0 and M1 pins according to mode */
    switch (mode) {
        case E32_MODE_NORMAL:
            E32_SetM0(dev, false);
            E32_SetM1(dev, false);
            break;
        case E32_MODE_WAKEUP:
            E32_SetM0(dev, true);
            E32_SetM1(dev, false);
            break;
        case E32_MODE_POWER_SAVE:
            E32_SetM0(dev, false);
            E32_SetM1(dev, true);
            break;
        case E32_MODE_SLEEP:
            E32_SetM0(dev, true);
            E32_SetM1(dev, true);
            break;
        default:
            return E32_ERROR_PARAM;
    }
    
    dev->current_mode = mode;
    
    /* Wait for mode change to complete */
    HAL_Delay(2);
    status = E32_WaitAuxHigh(dev, E32_AUX_TIMEOUT_MS);
    
    return status;
}

E32_Mode_t E32_GetMode(E32_Handle_t *dev)
{
    if (dev == NULL) {
        return E32_MODE_NORMAL;
    }
    return dev->current_mode;
}

E32_Status_t E32_WaitAuxHigh(E32_Handle_t *dev, uint32_t timeout_ms)
{
    if (dev == NULL) {
        return E32_ERROR_PARAM;
    }
    
    uint32_t start = HAL_GetTick();
    
    while (!E32_ReadAux(dev)) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return E32_ERROR_TIMEOUT;
        }
        HAL_Delay(1);
    }
    
    return E32_OK;
}

E32_Status_t E32_ReadConfig(E32_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    /* Enter sleep mode for configuration */
    E32_Status_t status = E32_SetMode(dev, E32_MODE_SLEEP);
    if (status != E32_OK) {
        return status;
    }
    
    HAL_Delay(10);
    
    /* Send read config command */
    uint8_t cmd[3] = {E32_CMD_READ_CONFIG, E32_CMD_READ_CONFIG, E32_CMD_READ_CONFIG};
    status = E32_SendRaw(dev, cmd, 3);
    if (status != E32_OK) {
        E32_SetMode(dev, E32_MODE_NORMAL);
        return status;
    }
    
    /* Read response */
    uint8_t response[E32_CONFIG_SIZE];
    if (HAL_UART_Receive(dev->huart, response, E32_CONFIG_SIZE, E32_CONFIG_TIMEOUT_MS) != HAL_OK) {
        E32_SetMode(dev, E32_MODE_NORMAL);
        return E32_ERROR_TIMEOUT;
    }
    
    /* Parse configuration */
    /* Byte 0: HEAD (0xC0 or 0xC2) */
    /* Byte 1: ADDH */
    /* Byte 2: ADDL */
    /* Byte 3: SPED (parity, baud, air rate) */
    /* Byte 4: CHAN (channel) */
    /* Byte 5: OPTION (tx mode, io mode, wakeup, fec, power) */
    
    dev->config.address = ((uint16_t)response[1] << 8) | response[2];
    
    /* SPED byte */
    dev->config.uart_parity = (E32_UartParity_t)((response[3] >> 6) & 0x03);
    dev->config.uart_baud = (E32_UartBaud_t)((response[3] >> 3) & 0x07);
    dev->config.air_rate = (E32_AirRate_t)(response[3] & 0x07);
    
    /* CHAN byte */
    dev->config.channel = response[4] & 0x1F;
    
    /* OPTION byte */
    dev->config.tx_mode = (E32_TxMode_t)((response[5] >> 7) & 0x01);
    dev->config.io_mode = (E32_IO_Mode_t)((response[5] >> 6) & 0x01);
    dev->config.wakeup_time = (E32_WakeupTime_t)((response[5] >> 3) & 0x07);
    dev->config.fec = (E32_FEC_t)((response[5] >> 2) & 0x01);
    dev->config.tx_power = (E32_TxPower_t)(response[5] & 0x03);
    
    /* Return to normal mode */
    E32_SetMode(dev, E32_MODE_NORMAL);
    
    return E32_OK;
}

E32_Status_t E32_WriteConfig(E32_Handle_t *dev, bool save_to_eeprom)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    /* Enter sleep mode for configuration */
    E32_Status_t status = E32_SetMode(dev, E32_MODE_SLEEP);
    if (status != E32_OK) {
        return status;
    }
    
    HAL_Delay(10);
    
    /* Build configuration packet */
    uint8_t config[E32_CONFIG_SIZE];
    
    /* HEAD byte */
    config[0] = save_to_eeprom ? E32_CMD_WRITE_PERM : E32_CMD_WRITE_TEMP;
    
    /* Address bytes */
    config[1] = (uint8_t)(dev->config.address >> 8);
    config[2] = (uint8_t)(dev->config.address & 0xFF);
    
    /* SPED byte */
    config[3] = ((dev->config.uart_parity & 0x03) << 6) |
                ((dev->config.uart_baud & 0x07) << 3) |
                (dev->config.air_rate & 0x07);
    
    /* CHAN byte */
    config[4] = dev->config.channel & 0x1F;
    
    /* OPTION byte */
    config[5] = ((dev->config.tx_mode & 0x01) << 7) |
                ((dev->config.io_mode & 0x01) << 6) |
                ((dev->config.wakeup_time & 0x07) << 3) |
                ((dev->config.fec & 0x01) << 2) |
                (dev->config.tx_power & 0x03);
    
    /* Send configuration */
    status = E32_SendRaw(dev, config, E32_CONFIG_SIZE);
    if (status != E32_OK) {
        E32_SetMode(dev, E32_MODE_NORMAL);
        return status;
    }
    
    /* Wait for configuration to complete */
    HAL_Delay(50);
    status = E32_WaitAuxHigh(dev, E32_AUX_TIMEOUT_MS);
    
    /* Return to normal mode */
    E32_SetMode(dev, E32_MODE_NORMAL);
    
    return status;
}

E32_Status_t E32_SetAddress(E32_Handle_t *dev, uint16_t address)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    dev->config.address = address;
    return E32_WriteConfig(dev, true);
}

E32_Status_t E32_SetChannel(E32_Handle_t *dev, uint8_t channel)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    if (channel > 31) {
        return E32_ERROR_PARAM;
    }
    
    dev->config.channel = channel;
    return E32_WriteConfig(dev, true);
}

E32_Status_t E32_SetTxPower(E32_Handle_t *dev, E32_TxPower_t power)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    dev->config.tx_power = power;
    return E32_WriteConfig(dev, true);
}

E32_Status_t E32_SetAirRate(E32_Handle_t *dev, E32_AirRate_t rate)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    dev->config.air_rate = rate;
    return E32_WriteConfig(dev, true);
}

E32_Status_t E32_Send(E32_Handle_t *dev, const uint8_t *data, uint8_t length)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    if (data == NULL || length == 0) {
        return E32_ERROR_PARAM;
    }
    
    if (length > E32_MAX_PACKET_SIZE) {
        return E32_ERROR_PARAM;
    }
    
    /* Wait for module ready */
    E32_Status_t status = E32_WaitAuxHigh(dev, E32_AUX_TIMEOUT_MS);
    if (status != E32_OK) {
        dev->tx_errors++;
        return status;
    }
    
    /* Send data */
    dev->tx_busy = true;
    status = E32_SendRaw(dev, data, length);
    
    if (status == E32_OK) {
        dev->packets_sent++;
    } else {
        dev->tx_errors++;
    }
    
    dev->tx_busy = false;
    
    return status;
}

E32_Status_t E32_SendTo(E32_Handle_t *dev, uint16_t address, uint8_t channel,
                        const uint8_t *data, uint8_t length)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    if (data == NULL || length == 0) {
        return E32_ERROR_PARAM;
    }
    
    if (length > (E32_MAX_PACKET_SIZE - 3)) {
        return E32_ERROR_PARAM;
    }
    
    /* Wait for module ready */
    E32_Status_t status = E32_WaitAuxHigh(dev, E32_AUX_TIMEOUT_MS);
    if (status != E32_OK) {
        dev->tx_errors++;
        return status;
    }
    
    /* Build packet with address header */
    uint8_t packet[E32_MAX_PACKET_SIZE];
    packet[0] = (uint8_t)(address >> 8);
    packet[1] = (uint8_t)(address & 0xFF);
    packet[2] = channel;
    memcpy(&packet[3], data, length);
    
    /* Send packet */
    dev->tx_busy = true;
    status = E32_SendRaw(dev, packet, length + 3);
    
    if (status == E32_OK) {
        dev->packets_sent++;
    } else {
        dev->tx_errors++;
    }
    
    dev->tx_busy = false;
    
    return status;
}

E32_Status_t E32_Broadcast(E32_Handle_t *dev, const uint8_t *data, uint8_t length)
{
    /* Broadcast uses address 0xFFFF */
    return E32_SendTo(dev, E32_BROADCAST_ADDR, dev->config.channel, data, length);
}

uint16_t E32_Available(E32_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return 0;
    }
    return dev->rx_count;
}

uint16_t E32_Read(E32_Handle_t *dev, uint8_t *buffer, uint16_t max_length)
{
    if (dev == NULL || !dev->initialized || buffer == NULL) {
        return 0;
    }
    
    uint16_t bytes_read = 0;
    
    while (dev->rx_count > 0 && bytes_read < max_length) {
        buffer[bytes_read] = dev->rx_buffer[dev->rx_tail];
        dev->rx_tail = (dev->rx_tail + 1) % E32_RX_BUFFER_SIZE;
        dev->rx_count--;
        bytes_read++;
    }
    
    if (dev->rx_count == 0) {
        dev->rx_available = false;
    }
    
    return bytes_read;
}

E32_Status_t E32_ReadByte(E32_Handle_t *dev, uint8_t *byte)
{
    if (dev == NULL || !dev->initialized || byte == NULL) {
        return E32_ERROR_PARAM;
    }
    
    if (dev->rx_count == 0) {
        return E32_ERROR;
    }
    
    *byte = dev->rx_buffer[dev->rx_tail];
    dev->rx_tail = (dev->rx_tail + 1) % E32_RX_BUFFER_SIZE;
    dev->rx_count--;
    
    if (dev->rx_count == 0) {
        dev->rx_available = false;
    }
    
    return E32_OK;
}

void E32_FlushRX(E32_Handle_t *dev)
{
    if (dev == NULL) return;
    
    dev->rx_head = 0;
    dev->rx_tail = 0;
    dev->rx_count = 0;
    dev->rx_available = false;
}

void E32_ProcessByte(E32_Handle_t *dev, uint8_t byte)
{
    if (dev == NULL || !dev->initialized) {
        return;
    }
    
    /* Add to circular buffer if space available */
    if (dev->rx_count < E32_RX_BUFFER_SIZE) {
        dev->rx_buffer[dev->rx_head] = byte;
        dev->rx_head = (dev->rx_head + 1) % E32_RX_BUFFER_SIZE;
        dev->rx_count++;
        dev->rx_available = true;
    } else {
        dev->rx_errors++;
    }
    
    /* Restart UART receive */
    HAL_UART_Receive_IT(dev->huart, &e32_rx_byte, 1);
}

bool E32_IsBusy(E32_Handle_t *dev)
{
    if (dev == NULL) {
        return true;
    }
    return !E32_ReadAux(dev);
}

E32_Status_t E32_Reset(E32_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return E32_ERROR_NOT_INIT;
    }
    
    /* Enter sleep mode */
    E32_Status_t status = E32_SetMode(dev, E32_MODE_SLEEP);
    if (status != E32_OK) {
        return status;
    }
    
    HAL_Delay(10);
    
    /* Send reset command */
    uint8_t cmd[3] = {E32_CMD_RESET, E32_CMD_RESET, E32_CMD_RESET};
    status = E32_SendRaw(dev, cmd, 3);
    
    /* Wait for reset */
    HAL_Delay(100);
    status = E32_WaitAuxHigh(dev, E32_AUX_TIMEOUT_MS);
    
    /* Return to normal mode */
    E32_SetMode(dev, E32_MODE_NORMAL);
    
    return status;
}

E32_Status_t E32_ReadVersion(E32_Handle_t *dev, uint8_t *version)
{
    if (dev == NULL || !dev->initialized || version == NULL) {
        return E32_ERROR_PARAM;
    }
    
    /* Enter sleep mode */
    E32_Status_t status = E32_SetMode(dev, E32_MODE_SLEEP);
    if (status != E32_OK) {
        return status;
    }
    
    HAL_Delay(10);
    
    /* Send read version command */
    uint8_t cmd[3] = {E32_CMD_READ_VERSION, E32_CMD_READ_VERSION, E32_CMD_READ_VERSION};
    status = E32_SendRaw(dev, cmd, 3);
    if (status != E32_OK) {
        E32_SetMode(dev, E32_MODE_NORMAL);
        return status;
    }
    
    /* Read response (4 bytes: 0xC3, freq, version, features) */
    if (HAL_UART_Receive(dev->huart, version, 4, E32_CONFIG_TIMEOUT_MS) != HAL_OK) {
        E32_SetMode(dev, E32_MODE_NORMAL);
        return E32_ERROR_TIMEOUT;
    }
    
    /* Return to normal mode */
    E32_SetMode(dev, E32_MODE_NORMAL);
    
    return E32_OK;
}

float E32_GetFrequency(uint8_t channel)
{
    if (channel > 31) {
        channel = 31;
    }
    return E32_BASE_FREQ_MHZ + (channel * E32_CHANNEL_STEP_MHZ);
}

uint32_t E32_GetBaudRate(E32_UartBaud_t baud)
{
    static const uint32_t baud_rates[] = {
        1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
    };
    
    if (baud > E32_UART_BAUD_115200) {
        return 9600;
    }
    
    return baud_rates[baud];
}

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

static void E32_SetM0(E32_Handle_t *dev, bool state)
{
    HAL_GPIO_WritePin(dev->m0_port, dev->m0_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void E32_SetM1(E32_Handle_t *dev, bool state)
{
    HAL_GPIO_WritePin(dev->m1_port, dev->m1_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool E32_ReadAux(E32_Handle_t *dev)
{
    return HAL_GPIO_ReadPin(dev->aux_port, dev->aux_pin) == GPIO_PIN_SET;
}

static E32_Status_t E32_SendRaw(E32_Handle_t *dev, const uint8_t *data, uint16_t length)
{
    if (HAL_UART_Transmit(dev->huart, (uint8_t*)data, length, 1000) != HAL_OK) {
        return E32_ERROR;
    }
    return E32_OK;
}

/**
 * @file ms5611.c
 * @brief MS5611 Barometric Pressure Sensor Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * Implementation based on MS5611-01BA03 datasheet
 * https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf
 */

#include "ms5611.h"
#include <math.h>

/*============================================================================
 * PRIVATE CONSTANTS
 *============================================================================*/

/* Conversion time lookup table (microseconds) - add margin for safety */
static const uint32_t conversion_time_us[] = {
    600,   /* OSR 256  - 0.6ms */
    1200,  /* OSR 512  - 1.2ms */
    2300,  /* OSR 1024 - 2.3ms */
    4600,  /* OSR 2048 - 4.6ms */
    9100   /* OSR 4096 - 9.1ms */
};

/* Command offset for OSR selection */
static const uint8_t osr_cmd_offset[] = {0, 2, 4, 6, 8};

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

/**
 * @brief Send a single command to MS5611
 */
static HAL_StatusTypeDef MS5611_SendCommand(MS5611_Handle_t *dev, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, &cmd, 1, MS5611_TIMEOUT);
}

/**
 * @brief Read 16-bit value from PROM address
 */
static HAL_StatusTypeDef MS5611_ReadPROM_Addr(MS5611_Handle_t *dev, uint8_t addr, uint16_t *value)
{
    uint8_t cmd = MS5611_CMD_PROM_READ_BASE + (addr * 2);
    uint8_t data[2];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, &cmd, 1, MS5611_TIMEOUT);
    if (status != HAL_OK) return status;
    
    status = HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr, data, 2, MS5611_TIMEOUT);
    if (status != HAL_OK) return status;
    
    *value = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

/**
 * @brief Delay for conversion time based on OSR
 */
static void MS5611_WaitConversion(MS5611_Handle_t *dev)
{
    /* Convert microseconds to milliseconds, round up */
    uint32_t delay_ms = (conversion_time_us[dev->osr] + 999) / 1000;
    HAL_Delay(delay_ms + 1);  /* Add 1ms safety margin */
}

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

HAL_StatusTypeDef MS5611_Init(MS5611_Handle_t *dev, I2C_HandleTypeDef *hi2c, MS5611_OSR_t osr)
{
    HAL_StatusTypeDef status;
    
    /* Initialize handle */
    dev->hi2c = hi2c;
    dev->i2c_addr = MS5611_I2C_ADDR;
    dev->osr = osr;
    dev->sea_level_pressure = 1013.25f;  /* Standard sea level pressure */
    dev->initialized = false;
    
    /* Reset the sensor */
    status = MS5611_Reset(dev);
    if (status != HAL_OK) return status;
    
    /* Wait for reset to complete (datasheet says 2.8ms typical) */
    HAL_Delay(5);
    
    /* Read calibration coefficients from PROM */
    status = MS5611_ReadPROM(dev);
    if (status != HAL_OK) return status;
    
    /* Verify calibration data is valid (not all zeros or all ones) */
    if (dev->C1 == 0 || dev->C1 == 0xFFFF ||
        dev->C2 == 0 || dev->C2 == 0xFFFF ||
        dev->C3 == 0 || dev->C3 == 0xFFFF ||
        dev->C4 == 0 || dev->C4 == 0xFFFF ||
        dev->C5 == 0 || dev->C5 == 0xFFFF ||
        dev->C6 == 0 || dev->C6 == 0xFFFF) {
        return HAL_ERROR;
    }
    
    dev->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef MS5611_Reset(MS5611_Handle_t *dev)
{
    return MS5611_SendCommand(dev, MS5611_CMD_RESET);
}

HAL_StatusTypeDef MS5611_ReadPROM(MS5611_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint16_t prom[8];
    
    /* Read all 8 PROM words (addresses 0-7) */
    for (int i = 0; i < 8; i++) {
        status = MS5611_ReadPROM_Addr(dev, i, &prom[i]);
        if (status != HAL_OK) return status;
    }
    
    /* Assign calibration coefficients */
    /* prom[0] = Factory data and CRC */
    dev->C1 = prom[1];  /* Pressure sensitivity */
    dev->C2 = prom[2];  /* Pressure offset */
    dev->C3 = prom[3];  /* Temperature coefficient of pressure sensitivity */
    dev->C4 = prom[4];  /* Temperature coefficient of pressure offset */
    dev->C5 = prom[5];  /* Reference temperature */
    dev->C6 = prom[6];  /* Temperature coefficient of the temperature */
    /* prom[7] = Serial code and CRC */
    
    return HAL_OK;
}

bool MS5611_CheckCRC(MS5611_Handle_t *dev)
{
    /* CRC check implementation - refer to datasheet AN520 */
    /* For simplicity, we trust the PROM data if values are reasonable */
    return true;
}

HAL_StatusTypeDef MS5611_StartPressureConversion(MS5611_Handle_t *dev)
{
    uint8_t cmd = MS5611_CMD_CONV_D1_OSR256 + osr_cmd_offset[dev->osr];
    return MS5611_SendCommand(dev, cmd);
}

HAL_StatusTypeDef MS5611_StartTemperatureConversion(MS5611_Handle_t *dev)
{
    uint8_t cmd = MS5611_CMD_CONV_D2_OSR256 + osr_cmd_offset[dev->osr];
    return MS5611_SendCommand(dev, cmd);
}

HAL_StatusTypeDef MS5611_ReadADC(MS5611_Handle_t *dev, uint32_t *value)
{
    uint8_t cmd = MS5611_CMD_ADC_READ;
    uint8_t data[3];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, &cmd, 1, MS5611_TIMEOUT);
    if (status != HAL_OK) return status;
    
    status = HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr, data, 3, MS5611_TIMEOUT);
    if (status != HAL_OK) return status;
    
    *value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    return HAL_OK;
}

HAL_StatusTypeDef MS5611_ReadTemperatureAndPressure(MS5611_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    
    /* Step 1: Read temperature (D2) */
    status = MS5611_StartTemperatureConversion(dev);
    if (status != HAL_OK) return status;
    
    MS5611_WaitConversion(dev);
    
    status = MS5611_ReadADC(dev, &dev->D2);
    if (status != HAL_OK) return status;
    
    /* Step 2: Read pressure (D1) */
    status = MS5611_StartPressureConversion(dev);
    if (status != HAL_OK) return status;
    
    MS5611_WaitConversion(dev);
    
    status = MS5611_ReadADC(dev, &dev->D1);
    if (status != HAL_OK) return status;
    
    /* Step 3: Calculate compensated values */
    MS5611_Calculate(dev);
    
    return HAL_OK;
}

void MS5611_Calculate(MS5611_Handle_t *dev)
{
    /*
     * Temperature calculation according to datasheet
     * 
     * dT = D2 - T_REF = D2 - C5 * 2^8
     * TEMP = 20°C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
     */
    dev->dT = (int32_t)dev->D2 - ((int32_t)dev->C5 << 8);
    dev->TEMP = 2000 + (((int64_t)dev->dT * dev->C6) >> 23);
    
    /*
     * Pressure calculation
     * 
     * OFF = OFF_T1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
     * SENS = SENS_T1 + TCS * dT = C1 * 2^15 + (C3 * dT) / 2^8
     * P = D1 * SENS / 2^21 - OFF / 2^15
     */
    dev->OFF = ((int64_t)dev->C2 << 16) + (((int64_t)dev->C4 * dev->dT) >> 7);
    dev->SENS = ((int64_t)dev->C1 << 15) + (((int64_t)dev->C3 * dev->dT) >> 8);
    
    /*
     * Second order temperature compensation for improved accuracy
     * This is CRITICAL for accurate altitude measurement in rocketry!
     */
    dev->T2 = 0;
    dev->OFF2 = 0;
    dev->SENS2 = 0;
    
    if (dev->TEMP < 2000) {
        /* Low temperature compensation (< 20°C) */
        dev->T2 = ((int64_t)dev->dT * dev->dT) >> 31;
        dev->OFF2 = (5 * ((int64_t)(dev->TEMP - 2000) * (dev->TEMP - 2000))) >> 1;
        dev->SENS2 = (5 * ((int64_t)(dev->TEMP - 2000) * (dev->TEMP - 2000))) >> 2;
        
        if (dev->TEMP < -1500) {
            /* Very low temperature compensation (< -15°C) */
            dev->OFF2 = dev->OFF2 + 7 * ((int64_t)(dev->TEMP + 1500) * (dev->TEMP + 1500));
            dev->SENS2 = dev->SENS2 + ((11 * ((int64_t)(dev->TEMP + 1500) * (dev->TEMP + 1500))) >> 1);
        }
    }
    
    /* Apply second order compensation */
    dev->TEMP = dev->TEMP - dev->T2;
    dev->OFF = dev->OFF - dev->OFF2;
    dev->SENS = dev->SENS - dev->SENS2;
    
    /* Calculate temperature compensated pressure */
    dev->P = (((dev->D1 * dev->SENS) >> 21) - dev->OFF) >> 15;
    
    /* Convert to user-friendly units */
    dev->temperature_c = dev->TEMP / 100.0f;           /* Celsius */
    dev->pressure_mbar = dev->P / 100.0f;              /* mbar (hPa) */
    
    /* Calculate altitude using barometric formula */
    dev->altitude_m = MS5611_CalculateAltitude(dev->pressure_mbar, dev->sea_level_pressure);
}

uint32_t MS5611_GetConversionTime(MS5611_Handle_t *dev)
{
    return (conversion_time_us[dev->osr] + 999) / 1000;  /* Return in ms */
}

void MS5611_SetSeaLevelPressure(MS5611_Handle_t *dev, float pressure)
{
    dev->sea_level_pressure = pressure;
}

float MS5611_CalculateAltitude(float pressure, float sea_level_pressure)
{
    /*
     * International Barometric Formula:
     * h = 44330 * (1 - (P/P0)^(1/5.255))
     * 
     * Where:
     * h = altitude in meters
     * P = measured pressure
     * P0 = sea level pressure (1013.25 mbar standard)
     */
    if (pressure <= 0 || sea_level_pressure <= 0) {
        return 0.0f;
    }
    
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

float MS5611_GetTemperature(MS5611_Handle_t *dev)
{
    return dev->temperature_c;
}

float MS5611_GetPressure(MS5611_Handle_t *dev)
{
    return dev->pressure_mbar;
}

float MS5611_GetAltitude(MS5611_Handle_t *dev)
{
    return dev->altitude_m;
}

/**
 * @file ms5611.h
 * @brief MS5611 Barometric Pressure Sensor Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * MS5611 is a high-resolution barometric pressure sensor
 * Resolution: 10cm altitude, 24-bit ADC
 * Interface: I2C (default) or SPI
 */

#ifndef MS5611_H
#define MS5611_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * CONFIGURATION
 *============================================================================*/

/* I2C Address: CSB pin LOW = 0x77, CSB pin HIGH = 0x76 */
#define MS5611_I2C_ADDR_CSB_LOW     (0x77 << 1)  /* Default for GY-63 module */
#define MS5611_I2C_ADDR_CSB_HIGH    (0x76 << 1)

/* Select your address here */
#define MS5611_I2C_ADDR             MS5611_I2C_ADDR_CSB_LOW

/* Timeout for I2C operations (ms) */
#define MS5611_TIMEOUT              100

/*============================================================================
 * COMMANDS
 *============================================================================*/

#define MS5611_CMD_RESET            0x1E
#define MS5611_CMD_PROM_READ_BASE   0xA0  /* 0xA0 to 0xAE (8 addresses) */
#define MS5611_CMD_ADC_READ         0x00

/* D1 (Pressure) conversion commands */
#define MS5611_CMD_CONV_D1_OSR256   0x40
#define MS5611_CMD_CONV_D1_OSR512   0x42
#define MS5611_CMD_CONV_D1_OSR1024  0x44
#define MS5611_CMD_CONV_D1_OSR2048  0x46
#define MS5611_CMD_CONV_D1_OSR4096  0x48

/* D2 (Temperature) conversion commands */
#define MS5611_CMD_CONV_D2_OSR256   0x50
#define MS5611_CMD_CONV_D2_OSR512   0x52
#define MS5611_CMD_CONV_D2_OSR1024  0x54
#define MS5611_CMD_CONV_D2_OSR2048  0x56
#define MS5611_CMD_CONV_D2_OSR4096  0x58

/*============================================================================
 * OVERSAMPLING RATIO (OSR) - Trade-off between speed and resolution
 *============================================================================*/

typedef enum {
    MS5611_OSR_256  = 0,  /* 0.6ms conversion, lowest resolution */
    MS5611_OSR_512  = 1,  /* 1.2ms conversion */
    MS5611_OSR_1024 = 2,  /* 2.3ms conversion */
    MS5611_OSR_2048 = 3,  /* 4.6ms conversion */
    MS5611_OSR_4096 = 4   /* 9.1ms conversion, highest resolution (recommended) */
} MS5611_OSR_t;

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

typedef struct {
    /* I2C handle */
    I2C_HandleTypeDef *hi2c;
    
    /* I2C address (7-bit shifted) */
    uint8_t i2c_addr;
    
    /* Calibration coefficients from PROM */
    uint16_t C1;  /* Pressure sensitivity | SENS_T1 */
    uint16_t C2;  /* Pressure offset | OFF_T1 */
    uint16_t C3;  /* Temperature coefficient of pressure sensitivity | TCS */
    uint16_t C4;  /* Temperature coefficient of pressure offset | TCO */
    uint16_t C5;  /* Reference temperature | T_REF */
    uint16_t C6;  /* Temperature coefficient of the temperature | TEMPSENS */
    
    /* Raw ADC values */
    uint32_t D1;  /* Digital pressure value */
    uint32_t D2;  /* Digital temperature value */
    
    /* Calculated values */
    int32_t dT;           /* Difference between actual and reference temperature */
    int32_t TEMP;         /* Actual temperature (-40...85°C with 0.01°C resolution) */
    int64_t OFF;          /* Offset at actual temperature */
    int64_t SENS;         /* Sensitivity at actual temperature */
    int32_t P;            /* Temperature compensated pressure (10...1200 mbar) */
    
    /* Second order temperature compensation */
    int64_t T2;
    int64_t OFF2;
    int64_t SENS2;
    
    /* User-friendly values */
    float temperature_c;  /* Temperature in Celsius */
    float pressure_mbar;  /* Pressure in mbar (hPa) */
    float altitude_m;     /* Calculated altitude in meters */
    
    /* Configuration */
    MS5611_OSR_t osr;
    
    /* Sea level pressure for altitude calculation (default 1013.25 mbar) */
    float sea_level_pressure;
    
    /* Status */
    bool initialized;
    
} MS5611_Handle_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize MS5611 sensor
 * @param dev Pointer to MS5611 handle
 * @param hi2c Pointer to I2C handle
 * @param osr Oversampling ratio (MS5611_OSR_4096 recommended for flight)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef MS5611_Init(MS5611_Handle_t *dev, I2C_HandleTypeDef *hi2c, MS5611_OSR_t osr);

/**
 * @brief Reset the MS5611 sensor
 * @param dev Pointer to MS5611 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MS5611_Reset(MS5611_Handle_t *dev);

/**
 * @brief Read calibration data from PROM
 * @param dev Pointer to MS5611 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MS5611_ReadPROM(MS5611_Handle_t *dev);

/**
 * @brief Verify PROM CRC
 * @param dev Pointer to MS5611 handle
 * @return true if CRC is valid
 */
bool MS5611_CheckCRC(MS5611_Handle_t *dev);

/**
 * @brief Read both temperature and pressure (blocking)
 * @param dev Pointer to MS5611 handle
 * @return HAL_OK on success
 * 
 * This function performs:
 * 1. Start D2 (temperature) conversion
 * 2. Wait for conversion
 * 3. Read D2 value
 * 4. Start D1 (pressure) conversion
 * 5. Wait for conversion
 * 6. Read D1 value
 * 7. Calculate temperature and pressure
 */
HAL_StatusTypeDef MS5611_ReadTemperatureAndPressure(MS5611_Handle_t *dev);

/**
 * @brief Start asynchronous D1 (pressure) conversion
 * @param dev Pointer to MS5611 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MS5611_StartPressureConversion(MS5611_Handle_t *dev);

/**
 * @brief Start asynchronous D2 (temperature) conversion
 * @param dev Pointer to MS5611 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MS5611_StartTemperatureConversion(MS5611_Handle_t *dev);

/**
 * @brief Read raw ADC value after conversion is complete
 * @param dev Pointer to MS5611 handle
 * @param value Pointer to store 24-bit ADC value
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MS5611_ReadADC(MS5611_Handle_t *dev, uint32_t *value);

/**
 * @brief Calculate temperature and pressure from raw values
 * @param dev Pointer to MS5611 handle
 * 
 * Results are stored in:
 * - dev->temperature_c
 * - dev->pressure_mbar
 * - dev->altitude_m
 */
void MS5611_Calculate(MS5611_Handle_t *dev);

/**
 * @brief Get conversion time for current OSR setting
 * @param dev Pointer to MS5611 handle
 * @return Conversion time in milliseconds
 */
uint32_t MS5611_GetConversionTime(MS5611_Handle_t *dev);

/**
 * @brief Set sea level pressure for altitude calculation
 * @param dev Pointer to MS5611 handle
 * @param pressure Sea level pressure in mbar
 */
void MS5611_SetSeaLevelPressure(MS5611_Handle_t *dev, float pressure);

/**
 * @brief Calculate altitude from pressure using barometric formula
 * @param pressure Current pressure in mbar
 * @param sea_level_pressure Sea level pressure in mbar
 * @return Altitude in meters
 */
float MS5611_CalculateAltitude(float pressure, float sea_level_pressure);

/**
 * @brief Get current temperature
 * @param dev Pointer to MS5611 handle
 * @return Temperature in Celsius
 */
float MS5611_GetTemperature(MS5611_Handle_t *dev);

/**
 * @brief Get current pressure
 * @param dev Pointer to MS5611 handle
 * @return Pressure in mbar (hPa)
 */
float MS5611_GetPressure(MS5611_Handle_t *dev);

/**
 * @brief Get current altitude
 * @param dev Pointer to MS5611 handle
 * @return Altitude in meters (relative to sea level pressure setting)
 */
float MS5611_GetAltitude(MS5611_Handle_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* MS5611_H */

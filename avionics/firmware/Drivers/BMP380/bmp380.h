/**
 * @file bmp380.h
 * @brief Bosch BMP380 Barometric Pressure Sensor Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * BMP380 is a high-precision barometric pressure sensor:
 * - Pressure range: 300-1250 hPa
 * - Pressure resolution: 0.016 Pa (RMS noise)
 * - Relative accuracy: ±0.06 hPa (±0.5m)
 * - Absolute accuracy: ±0.5 hPa
 * - Temperature range: -40 to +85°C
 * 
 * Interface: I2C (up to 3.4MHz) or SPI (up to 10MHz)
 * Default I2C Address: 0x77 (SDO=high) or 0x76 (SDO=low)
 */

#ifndef BMP380_H
#define BMP380_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * I2C ADDRESS
 *============================================================================*/

#define BMP380_ADDR_LOW         0x76    /* SDO = GND */
#define BMP380_ADDR_HIGH        0x77    /* SDO = VDD */
#define BMP380_DEFAULT_ADDR     BMP380_ADDR_HIGH

/*============================================================================
 * REGISTER MAP
 *============================================================================*/

#define BMP380_REG_CHIP_ID      0x00
#define BMP380_REG_ERR_REG      0x02
#define BMP380_REG_STATUS       0x03
#define BMP380_REG_DATA_0       0x04    /* Pressure XLSB */
#define BMP380_REG_DATA_1       0x05    /* Pressure LSB */
#define BMP380_REG_DATA_2       0x06    /* Pressure MSB */
#define BMP380_REG_DATA_3       0x07    /* Temperature XLSB */
#define BMP380_REG_DATA_4       0x08    /* Temperature LSB */
#define BMP380_REG_DATA_5       0x09    /* Temperature MSB */
#define BMP380_REG_SENSORTIME_0 0x0C
#define BMP380_REG_SENSORTIME_1 0x0D
#define BMP380_REG_SENSORTIME_2 0x0E
#define BMP380_REG_EVENT        0x10
#define BMP380_REG_INT_STATUS   0x11
#define BMP380_REG_FIFO_LENGTH_0 0x12
#define BMP380_REG_FIFO_LENGTH_1 0x13
#define BMP380_REG_FIFO_DATA    0x14
#define BMP380_REG_FIFO_WM_0    0x15
#define BMP380_REG_FIFO_WM_1    0x16
#define BMP380_REG_FIFO_CONFIG_1 0x17
#define BMP380_REG_FIFO_CONFIG_2 0x18
#define BMP380_REG_INT_CTRL     0x19
#define BMP380_REG_IF_CONF      0x1A
#define BMP380_REG_PWR_CTRL     0x1B
#define BMP380_REG_OSR          0x1C    /* Oversampling */
#define BMP380_REG_ODR          0x1D    /* Output data rate */
#define BMP380_REG_CONFIG       0x1F    /* IIR filter */
#define BMP380_REG_CMD          0x7E

/* Calibration data registers */
#define BMP380_REG_CALIB_DATA   0x31    /* 0x31-0x45 (21 bytes) */

/*============================================================================
 * CHIP ID
 *============================================================================*/

#define BMP380_CHIP_ID          0x50
#define BMP390_CHIP_ID          0x60    /* Also compatible */

/*============================================================================
 * COMMANDS
 *============================================================================*/

#define BMP380_CMD_NOP          0x00
#define BMP380_CMD_FIFO_FLUSH   0xB0
#define BMP380_CMD_SOFT_RESET   0xB6

/*============================================================================
 * POWER CONTROL REGISTER BITS
 *============================================================================*/

#define BMP380_PRESS_EN         (1 << 0)
#define BMP380_TEMP_EN          (1 << 1)
#define BMP380_MODE_SLEEP       (0 << 4)
#define BMP380_MODE_FORCED      (1 << 4)
#define BMP380_MODE_NORMAL      (3 << 4)

/*============================================================================
 * CONFIGURATION ENUMS
 *============================================================================*/

typedef enum {
    BMP380_MODE_SLEEP_E     = 0x00,
    BMP380_MODE_FORCED_E    = 0x01,
    BMP380_MODE_NORMAL_E    = 0x03
} BMP380_Mode_t;

typedef enum {
    BMP380_OSR_x1           = 0x00,     /* No oversampling */
    BMP380_OSR_x2           = 0x01,
    BMP380_OSR_x4           = 0x02,
    BMP380_OSR_x8           = 0x03,
    BMP380_OSR_x16          = 0x04,
    BMP380_OSR_x32          = 0x05
} BMP380_Oversampling_t;

typedef enum {
    BMP380_ODR_200_HZ       = 0x00,
    BMP380_ODR_100_HZ       = 0x01,
    BMP380_ODR_50_HZ        = 0x02,
    BMP380_ODR_25_HZ        = 0x03,
    BMP380_ODR_12_5_HZ      = 0x04,
    BMP380_ODR_6_25_HZ      = 0x05,
    BMP380_ODR_3_1_HZ       = 0x06,
    BMP380_ODR_1_5_HZ       = 0x07,
    BMP380_ODR_0_78_HZ      = 0x08,
    BMP380_ODR_0_39_HZ      = 0x09,
    BMP380_ODR_0_2_HZ       = 0x0A,
    BMP380_ODR_0_1_HZ       = 0x0B,
    BMP380_ODR_0_05_HZ      = 0x0C,
    BMP380_ODR_0_02_HZ      = 0x0D,
    BMP380_ODR_0_01_HZ      = 0x0E,
    BMP380_ODR_0_006_HZ     = 0x0F,
    BMP380_ODR_0_003_HZ     = 0x10,
    BMP380_ODR_0_0015_HZ    = 0x11
} BMP380_ODR_t;

typedef enum {
    BMP380_IIR_COEF_0       = 0x00,     /* Bypass (no filter) */
    BMP380_IIR_COEF_1       = 0x01,
    BMP380_IIR_COEF_3       = 0x02,
    BMP380_IIR_COEF_7       = 0x03,
    BMP380_IIR_COEF_15      = 0x04,
    BMP380_IIR_COEF_31      = 0x05,
    BMP380_IIR_COEF_63      = 0x06,
    BMP380_IIR_COEF_127     = 0x07
} BMP380_IIR_Filter_t;

/*============================================================================
 * CALIBRATION DATA STRUCTURE
 *============================================================================*/

typedef struct {
    /* Temperature compensation */
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    
    /* Pressure compensation */
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    
    /* Intermediate value */
    float t_lin;
} BMP380_CalibData_t;

/*============================================================================
 * HANDLE STRUCTURE
 *============================================================================*/

typedef struct {
    /* I2C handle and address */
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
    
    /* Chip ID */
    uint8_t chip_id;
    
    /* Calibration data */
    BMP380_CalibData_t calib;
    
    /* Configuration */
    BMP380_Mode_t mode;
    BMP380_Oversampling_t osr_temp;
    BMP380_Oversampling_t osr_press;
    BMP380_ODR_t odr;
    BMP380_IIR_Filter_t iir_filter;
    
    /* Sensor data */
    float temperature;      /* °C */
    float pressure;         /* Pa */
    float altitude;         /* meters */
    
    /* Raw ADC values */
    uint32_t raw_temp;
    uint32_t raw_press;
    
    /* Reference pressure for altitude (default: 101325 Pa) */
    float sea_level_pressure;
    
    /* Status */
    bool temp_enabled;
    bool press_enabled;
    bool data_ready;
    
    /* Initialization flag */
    bool initialized;
    
} BMP380_Handle_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize BMP380 sensor
 * @param dev Pointer to BMP380 handle
 * @param hi2c Pointer to I2C handle
 * @param addr I2C address
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_Init(BMP380_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @brief Configure sensor settings
 * @param dev Pointer to BMP380 handle
 * @param osr_temp Temperature oversampling
 * @param osr_press Pressure oversampling
 * @param odr Output data rate
 * @param iir IIR filter coefficient
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_Configure(BMP380_Handle_t *dev,
                                   BMP380_Oversampling_t osr_temp,
                                   BMP380_Oversampling_t osr_press,
                                   BMP380_ODR_t odr,
                                   BMP380_IIR_Filter_t iir);

/**
 * @brief Set operation mode
 * @param dev Pointer to BMP380 handle
 * @param mode Operation mode
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_SetMode(BMP380_Handle_t *dev, BMP380_Mode_t mode);

/**
 * @brief Enable/disable sensors
 * @param dev Pointer to BMP380 handle
 * @param temp_en Enable temperature measurement
 * @param press_en Enable pressure measurement
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_EnableSensors(BMP380_Handle_t *dev, bool temp_en, bool press_en);

/**
 * @brief Trigger a forced measurement
 * @param dev Pointer to BMP380 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_TriggerMeasurement(BMP380_Handle_t *dev);

/**
 * @brief Check if data is ready
 * @param dev Pointer to BMP380 handle
 * @return true if new data available
 */
bool BMP380_IsDataReady(BMP380_Handle_t *dev);

/**
 * @brief Read temperature and pressure
 * @param dev Pointer to BMP380 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_ReadData(BMP380_Handle_t *dev);

/**
 * @brief Get temperature
 * @param dev Pointer to BMP380 handle
 * @return Temperature in °C
 */
float BMP380_GetTemperature(BMP380_Handle_t *dev);

/**
 * @brief Get pressure
 * @param dev Pointer to BMP380 handle
 * @return Pressure in Pa
 */
float BMP380_GetPressure(BMP380_Handle_t *dev);

/**
 * @brief Get pressure in hPa (mbar)
 * @param dev Pointer to BMP380 handle
 * @return Pressure in hPa
 */
float BMP380_GetPressureHPa(BMP380_Handle_t *dev);

/**
 * @brief Calculate altitude from pressure
 * @param dev Pointer to BMP380 handle
 * @return Altitude in meters
 */
float BMP380_GetAltitude(BMP380_Handle_t *dev);

/**
 * @brief Set sea level reference pressure
 * @param dev Pointer to BMP380 handle
 * @param pressure Sea level pressure in Pa
 */
void BMP380_SetSeaLevelPressure(BMP380_Handle_t *dev, float pressure);

/**
 * @brief Calibrate sea level pressure from known altitude
 * @param dev Pointer to BMP380 handle
 * @param known_altitude Known altitude in meters
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_CalibrateAltitude(BMP380_Handle_t *dev, float known_altitude);

/**
 * @brief Software reset
 * @param dev Pointer to BMP380 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_Reset(BMP380_Handle_t *dev);

/**
 * @brief Get error status
 * @param dev Pointer to BMP380 handle
 * @param error Pointer to store error byte
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMP380_GetError(BMP380_Handle_t *dev, uint8_t *error);

#ifdef __cplusplus
}
#endif

#endif /* BMP380_H */

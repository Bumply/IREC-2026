/**
 * @file bmp380.c
 * @brief Bosch BMP380 Barometric Pressure Sensor Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 */

#include "bmp380.h"
#include <string.h>
#include <math.h>

/*============================================================================
 * PRIVATE DEFINES
 *============================================================================*/

#define BMP380_I2C_TIMEOUT      100

/* Sea level pressure default (Pa) */
#define DEFAULT_SEA_LEVEL_PA    101325.0f

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

static HAL_StatusTypeDef BMP380_WriteReg(BMP380_Handle_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, data, 2, BMP380_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BMP380_ReadReg(BMP380_Handle_t *dev, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, &reg, 1, BMP380_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr << 1, value, 1, BMP380_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BMP380_ReadRegs(BMP380_Handle_t *dev, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, &reg, 1, BMP380_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr << 1, buffer, len, BMP380_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BMP380_ReadCalibData(BMP380_Handle_t *dev)
{
    uint8_t calib[21];
    HAL_StatusTypeDef status;
    
    status = BMP380_ReadRegs(dev, BMP380_REG_CALIB_DATA, calib, 21);
    if (status != HAL_OK) return status;
    
    /* Parse calibration data - see BMP380 datasheet section 3.11.1 */
    dev->calib.par_t1 = (uint16_t)((calib[1] << 8) | calib[0]);
    dev->calib.par_t2 = (uint16_t)((calib[3] << 8) | calib[2]);
    dev->calib.par_t3 = (int8_t)calib[4];
    
    dev->calib.par_p1 = (int16_t)((calib[6] << 8) | calib[5]);
    dev->calib.par_p2 = (int16_t)((calib[8] << 8) | calib[7]);
    dev->calib.par_p3 = (int8_t)calib[9];
    dev->calib.par_p4 = (int8_t)calib[10];
    dev->calib.par_p5 = (uint16_t)((calib[12] << 8) | calib[11]);
    dev->calib.par_p6 = (uint16_t)((calib[14] << 8) | calib[13]);
    dev->calib.par_p7 = (int8_t)calib[15];
    dev->calib.par_p8 = (int8_t)calib[16];
    dev->calib.par_p9 = (int16_t)((calib[18] << 8) | calib[17]);
    dev->calib.par_p10 = (int8_t)calib[19];
    dev->calib.par_p11 = (int8_t)calib[20];
    
    return HAL_OK;
}

static float BMP380_CompensateTemp(BMP380_Handle_t *dev, uint32_t raw_temp)
{
    float partial_data1;
    float partial_data2;
    
    partial_data1 = (float)(raw_temp - (256.0f * dev->calib.par_t1));
    partial_data2 = dev->calib.par_t2 * (1.0f / 262144.0f);
    
    dev->calib.t_lin = partial_data1 * partial_data2 + 
                       (partial_data1 * partial_data1) * 
                       (dev->calib.par_t3 * (1.0f / 1099511627776.0f));
    
    return dev->calib.t_lin;
}

static float BMP380_CompensatePress(BMP380_Handle_t *dev, uint32_t raw_press)
{
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;
    
    float t_lin = dev->calib.t_lin;
    
    partial_data1 = dev->calib.par_p6 * t_lin;
    partial_data2 = dev->calib.par_p7 * (t_lin * t_lin);
    partial_data3 = dev->calib.par_p8 * (t_lin * t_lin * t_lin);
    partial_out1 = dev->calib.par_p5 + partial_data1 + partial_data2 + partial_data3;
    
    partial_data1 = dev->calib.par_p2 * t_lin;
    partial_data2 = dev->calib.par_p3 * (t_lin * t_lin);
    partial_data3 = dev->calib.par_p4 * (t_lin * t_lin * t_lin);
    partial_out2 = (float)raw_press * (dev->calib.par_p1 + partial_data1 + 
                   partial_data2 + partial_data3);
    
    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = dev->calib.par_p9 + dev->calib.par_p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)raw_press * (float)raw_press * 
                   (float)raw_press) * dev->calib.par_p11;
    
    return partial_out1 + partial_out2 + partial_data4;
}

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

HAL_StatusTypeDef BMP380_Init(BMP380_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    HAL_StatusTypeDef status;
    
    if (dev == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }
    
    /* Clear handle */
    memset(dev, 0, sizeof(BMP380_Handle_t));
    
    dev->hi2c = hi2c;
    dev->i2c_addr = addr;
    dev->sea_level_pressure = DEFAULT_SEA_LEVEL_PA;
    
    /* Soft reset */
    status = BMP380_Reset(dev);
    if (status != HAL_OK) return status;
    
    HAL_Delay(10);
    
    /* Read chip ID */
    status = BMP380_ReadReg(dev, BMP380_REG_CHIP_ID, &dev->chip_id);
    if (status != HAL_OK) return status;
    
    if (dev->chip_id != BMP380_CHIP_ID && dev->chip_id != BMP390_CHIP_ID) {
        return HAL_ERROR;
    }
    
    /* Read calibration data */
    status = BMP380_ReadCalibData(dev);
    if (status != HAL_OK) return status;
    
    /* Configure default settings:
     * - Temperature oversampling: x2
     * - Pressure oversampling: x8
     * - ODR: 50 Hz
     * - IIR filter: coef 3
     */
    status = BMP380_Configure(dev, BMP380_OSR_x2, BMP380_OSR_x8, 
                              BMP380_ODR_50_HZ, BMP380_IIR_COEF_3);
    if (status != HAL_OK) return status;
    
    /* Enable temperature and pressure */
    status = BMP380_EnableSensors(dev, true, true);
    if (status != HAL_OK) return status;
    
    /* Set normal mode */
    status = BMP380_SetMode(dev, BMP380_MODE_NORMAL_E);
    if (status != HAL_OK) return status;
    
    dev->initialized = true;
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP380_Configure(BMP380_Handle_t *dev,
                                   BMP380_Oversampling_t osr_temp,
                                   BMP380_Oversampling_t osr_press,
                                   BMP380_ODR_t odr,
                                   BMP380_IIR_Filter_t iir)
{
    HAL_StatusTypeDef status;
    
    if (dev == NULL) return HAL_ERROR;
    
    /* Set oversampling (OSR register) */
    uint8_t osr_reg = ((osr_temp & 0x07) << 3) | (osr_press & 0x07);
    status = BMP380_WriteReg(dev, BMP380_REG_OSR, osr_reg);
    if (status != HAL_OK) return status;
    
    /* Set output data rate (ODR register) */
    status = BMP380_WriteReg(dev, BMP380_REG_ODR, odr);
    if (status != HAL_OK) return status;
    
    /* Set IIR filter (CONFIG register) */
    uint8_t config_reg = (iir & 0x07) << 1;
    status = BMP380_WriteReg(dev, BMP380_REG_CONFIG, config_reg);
    if (status != HAL_OK) return status;
    
    dev->osr_temp = osr_temp;
    dev->osr_press = osr_press;
    dev->odr = odr;
    dev->iir_filter = iir;
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP380_SetMode(BMP380_Handle_t *dev, BMP380_Mode_t mode)
{
    HAL_StatusTypeDef status;
    uint8_t pwr_ctrl;
    
    if (dev == NULL) return HAL_ERROR;
    
    /* Read current power control register */
    status = BMP380_ReadReg(dev, BMP380_REG_PWR_CTRL, &pwr_ctrl);
    if (status != HAL_OK) return status;
    
    /* Clear mode bits and set new mode */
    pwr_ctrl = (pwr_ctrl & 0xCF) | ((mode & 0x03) << 4);
    
    status = BMP380_WriteReg(dev, BMP380_REG_PWR_CTRL, pwr_ctrl);
    if (status != HAL_OK) return status;
    
    dev->mode = mode;
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP380_EnableSensors(BMP380_Handle_t *dev, bool temp_en, bool press_en)
{
    HAL_StatusTypeDef status;
    uint8_t pwr_ctrl;
    
    if (dev == NULL) return HAL_ERROR;
    
    /* Read current power control register */
    status = BMP380_ReadReg(dev, BMP380_REG_PWR_CTRL, &pwr_ctrl);
    if (status != HAL_OK) return status;
    
    /* Set sensor enable bits */
    pwr_ctrl = (pwr_ctrl & 0xFC);
    if (press_en) pwr_ctrl |= BMP380_PRESS_EN;
    if (temp_en) pwr_ctrl |= BMP380_TEMP_EN;
    
    status = BMP380_WriteReg(dev, BMP380_REG_PWR_CTRL, pwr_ctrl);
    if (status != HAL_OK) return status;
    
    dev->temp_enabled = temp_en;
    dev->press_enabled = press_en;
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP380_TriggerMeasurement(BMP380_Handle_t *dev)
{
    if (dev == NULL) return HAL_ERROR;
    return BMP380_SetMode(dev, BMP380_MODE_FORCED_E);
}

bool BMP380_IsDataReady(BMP380_Handle_t *dev)
{
    uint8_t status_reg;
    
    if (dev == NULL || !dev->initialized) return false;
    
    if (BMP380_ReadReg(dev, BMP380_REG_STATUS, &status_reg) != HAL_OK) {
        return false;
    }
    
    /* Check drdy_press and drdy_temp bits */
    return ((status_reg & 0x60) == 0x60);
}

HAL_StatusTypeDef BMP380_ReadData(BMP380_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t data[6];
    
    if (dev == NULL || !dev->initialized) return HAL_ERROR;
    
    /* Read pressure and temperature data (6 bytes) */
    status = BMP380_ReadRegs(dev, BMP380_REG_DATA_0, data, 6);
    if (status != HAL_OK) return status;
    
    /* Parse raw values (24-bit unsigned) */
    dev->raw_press = (uint32_t)data[0] | 
                     ((uint32_t)data[1] << 8) | 
                     ((uint32_t)data[2] << 16);
    
    dev->raw_temp = (uint32_t)data[3] | 
                    ((uint32_t)data[4] << 8) | 
                    ((uint32_t)data[5] << 16);
    
    /* Compensate temperature first (needed for pressure) */
    dev->temperature = BMP380_CompensateTemp(dev, dev->raw_temp);
    
    /* Compensate pressure */
    dev->pressure = BMP380_CompensatePress(dev, dev->raw_press);
    
    /* Calculate altitude */
    dev->altitude = BMP380_GetAltitude(dev);
    
    dev->data_ready = true;
    
    return HAL_OK;
}

float BMP380_GetTemperature(BMP380_Handle_t *dev)
{
    if (dev == NULL) return 0.0f;
    return dev->temperature;
}

float BMP380_GetPressure(BMP380_Handle_t *dev)
{
    if (dev == NULL) return 0.0f;
    return dev->pressure;
}

float BMP380_GetPressureHPa(BMP380_Handle_t *dev)
{
    if (dev == NULL) return 0.0f;
    return dev->pressure / 100.0f;
}

float BMP380_GetAltitude(BMP380_Handle_t *dev)
{
    if (dev == NULL || dev->pressure == 0) return 0.0f;
    
    /* Barometric formula:
     * altitude = 44330 * (1 - (P/P0)^0.1903)
     */
    float pressure_ratio = dev->pressure / dev->sea_level_pressure;
    float altitude = 44330.0f * (1.0f - powf(pressure_ratio, 0.1903f));
    
    return altitude;
}

void BMP380_SetSeaLevelPressure(BMP380_Handle_t *dev, float pressure)
{
    if (dev == NULL) return;
    dev->sea_level_pressure = pressure;
}

HAL_StatusTypeDef BMP380_CalibrateAltitude(BMP380_Handle_t *dev, float known_altitude)
{
    if (dev == NULL || !dev->initialized) return HAL_ERROR;
    
    /* Read current pressure */
    HAL_StatusTypeDef status = BMP380_ReadData(dev);
    if (status != HAL_OK) return status;
    
    /* Calculate sea level pressure from known altitude:
     * P0 = P / (1 - altitude/44330)^5.255
     */
    float factor = 1.0f - (known_altitude / 44330.0f);
    dev->sea_level_pressure = dev->pressure / powf(factor, 5.255f);
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP380_Reset(BMP380_Handle_t *dev)
{
    if (dev == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status = BMP380_WriteReg(dev, BMP380_REG_CMD, BMP380_CMD_SOFT_RESET);
    if (status != HAL_OK) return status;
    
    HAL_Delay(5);
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP380_GetError(BMP380_Handle_t *dev, uint8_t *error)
{
    if (dev == NULL || error == NULL) return HAL_ERROR;
    return BMP380_ReadReg(dev, BMP380_REG_ERR_REG, error);
}

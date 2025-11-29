/**
 * @file mpu9250.c
 * @brief MPU-9250 9-DOF IMU Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * Implementation based on MPU-9250 Register Map and Descriptions (RM-MPU-9250A-00)
 * and AK8963 datasheet for magnetometer
 */

#include "mpu9250.h"
#include <math.h>
#include <string.h>

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

static HAL_StatusTypeDef MPU9250_WriteReg(MPU9250_Handle_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return HAL_I2C_Master_Transmit(dev->hi2c, MPU9250_I2C_ADDR, buf, 2, MPU9250_TIMEOUT);
}

static HAL_StatusTypeDef MPU9250_ReadReg(MPU9250_Handle_t *dev, uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, MPU9250_I2C_ADDR, &reg, 1, MPU9250_TIMEOUT);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, MPU9250_I2C_ADDR, data, 1, MPU9250_TIMEOUT);
}

static HAL_StatusTypeDef MPU9250_ReadRegs(MPU9250_Handle_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, MPU9250_I2C_ADDR, &reg, 1, MPU9250_TIMEOUT);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, MPU9250_I2C_ADDR, data, len, MPU9250_TIMEOUT);
}

/* Write to AK8963 magnetometer via MPU9250's auxiliary I2C */
static HAL_StatusTypeDef AK8963_WriteReg(MPU9250_Handle_t *dev, uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    
    /* Set slave 0 address for write */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_ADDR, 0x0C);  /* AK8963 address, write */
    if (status != HAL_OK) return status;
    
    /* Set register address */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_REG, reg);
    if (status != HAL_OK) return status;
    
    /* Set data to write */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_DO, data);
    if (status != HAL_OK) return status;
    
    /* Enable slave 0, 1 byte */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_CTRL, 0x81);
    if (status != HAL_OK) return status;
    
    HAL_Delay(1);
    return HAL_OK;
}

/* Read from AK8963 via MPU9250's auxiliary I2C */
static HAL_StatusTypeDef AK8963_ReadRegs(MPU9250_Handle_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    
    /* Set slave 0 address for read */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_ADDR, 0x0C | 0x80);  /* AK8963 address, read */
    if (status != HAL_OK) return status;
    
    /* Set register address */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_REG, reg);
    if (status != HAL_OK) return status;
    
    /* Enable slave 0, read len bytes */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_CTRL, 0x80 | len);
    if (status != HAL_OK) return status;
    
    HAL_Delay(1);
    
    /* Read data from external sensor data registers */
    return MPU9250_ReadRegs(dev, MPU9250_REG_EXT_SENS_DATA_00, data, len);
}

static void MPU9250_UpdateScaleFactors(MPU9250_Handle_t *dev)
{
    /* Accelerometer scale factor: LSB to g */
    switch (dev->accel_fs) {
        case MPU9250_ACCEL_FS_2G:  dev->accel_scale = 2.0f / 32768.0f;  break;
        case MPU9250_ACCEL_FS_4G:  dev->accel_scale = 4.0f / 32768.0f;  break;
        case MPU9250_ACCEL_FS_8G:  dev->accel_scale = 8.0f / 32768.0f;  break;
        case MPU9250_ACCEL_FS_16G: dev->accel_scale = 16.0f / 32768.0f; break;
        default: dev->accel_scale = 16.0f / 32768.0f; break;
    }
    
    /* Gyroscope scale factor: LSB to deg/s */
    switch (dev->gyro_fs) {
        case MPU9250_GYRO_FS_250DPS:  dev->gyro_scale = 250.0f / 32768.0f;  break;
        case MPU9250_GYRO_FS_500DPS:  dev->gyro_scale = 500.0f / 32768.0f;  break;
        case MPU9250_GYRO_FS_1000DPS: dev->gyro_scale = 1000.0f / 32768.0f; break;
        case MPU9250_GYRO_FS_2000DPS: dev->gyro_scale = 2000.0f / 32768.0f; break;
        default: dev->gyro_scale = 2000.0f / 32768.0f; break;
    }
    
    /* Magnetometer scale factor: 16-bit, 4912 uT full scale */
    dev->mag_scale = 4912.0f / 32768.0f;
}

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

HAL_StatusTypeDef MPU9250_Init(MPU9250_Handle_t *dev, I2C_HandleTypeDef *hi2c,
                                MPU9250_AccelFS_t accel_fs, MPU9250_GyroFS_t gyro_fs)
{
    HAL_StatusTypeDef status;
    uint8_t data;
    
    /* Initialize handle */
    memset(dev, 0, sizeof(MPU9250_Handle_t));
    dev->hi2c = hi2c;
    dev->accel_fs = accel_fs;
    dev->gyro_fs = gyro_fs;
    dev->dlpf = MPU9250_DLPF_BW_41HZ;  /* Default */
    dev->sample_rate_div = 0;          /* 1 kHz sample rate */
    
    /* Set default magnetometer scale factors */
    dev->mag_scale_factor.x = 1.0f;
    dev->mag_scale_factor.y = 1.0f;
    dev->mag_scale_factor.z = 1.0f;
    
    /* Check device ID */
    status = MPU9250_ReadReg(dev, MPU9250_REG_WHO_AM_I, &data);
    if (status != HAL_OK) return status;
    
    if (data != MPU9250_WHO_AM_I_VALUE && data != MPU9255_WHO_AM_I_VALUE) {
        return HAL_ERROR;  /* Device not found */
    }
    dev->device_id = data;
    
    /* Reset device */
    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_1, 0x80);
    if (status != HAL_OK) return status;
    HAL_Delay(100);
    
    /* Wake up and select best clock source (PLL with X axis gyro reference) */
    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_1, 0x01);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    /* Configure accelerometer */
    status = MPU9250_WriteReg(dev, MPU9250_REG_ACCEL_CONFIG, accel_fs);
    if (status != HAL_OK) return status;
    
    /* Configure accelerometer low-pass filter */
    status = MPU9250_WriteReg(dev, MPU9250_REG_ACCEL_CONFIG2, 0x03);  /* DLPF 41Hz */
    if (status != HAL_OK) return status;
    
    /* Configure gyroscope */
    status = MPU9250_WriteReg(dev, MPU9250_REG_GYRO_CONFIG, gyro_fs);
    if (status != HAL_OK) return status;
    
    /* Configure DLPF */
    status = MPU9250_WriteReg(dev, MPU9250_REG_CONFIG, dev->dlpf);
    if (status != HAL_OK) return status;
    
    /* Set sample rate divider (1 kHz / (1 + div)) */
    status = MPU9250_WriteReg(dev, MPU9250_REG_SMPLRT_DIV, dev->sample_rate_div);
    if (status != HAL_OK) return status;
    
    /* Enable I2C master mode for magnetometer access */
    status = MPU9250_WriteReg(dev, MPU9250_REG_USER_CTRL, 0x20);
    if (status != HAL_OK) return status;
    
    /* Set I2C master clock to 400 kHz */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_MST_CTRL, 0x0D);
    if (status != HAL_OK) return status;
    
    /* Update scale factors */
    MPU9250_UpdateScaleFactors(dev);
    
    dev->initialized = true;
    return HAL_OK;
}

bool MPU9250_IsConnected(MPU9250_Handle_t *dev)
{
    uint8_t data;
    if (MPU9250_ReadReg(dev, MPU9250_REG_WHO_AM_I, &data) != HAL_OK) {
        return false;
    }
    return (data == MPU9250_WHO_AM_I_VALUE || data == MPU9255_WHO_AM_I_VALUE);
}

HAL_StatusTypeDef MPU9250_InitMagnetometer(MPU9250_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t data[3];
    
    /* Reset magnetometer */
    status = AK8963_WriteReg(dev, AK8963_REG_CNTL2, 0x01);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    /* Enter Fuse ROM access mode to read sensitivity adjustment values */
    status = AK8963_WriteReg(dev, AK8963_REG_CNTL1, AK8963_MODE_FUSEROM);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    /* Read sensitivity adjustment values */
    status = AK8963_ReadRegs(dev, AK8963_REG_ASAX, data, 3);
    if (status != HAL_OK) return status;
    
    /* Calculate sensitivity adjustment factors */
    /* Formula: Hadj = H * ((ASA - 128) * 0.5 / 128 + 1) */
    dev->mag_asa[0] = (float)(data[0] - 128) / 256.0f + 1.0f;
    dev->mag_asa[1] = (float)(data[1] - 128) / 256.0f + 1.0f;
    dev->mag_asa[2] = (float)(data[2] - 128) / 256.0f + 1.0f;
    
    /* Power down before changing mode */
    status = AK8963_WriteReg(dev, AK8963_REG_CNTL1, AK8963_MODE_POWERDOWN);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    /* Set to continuous measurement mode 2 (100 Hz), 16-bit output */
    status = AK8963_WriteReg(dev, AK8963_REG_CNTL1, AK8963_MODE_CONT_100HZ | AK8963_BIT_16);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    /* Configure slave 0 for continuous magnetometer reading */
    /* This allows reading mag data along with accel/gyro in one burst */
    
    /* Set slave 0 to read AK8963 starting from ST1 (7 bytes: ST1 + 6 data + ST2) */
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_ADDR, 0x0C | 0x80);
    if (status != HAL_OK) return status;
    
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_ST1);
    if (status != HAL_OK) return status;
    
    status = MPU9250_WriteReg(dev, MPU9250_REG_I2C_SLV0_CTRL, 0x87);  /* Enable, read 7 bytes */
    if (status != HAL_OK) return status;
    
    dev->mag_enabled = true;
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadAll(MPU9250_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t buf[21];  /* 6 accel + 2 temp + 6 gyro + 7 mag */
    
    /* Read all data in one burst (accel, temp, gyro) */
    status = MPU9250_ReadRegs(dev, MPU9250_REG_ACCEL_XOUT_H, buf, 14);
    if (status != HAL_OK) return status;
    
    /* Parse accelerometer data (big-endian) */
    dev->accel_raw.x = (int16_t)((buf[0] << 8) | buf[1]);
    dev->accel_raw.y = (int16_t)((buf[2] << 8) | buf[3]);
    dev->accel_raw.z = (int16_t)((buf[4] << 8) | buf[5]);
    
    /* Parse temperature data */
    dev->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    
    /* Parse gyroscope data */
    dev->gyro_raw.x = (int16_t)((buf[8] << 8) | buf[9]);
    dev->gyro_raw.y = (int16_t)((buf[10] << 8) | buf[11]);
    dev->gyro_raw.z = (int16_t)((buf[12] << 8) | buf[13]);
    
    /* Read magnetometer data from external sensor registers (if enabled) */
    if (dev->mag_enabled) {
        status = MPU9250_ReadRegs(dev, MPU9250_REG_EXT_SENS_DATA_00, buf, 7);
        if (status == HAL_OK && (buf[0] & 0x01)) {  /* Check data ready bit */
            /* Magnetometer data is little-endian! */
            dev->mag_raw.x = (int16_t)((buf[2] << 8) | buf[1]);
            dev->mag_raw.y = (int16_t)((buf[4] << 8) | buf[3]);
            dev->mag_raw.z = (int16_t)((buf[6] << 8) | buf[5]);
        }
    }
    
    /* Convert to physical units */
    dev->accel.x = (float)dev->accel_raw.x * dev->accel_scale - dev->accel_offset.x;
    dev->accel.y = (float)dev->accel_raw.y * dev->accel_scale - dev->accel_offset.y;
    dev->accel.z = (float)dev->accel_raw.z * dev->accel_scale - dev->accel_offset.z;
    
    dev->gyro.x = (float)dev->gyro_raw.x * dev->gyro_scale - dev->gyro_offset.x;
    dev->gyro.y = (float)dev->gyro_raw.y * dev->gyro_scale - dev->gyro_offset.y;
    dev->gyro.z = (float)dev->gyro_raw.z * dev->gyro_scale - dev->gyro_offset.z;
    
    if (dev->mag_enabled) {
        /* Apply sensitivity adjustment and scale */
        dev->mag.x = ((float)dev->mag_raw.x * dev->mag_scale * dev->mag_asa[0] - dev->mag_offset.x) * dev->mag_scale_factor.x;
        dev->mag.y = ((float)dev->mag_raw.y * dev->mag_scale * dev->mag_asa[1] - dev->mag_offset.y) * dev->mag_scale_factor.y;
        dev->mag.z = ((float)dev->mag_raw.z * dev->mag_scale * dev->mag_asa[2] - dev->mag_offset.z) * dev->mag_scale_factor.z;
    }
    
    /* Temperature in Celsius: TEMP_degC = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21 */
    dev->temperature = ((float)dev->temp_raw / 333.87f) + 21.0f;
    
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadAccel(MPU9250_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t buf[6];
    
    status = MPU9250_ReadRegs(dev, MPU9250_REG_ACCEL_XOUT_H, buf, 6);
    if (status != HAL_OK) return status;
    
    dev->accel_raw.x = (int16_t)((buf[0] << 8) | buf[1]);
    dev->accel_raw.y = (int16_t)((buf[2] << 8) | buf[3]);
    dev->accel_raw.z = (int16_t)((buf[4] << 8) | buf[5]);
    
    dev->accel.x = (float)dev->accel_raw.x * dev->accel_scale - dev->accel_offset.x;
    dev->accel.y = (float)dev->accel_raw.y * dev->accel_scale - dev->accel_offset.y;
    dev->accel.z = (float)dev->accel_raw.z * dev->accel_scale - dev->accel_offset.z;
    
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadGyro(MPU9250_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t buf[6];
    
    status = MPU9250_ReadRegs(dev, MPU9250_REG_GYRO_XOUT_H, buf, 6);
    if (status != HAL_OK) return status;
    
    dev->gyro_raw.x = (int16_t)((buf[0] << 8) | buf[1]);
    dev->gyro_raw.y = (int16_t)((buf[2] << 8) | buf[3]);
    dev->gyro_raw.z = (int16_t)((buf[4] << 8) | buf[5]);
    
    dev->gyro.x = (float)dev->gyro_raw.x * dev->gyro_scale - dev->gyro_offset.x;
    dev->gyro.y = (float)dev->gyro_raw.y * dev->gyro_scale - dev->gyro_offset.y;
    dev->gyro.z = (float)dev->gyro_raw.z * dev->gyro_scale - dev->gyro_offset.z;
    
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadMag(MPU9250_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t buf[7];
    
    if (!dev->mag_enabled) return HAL_ERROR;
    
    status = MPU9250_ReadRegs(dev, MPU9250_REG_EXT_SENS_DATA_00, buf, 7);
    if (status != HAL_OK) return status;
    
    if (!(buf[0] & 0x01)) {
        return HAL_BUSY;  /* Data not ready */
    }
    
    /* Little-endian */
    dev->mag_raw.x = (int16_t)((buf[2] << 8) | buf[1]);
    dev->mag_raw.y = (int16_t)((buf[4] << 8) | buf[3]);
    dev->mag_raw.z = (int16_t)((buf[6] << 8) | buf[5]);
    
    dev->mag.x = ((float)dev->mag_raw.x * dev->mag_scale * dev->mag_asa[0] - dev->mag_offset.x) * dev->mag_scale_factor.x;
    dev->mag.y = ((float)dev->mag_raw.y * dev->mag_scale * dev->mag_asa[1] - dev->mag_offset.y) * dev->mag_scale_factor.y;
    dev->mag.z = ((float)dev->mag_raw.z * dev->mag_scale * dev->mag_asa[2] - dev->mag_offset.z) * dev->mag_scale_factor.z;
    
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadTemperature(MPU9250_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t buf[2];
    
    status = MPU9250_ReadRegs(dev, MPU9250_REG_TEMP_OUT_H, buf, 2);
    if (status != HAL_OK) return status;
    
    dev->temp_raw = (int16_t)((buf[0] << 8) | buf[1]);
    dev->temperature = ((float)dev->temp_raw / 333.87f) + 21.0f;
    
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_SetAccelFS(MPU9250_Handle_t *dev, MPU9250_AccelFS_t fs)
{
    HAL_StatusTypeDef status;
    
    status = MPU9250_WriteReg(dev, MPU9250_REG_ACCEL_CONFIG, fs);
    if (status != HAL_OK) return status;
    
    dev->accel_fs = fs;
    MPU9250_UpdateScaleFactors(dev);
    
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_SetGyroFS(MPU9250_Handle_t *dev, MPU9250_GyroFS_t fs)
{
    HAL_StatusTypeDef status;
    
    status = MPU9250_WriteReg(dev, MPU9250_REG_GYRO_CONFIG, fs);
    if (status != HAL_OK) return status;
    
    dev->gyro_fs = fs;
    MPU9250_UpdateScaleFactors(dev);
    
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_SetDLPF(MPU9250_Handle_t *dev, MPU9250_DLPF_t dlpf)
{
    dev->dlpf = dlpf;
    return MPU9250_WriteReg(dev, MPU9250_REG_CONFIG, dlpf);
}

HAL_StatusTypeDef MPU9250_SetSampleRateDivider(MPU9250_Handle_t *dev, uint8_t div)
{
    dev->sample_rate_div = div;
    return MPU9250_WriteReg(dev, MPU9250_REG_SMPLRT_DIV, div);
}

HAL_StatusTypeDef MPU9250_CalibrateGyro(MPU9250_Handle_t *dev, uint16_t samples)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    /* Clear existing offset */
    dev->gyro_offset.x = 0;
    dev->gyro_offset.y = 0;
    dev->gyro_offset.z = 0;
    
    /* Collect samples */
    for (uint16_t i = 0; i < samples; i++) {
        if (MPU9250_ReadGyro(dev) != HAL_OK) {
            return HAL_ERROR;
        }
        sum_x += dev->gyro.x;
        sum_y += dev->gyro.y;
        sum_z += dev->gyro.z;
        HAL_Delay(2);
    }
    
    /* Calculate average offset */
    dev->gyro_offset.x = sum_x / samples;
    dev->gyro_offset.y = sum_y / samples;
    dev->gyro_offset.z = sum_z / samples;
    
    return HAL_OK;
}

void MPU9250_SetGyroOffset(MPU9250_Handle_t *dev, MPU9250_Vector3f_t offset)
{
    dev->gyro_offset = offset;
}

void MPU9250_SetAccelOffset(MPU9250_Handle_t *dev, MPU9250_Vector3f_t offset)
{
    dev->accel_offset = offset;
}

MPU9250_Vector3f_t MPU9250_GetAccel(MPU9250_Handle_t *dev)
{
    return dev->accel;
}

MPU9250_Vector3f_t MPU9250_GetGyro(MPU9250_Handle_t *dev)
{
    return dev->gyro;
}

MPU9250_Vector3f_t MPU9250_GetMag(MPU9250_Handle_t *dev)
{
    return dev->mag;
}

float MPU9250_GetTemperature(MPU9250_Handle_t *dev)
{
    return dev->temperature;
}

float MPU9250_GetAccelMagnitude(MPU9250_Handle_t *dev)
{
    return sqrtf(dev->accel.x * dev->accel.x + 
                 dev->accel.y * dev->accel.y + 
                 dev->accel.z * dev->accel.z);
}

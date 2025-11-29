/**
 * @file mpu9250.h
 * @brief MPU-9250 9-DOF IMU Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * MPU-9250 contains:
 * - 3-axis gyroscope (±250, ±500, ±1000, ±2000 °/s)
 * - 3-axis accelerometer (±2g, ±4g, ±8g, ±16g)
 * - 3-axis magnetometer (AK8963)
 * 
 * For rocket flight, we use ±16g accelerometer range
 */

#ifndef MPU9250_H
#define MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * I2C ADDRESSES
 *============================================================================*/

/* MPU-9250 I2C address: AD0 pin LOW = 0x68, AD0 pin HIGH = 0x69 */
#define MPU9250_I2C_ADDR_AD0_LOW    (0x68 << 1)
#define MPU9250_I2C_ADDR_AD0_HIGH   (0x69 << 1)
#define MPU9250_I2C_ADDR            MPU9250_I2C_ADDR_AD0_LOW

/* AK8963 Magnetometer I2C address (accessed via MPU9250 aux I2C) */
#define AK8963_I2C_ADDR             (0x0C << 1)

/* Timeout for I2C operations (ms) */
#define MPU9250_TIMEOUT             100

/*============================================================================
 * MPU-9250 REGISTER MAP
 *============================================================================*/

#define MPU9250_REG_SELF_TEST_X_GYRO    0x00
#define MPU9250_REG_SELF_TEST_Y_GYRO    0x01
#define MPU9250_REG_SELF_TEST_Z_GYRO    0x02
#define MPU9250_REG_SELF_TEST_X_ACCEL   0x0D
#define MPU9250_REG_SELF_TEST_Y_ACCEL   0x0E
#define MPU9250_REG_SELF_TEST_Z_ACCEL   0x0F
#define MPU9250_REG_XG_OFFSET_H         0x13
#define MPU9250_REG_XG_OFFSET_L         0x14
#define MPU9250_REG_YG_OFFSET_H         0x15
#define MPU9250_REG_YG_OFFSET_L         0x16
#define MPU9250_REG_ZG_OFFSET_H         0x17
#define MPU9250_REG_ZG_OFFSET_L         0x18
#define MPU9250_REG_SMPLRT_DIV          0x19
#define MPU9250_REG_CONFIG              0x1A
#define MPU9250_REG_GYRO_CONFIG         0x1B
#define MPU9250_REG_ACCEL_CONFIG        0x1C
#define MPU9250_REG_ACCEL_CONFIG2       0x1D
#define MPU9250_REG_LP_ACCEL_ODR        0x1E
#define MPU9250_REG_WOM_THR             0x1F
#define MPU9250_REG_FIFO_EN             0x23
#define MPU9250_REG_I2C_MST_CTRL        0x24
#define MPU9250_REG_I2C_SLV0_ADDR       0x25
#define MPU9250_REG_I2C_SLV0_REG        0x26
#define MPU9250_REG_I2C_SLV0_CTRL       0x27
#define MPU9250_REG_I2C_SLV1_ADDR       0x28
#define MPU9250_REG_I2C_SLV1_REG        0x29
#define MPU9250_REG_I2C_SLV1_CTRL       0x2A
#define MPU9250_REG_I2C_SLV2_ADDR       0x2B
#define MPU9250_REG_I2C_SLV2_REG        0x2C
#define MPU9250_REG_I2C_SLV2_CTRL       0x2D
#define MPU9250_REG_I2C_SLV3_ADDR       0x2E
#define MPU9250_REG_I2C_SLV3_REG        0x2F
#define MPU9250_REG_I2C_SLV3_CTRL       0x30
#define MPU9250_REG_I2C_SLV4_ADDR       0x31
#define MPU9250_REG_I2C_SLV4_REG        0x32
#define MPU9250_REG_I2C_SLV4_DO         0x33
#define MPU9250_REG_I2C_SLV4_CTRL       0x34
#define MPU9250_REG_I2C_SLV4_DI         0x35
#define MPU9250_REG_I2C_MST_STATUS      0x36
#define MPU9250_REG_INT_PIN_CFG         0x37
#define MPU9250_REG_INT_ENABLE          0x38
#define MPU9250_REG_INT_STATUS          0x3A
#define MPU9250_REG_ACCEL_XOUT_H        0x3B
#define MPU9250_REG_ACCEL_XOUT_L        0x3C
#define MPU9250_REG_ACCEL_YOUT_H        0x3D
#define MPU9250_REG_ACCEL_YOUT_L        0x3E
#define MPU9250_REG_ACCEL_ZOUT_H        0x3F
#define MPU9250_REG_ACCEL_ZOUT_L        0x40
#define MPU9250_REG_TEMP_OUT_H          0x41
#define MPU9250_REG_TEMP_OUT_L          0x42
#define MPU9250_REG_GYRO_XOUT_H         0x43
#define MPU9250_REG_GYRO_XOUT_L         0x44
#define MPU9250_REG_GYRO_YOUT_H         0x45
#define MPU9250_REG_GYRO_YOUT_L         0x46
#define MPU9250_REG_GYRO_ZOUT_H         0x47
#define MPU9250_REG_GYRO_ZOUT_L         0x48
#define MPU9250_REG_EXT_SENS_DATA_00    0x49
#define MPU9250_REG_I2C_SLV0_DO         0x63
#define MPU9250_REG_I2C_SLV1_DO         0x64
#define MPU9250_REG_I2C_SLV2_DO         0x65
#define MPU9250_REG_I2C_SLV3_DO         0x66
#define MPU9250_REG_I2C_MST_DELAY_CTRL  0x67
#define MPU9250_REG_SIGNAL_PATH_RESET   0x68
#define MPU9250_REG_MOT_DETECT_CTRL     0x69
#define MPU9250_REG_USER_CTRL           0x6A
#define MPU9250_REG_PWR_MGMT_1          0x6B
#define MPU9250_REG_PWR_MGMT_2          0x6C
#define MPU9250_REG_FIFO_COUNTH         0x72
#define MPU9250_REG_FIFO_COUNTL         0x73
#define MPU9250_REG_FIFO_R_W            0x74
#define MPU9250_REG_WHO_AM_I            0x75
#define MPU9250_REG_XA_OFFSET_H         0x77
#define MPU9250_REG_XA_OFFSET_L         0x78
#define MPU9250_REG_YA_OFFSET_H         0x7A
#define MPU9250_REG_YA_OFFSET_L         0x7B
#define MPU9250_REG_ZA_OFFSET_H         0x7D
#define MPU9250_REG_ZA_OFFSET_L         0x7E

/* Device ID */
#define MPU9250_WHO_AM_I_VALUE          0x71
#define MPU9255_WHO_AM_I_VALUE          0x73

/*============================================================================
 * AK8963 MAGNETOMETER REGISTER MAP
 *============================================================================*/

#define AK8963_REG_WIA                  0x00  /* Device ID (should be 0x48) */
#define AK8963_REG_INFO                 0x01
#define AK8963_REG_ST1                  0x02  /* Status 1 */
#define AK8963_REG_HXL                  0x03  /* X-axis data */
#define AK8963_REG_HXH                  0x04
#define AK8963_REG_HYL                  0x05  /* Y-axis data */
#define AK8963_REG_HYH                  0x06
#define AK8963_REG_HZL                  0x07  /* Z-axis data */
#define AK8963_REG_HZH                  0x08
#define AK8963_REG_ST2                  0x09  /* Status 2 */
#define AK8963_REG_CNTL1                0x0A  /* Control 1 */
#define AK8963_REG_CNTL2                0x0B  /* Control 2 */
#define AK8963_REG_ASTC                 0x0C  /* Self-test */
#define AK8963_REG_ASAX                 0x10  /* X-axis sensitivity adjustment */
#define AK8963_REG_ASAY                 0x11
#define AK8963_REG_ASAZ                 0x12

#define AK8963_WHO_AM_I_VALUE           0x48

/*============================================================================
 * CONFIGURATION ENUMS
 *============================================================================*/

typedef enum {
    MPU9250_ACCEL_FS_2G  = 0x00,
    MPU9250_ACCEL_FS_4G  = 0x08,
    MPU9250_ACCEL_FS_8G  = 0x10,
    MPU9250_ACCEL_FS_16G = 0x18   /* Recommended for rocket flight */
} MPU9250_AccelFS_t;

typedef enum {
    MPU9250_GYRO_FS_250DPS  = 0x00,
    MPU9250_GYRO_FS_500DPS  = 0x08,
    MPU9250_GYRO_FS_1000DPS = 0x10,
    MPU9250_GYRO_FS_2000DPS = 0x18  /* Recommended for rocket flight */
} MPU9250_GyroFS_t;

typedef enum {
    MPU9250_DLPF_BW_250HZ = 0,
    MPU9250_DLPF_BW_184HZ = 1,
    MPU9250_DLPF_BW_92HZ  = 2,
    MPU9250_DLPF_BW_41HZ  = 3,
    MPU9250_DLPF_BW_20HZ  = 4,
    MPU9250_DLPF_BW_10HZ  = 5,
    MPU9250_DLPF_BW_5HZ   = 6
} MPU9250_DLPF_t;

typedef enum {
    AK8963_MODE_POWERDOWN   = 0x00,
    AK8963_MODE_SINGLE      = 0x01,
    AK8963_MODE_CONT_8HZ    = 0x02,
    AK8963_MODE_CONT_100HZ  = 0x06,  /* Recommended for flight */
    AK8963_MODE_SELFTEST    = 0x08,
    AK8963_MODE_FUSEROM     = 0x0F
} AK8963_Mode_t;

typedef enum {
    AK8963_BIT_14 = 0x00,
    AK8963_BIT_16 = 0x10   /* Recommended */
} AK8963_Resolution_t;

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

typedef struct {
    float x;
    float y;
    float z;
} MPU9250_Vector3f_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU9250_Vector3i_t;

typedef struct {
    /* I2C handle */
    I2C_HandleTypeDef *hi2c;
    
    /* Configuration */
    MPU9250_AccelFS_t accel_fs;
    MPU9250_GyroFS_t gyro_fs;
    MPU9250_DLPF_t dlpf;
    uint8_t sample_rate_div;
    
    /* Scale factors (based on full scale setting) */
    float accel_scale;   /* LSB to g */
    float gyro_scale;    /* LSB to deg/s */
    float mag_scale;     /* LSB to uT */
    
    /* Magnetometer sensitivity adjustment values */
    float mag_asa[3];
    
    /* Raw sensor data (16-bit signed) */
    MPU9250_Vector3i_t accel_raw;
    MPU9250_Vector3i_t gyro_raw;
    MPU9250_Vector3i_t mag_raw;
    int16_t temp_raw;
    
    /* Calibrated sensor data */
    MPU9250_Vector3f_t accel;      /* Acceleration in g */
    MPU9250_Vector3f_t gyro;       /* Angular rate in deg/s */
    MPU9250_Vector3f_t mag;        /* Magnetic field in uT */
    float temperature;             /* Temperature in Celsius */
    
    /* Calibration offsets (can be set after calibration) */
    MPU9250_Vector3f_t accel_offset;
    MPU9250_Vector3f_t gyro_offset;
    MPU9250_Vector3f_t mag_offset;
    MPU9250_Vector3f_t mag_scale_factor;
    
    /* Status */
    bool initialized;
    bool mag_enabled;
    uint8_t device_id;
    
} MPU9250_Handle_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize MPU-9250 sensor
 * @param dev Pointer to MPU9250 handle
 * @param hi2c Pointer to I2C handle
 * @param accel_fs Accelerometer full scale range
 * @param gyro_fs Gyroscope full scale range
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_Init(MPU9250_Handle_t *dev, I2C_HandleTypeDef *hi2c,
                                MPU9250_AccelFS_t accel_fs, MPU9250_GyroFS_t gyro_fs);

/**
 * @brief Check if MPU-9250 is connected
 * @param dev Pointer to MPU9250 handle
 * @return true if device responds with correct ID
 */
bool MPU9250_IsConnected(MPU9250_Handle_t *dev);

/**
 * @brief Initialize AK8963 magnetometer
 * @param dev Pointer to MPU9250 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_InitMagnetometer(MPU9250_Handle_t *dev);

/**
 * @brief Read all sensor data (accel, gyro, temp, mag)
 * @param dev Pointer to MPU9250 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_ReadAll(MPU9250_Handle_t *dev);

/**
 * @brief Read accelerometer data only
 * @param dev Pointer to MPU9250 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_ReadAccel(MPU9250_Handle_t *dev);

/**
 * @brief Read gyroscope data only
 * @param dev Pointer to MPU9250 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_ReadGyro(MPU9250_Handle_t *dev);

/**
 * @brief Read magnetometer data only
 * @param dev Pointer to MPU9250 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_ReadMag(MPU9250_Handle_t *dev);

/**
 * @brief Read temperature
 * @param dev Pointer to MPU9250 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_ReadTemperature(MPU9250_Handle_t *dev);

/**
 * @brief Set accelerometer full scale range
 * @param dev Pointer to MPU9250 handle
 * @param fs Full scale setting
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_SetAccelFS(MPU9250_Handle_t *dev, MPU9250_AccelFS_t fs);

/**
 * @brief Set gyroscope full scale range
 * @param dev Pointer to MPU9250 handle
 * @param fs Full scale setting
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_SetGyroFS(MPU9250_Handle_t *dev, MPU9250_GyroFS_t fs);

/**
 * @brief Set digital low-pass filter bandwidth
 * @param dev Pointer to MPU9250 handle
 * @param dlpf DLPF setting
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_SetDLPF(MPU9250_Handle_t *dev, MPU9250_DLPF_t dlpf);

/**
 * @brief Set sample rate divider
 * @param dev Pointer to MPU9250 handle
 * @param div Sample rate = Internal_Sample_Rate / (1 + div)
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_SetSampleRateDivider(MPU9250_Handle_t *dev, uint8_t div);

/**
 * @brief Calibrate gyroscope (device must be stationary)
 * @param dev Pointer to MPU9250 handle
 * @param samples Number of samples to average
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MPU9250_CalibrateGyro(MPU9250_Handle_t *dev, uint16_t samples);

/**
 * @brief Set gyroscope offset manually
 * @param dev Pointer to MPU9250 handle
 * @param offset Offset values in deg/s
 */
void MPU9250_SetGyroOffset(MPU9250_Handle_t *dev, MPU9250_Vector3f_t offset);

/**
 * @brief Set accelerometer offset manually
 * @param dev Pointer to MPU9250 handle
 * @param offset Offset values in g
 */
void MPU9250_SetAccelOffset(MPU9250_Handle_t *dev, MPU9250_Vector3f_t offset);

/**
 * @brief Get acceleration vector
 * @param dev Pointer to MPU9250 handle
 * @return Acceleration in g
 */
MPU9250_Vector3f_t MPU9250_GetAccel(MPU9250_Handle_t *dev);

/**
 * @brief Get angular rate vector
 * @param dev Pointer to MPU9250 handle
 * @return Angular rate in deg/s
 */
MPU9250_Vector3f_t MPU9250_GetGyro(MPU9250_Handle_t *dev);

/**
 * @brief Get magnetic field vector
 * @param dev Pointer to MPU9250 handle
 * @return Magnetic field in uT
 */
MPU9250_Vector3f_t MPU9250_GetMag(MPU9250_Handle_t *dev);

/**
 * @brief Get temperature
 * @param dev Pointer to MPU9250 handle
 * @return Temperature in Celsius
 */
float MPU9250_GetTemperature(MPU9250_Handle_t *dev);

/**
 * @brief Get total acceleration magnitude
 * @param dev Pointer to MPU9250 handle
 * @return |a| = sqrt(ax^2 + ay^2 + az^2) in g
 */
float MPU9250_GetAccelMagnitude(MPU9250_Handle_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* MPU9250_H */

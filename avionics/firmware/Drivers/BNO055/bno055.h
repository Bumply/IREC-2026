/**
 * @file bno055.h
 * @brief Bosch BNO055 9-DOF Absolute Orientation Sensor Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * BNO055 is a 9-DOF sensor with integrated sensor fusion:
 * - Accelerometer: ±2g/4g/8g/16g
 * - Gyroscope: ±125/250/500/1000/2000 °/s
 * - Magnetometer: ±1300/2600 µT
 * - Built-in ARM Cortex-M0 for sensor fusion
 * - Outputs: Euler angles, Quaternions, Linear acceleration, Gravity vector
 * 
 * Interface: I2C (up to 400kHz)
 * Default I2C Address: 0x28 (ADR pin low) or 0x29 (ADR pin high)
 */

#ifndef BNO055_H
#define BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * I2C ADDRESS
 *============================================================================*/

#define BNO055_ADDR_LOW         0x28    /* ADR pin = LOW */
#define BNO055_ADDR_HIGH        0x29    /* ADR pin = HIGH */
#define BNO055_DEFAULT_ADDR     BNO055_ADDR_LOW

/*============================================================================
 * REGISTER MAP - PAGE 0
 *============================================================================*/

/* Chip ID registers */
#define BNO055_REG_CHIP_ID          0x00
#define BNO055_REG_ACC_ID           0x01
#define BNO055_REG_MAG_ID           0x02
#define BNO055_REG_GYR_ID           0x03
#define BNO055_REG_SW_REV_LSB       0x04
#define BNO055_REG_SW_REV_MSB       0x05
#define BNO055_REG_BL_REV           0x06

/* Page ID */
#define BNO055_REG_PAGE_ID          0x07

/* Accelerometer data */
#define BNO055_REG_ACC_X_LSB        0x08
#define BNO055_REG_ACC_X_MSB        0x09
#define BNO055_REG_ACC_Y_LSB        0x0A
#define BNO055_REG_ACC_Y_MSB        0x0B
#define BNO055_REG_ACC_Z_LSB        0x0C
#define BNO055_REG_ACC_Z_MSB        0x0D

/* Magnetometer data */
#define BNO055_REG_MAG_X_LSB        0x0E
#define BNO055_REG_MAG_X_MSB        0x0F
#define BNO055_REG_MAG_Y_LSB        0x10
#define BNO055_REG_MAG_Y_MSB        0x11
#define BNO055_REG_MAG_Z_LSB        0x12
#define BNO055_REG_MAG_Z_MSB        0x13

/* Gyroscope data */
#define BNO055_REG_GYR_X_LSB        0x14
#define BNO055_REG_GYR_X_MSB        0x15
#define BNO055_REG_GYR_Y_LSB        0x16
#define BNO055_REG_GYR_Y_MSB        0x17
#define BNO055_REG_GYR_Z_LSB        0x18
#define BNO055_REG_GYR_Z_MSB        0x19

/* Euler angles */
#define BNO055_REG_EUL_HEADING_LSB  0x1A
#define BNO055_REG_EUL_HEADING_MSB  0x1B
#define BNO055_REG_EUL_ROLL_LSB     0x1C
#define BNO055_REG_EUL_ROLL_MSB     0x1D
#define BNO055_REG_EUL_PITCH_LSB    0x1E
#define BNO055_REG_EUL_PITCH_MSB    0x1F

/* Quaternion data */
#define BNO055_REG_QUA_W_LSB        0x20
#define BNO055_REG_QUA_W_MSB        0x21
#define BNO055_REG_QUA_X_LSB        0x22
#define BNO055_REG_QUA_X_MSB        0x23
#define BNO055_REG_QUA_Y_LSB        0x24
#define BNO055_REG_QUA_Y_MSB        0x25
#define BNO055_REG_QUA_Z_LSB        0x26
#define BNO055_REG_QUA_Z_MSB        0x27

/* Linear acceleration (acceleration minus gravity) */
#define BNO055_REG_LIA_X_LSB        0x28
#define BNO055_REG_LIA_X_MSB        0x29
#define BNO055_REG_LIA_Y_LSB        0x2A
#define BNO055_REG_LIA_Y_MSB        0x2B
#define BNO055_REG_LIA_Z_LSB        0x2C
#define BNO055_REG_LIA_Z_MSB        0x2D

/* Gravity vector */
#define BNO055_REG_GRV_X_LSB        0x2E
#define BNO055_REG_GRV_X_MSB        0x2F
#define BNO055_REG_GRV_Y_LSB        0x30
#define BNO055_REG_GRV_Y_MSB        0x31
#define BNO055_REG_GRV_Z_LSB        0x32
#define BNO055_REG_GRV_Z_MSB        0x33

/* Temperature */
#define BNO055_REG_TEMP             0x34

/* Calibration status */
#define BNO055_REG_CALIB_STAT       0x35

/* Self-test result */
#define BNO055_REG_ST_RESULT        0x36

/* Interrupt status */
#define BNO055_REG_INT_STA          0x37

/* System status and error */
#define BNO055_REG_SYS_CLK_STATUS   0x38
#define BNO055_REG_SYS_STATUS       0x39
#define BNO055_REG_SYS_ERR          0x3A

/* Unit selection */
#define BNO055_REG_UNIT_SEL         0x3B

/* Operation mode */
#define BNO055_REG_OPR_MODE         0x3D

/* Power mode */
#define BNO055_REG_PWR_MODE         0x3E

/* System trigger */
#define BNO055_REG_SYS_TRIGGER      0x3F

/* Temperature source */
#define BNO055_REG_TEMP_SOURCE      0x40

/* Axis remap */
#define BNO055_REG_AXIS_MAP_CONFIG  0x41
#define BNO055_REG_AXIS_MAP_SIGN    0x42

/* Accelerometer offset */
#define BNO055_REG_ACC_OFFSET_X_LSB 0x55
#define BNO055_REG_ACC_OFFSET_X_MSB 0x56
#define BNO055_REG_ACC_OFFSET_Y_LSB 0x57
#define BNO055_REG_ACC_OFFSET_Y_MSB 0x58
#define BNO055_REG_ACC_OFFSET_Z_LSB 0x59
#define BNO055_REG_ACC_OFFSET_Z_MSB 0x5A

/* Magnetometer offset */
#define BNO055_REG_MAG_OFFSET_X_LSB 0x5B
#define BNO055_REG_MAG_OFFSET_X_MSB 0x5C
#define BNO055_REG_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_REG_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_REG_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_REG_MAG_OFFSET_Z_MSB 0x60

/* Gyroscope offset */
#define BNO055_REG_GYR_OFFSET_X_LSB 0x61
#define BNO055_REG_GYR_OFFSET_X_MSB 0x62
#define BNO055_REG_GYR_OFFSET_Y_LSB 0x63
#define BNO055_REG_GYR_OFFSET_Y_MSB 0x64
#define BNO055_REG_GYR_OFFSET_Z_LSB 0x65
#define BNO055_REG_GYR_OFFSET_Z_MSB 0x66

/* Radius */
#define BNO055_REG_ACC_RADIUS_LSB   0x67
#define BNO055_REG_ACC_RADIUS_MSB   0x68
#define BNO055_REG_MAG_RADIUS_LSB   0x69
#define BNO055_REG_MAG_RADIUS_MSB   0x6A

/*============================================================================
 * CHIP IDs
 *============================================================================*/

#define BNO055_CHIP_ID              0xA0
#define BNO055_ACC_ID               0xFB
#define BNO055_MAG_ID               0x32
#define BNO055_GYR_ID               0x0F

/*============================================================================
 * OPERATION MODES
 *============================================================================*/

typedef enum {
    /* Config mode */
    BNO055_MODE_CONFIG          = 0x00,
    
    /* Non-fusion modes */
    BNO055_MODE_ACC_ONLY        = 0x01,
    BNO055_MODE_MAG_ONLY        = 0x02,
    BNO055_MODE_GYRO_ONLY       = 0x03,
    BNO055_MODE_ACC_MAG         = 0x04,
    BNO055_MODE_ACC_GYRO        = 0x05,
    BNO055_MODE_MAG_GYRO        = 0x06,
    BNO055_MODE_AMG             = 0x07,
    
    /* Fusion modes */
    BNO055_MODE_IMU             = 0x08,     /* Accel + Gyro relative orientation */
    BNO055_MODE_COMPASS         = 0x09,     /* Accel + Mag tilt-compensated compass */
    BNO055_MODE_M4G             = 0x0A,     /* Accel + Mag + Gyro (gyro for stability) */
    BNO055_MODE_NDOF_FMC_OFF    = 0x0B,     /* Full fusion, no magnetometer calibration */
    BNO055_MODE_NDOF            = 0x0C      /* Full 9-DOF fusion (recommended) */
} BNO055_OpMode_t;

/*============================================================================
 * POWER MODES
 *============================================================================*/

typedef enum {
    BNO055_POWER_NORMAL         = 0x00,
    BNO055_POWER_LOW            = 0x01,
    BNO055_POWER_SUSPEND        = 0x02
} BNO055_PowerMode_t;

/*============================================================================
 * SYSTEM STATUS
 *============================================================================*/

typedef enum {
    BNO055_SYS_IDLE             = 0x00,
    BNO055_SYS_ERROR            = 0x01,
    BNO055_SYS_INIT_PERIPHERALS = 0x02,
    BNO055_SYS_INIT_SYSTEM      = 0x03,
    BNO055_SYS_SELF_TEST        = 0x04,
    BNO055_SYS_FUSION_RUNNING   = 0x05,
    BNO055_SYS_NO_FUSION        = 0x06
} BNO055_SysStatus_t;

typedef enum {
    BNO055_ERR_NONE             = 0x00,
    BNO055_ERR_PERIPHERAL_INIT  = 0x01,
    BNO055_ERR_SYS_INIT         = 0x02,
    BNO055_ERR_SELF_TEST        = 0x03,
    BNO055_ERR_REG_VAL_RANGE    = 0x04,
    BNO055_ERR_REG_ADDR_RANGE   = 0x05,
    BNO055_ERR_REG_WRITE        = 0x06,
    BNO055_ERR_LOW_POWER_NA     = 0x07,
    BNO055_ERR_ACC_POWER_NA     = 0x08,
    BNO055_ERR_FUSION_CONFIG    = 0x09,
    BNO055_ERR_SENSOR_CONFIG    = 0x0A
} BNO055_SysError_t;

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

typedef struct {
    float x;
    float y;
    float z;
} BNO055_Vector3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} BNO055_Quaternion_t;

typedef struct {
    float heading;      /* Yaw: 0-360° */
    float roll;         /* Roll: -180 to +180° */
    float pitch;        /* Pitch: -90 to +90° */
} BNO055_Euler_t;

typedef struct {
    uint8_t system;     /* 0-3: 3 = fully calibrated */
    uint8_t gyro;       /* 0-3 */
    uint8_t accel;      /* 0-3 */
    uint8_t mag;        /* 0-3 */
} BNO055_CalibStatus_t;

typedef struct {
    int16_t acc_offset_x;
    int16_t acc_offset_y;
    int16_t acc_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t gyr_offset_x;
    int16_t gyr_offset_y;
    int16_t gyr_offset_z;
    int16_t acc_radius;
    int16_t mag_radius;
} BNO055_Offsets_t;

typedef struct {
    /* I2C handle and address */
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
    
    /* Current mode */
    BNO055_OpMode_t op_mode;
    
    /* Chip information */
    uint8_t chip_id;
    uint8_t sw_rev_lsb;
    uint8_t sw_rev_msb;
    uint8_t bl_rev;
    
    /* Calibration data */
    BNO055_CalibStatus_t calib_status;
    BNO055_Offsets_t offsets;
    
    /* Sensor data */
    BNO055_Vector3_t accel;         /* m/s² */
    BNO055_Vector3_t gyro;          /* °/s or rad/s */
    BNO055_Vector3_t mag;           /* µT */
    BNO055_Euler_t euler;           /* degrees */
    BNO055_Quaternion_t quaternion;
    BNO055_Vector3_t linear_accel;  /* m/s² without gravity */
    BNO055_Vector3_t gravity;       /* m/s² gravity vector */
    int8_t temperature;             /* °C */
    
    /* Status */
    BNO055_SysStatus_t sys_status;
    BNO055_SysError_t sys_error;
    
    /* Initialization flag */
    bool initialized;
    
} BNO055_Handle_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize BNO055 sensor
 * @param dev Pointer to BNO055 handle
 * @param hi2c Pointer to I2C handle
 * @param addr I2C address (BNO055_ADDR_LOW or BNO055_ADDR_HIGH)
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_Init(BNO055_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @brief Set operation mode
 * @param dev Pointer to BNO055 handle
 * @param mode Operation mode
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_SetMode(BNO055_Handle_t *dev, BNO055_OpMode_t mode);

/**
 * @brief Set power mode
 * @param dev Pointer to BNO055 handle
 * @param mode Power mode
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_SetPowerMode(BNO055_Handle_t *dev, BNO055_PowerMode_t mode);

/**
 * @brief Read all sensor data (based on current mode)
 * @param dev Pointer to BNO055 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadAll(BNO055_Handle_t *dev);

/**
 * @brief Read Euler angles (heading, roll, pitch)
 * @param dev Pointer to BNO055 handle
 * @param euler Pointer to Euler structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadEuler(BNO055_Handle_t *dev, BNO055_Euler_t *euler);

/**
 * @brief Read quaternion orientation
 * @param dev Pointer to BNO055 handle
 * @param quat Pointer to Quaternion structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadQuaternion(BNO055_Handle_t *dev, BNO055_Quaternion_t *quat);

/**
 * @brief Read accelerometer data
 * @param dev Pointer to BNO055 handle
 * @param accel Pointer to Vector3 structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadAccel(BNO055_Handle_t *dev, BNO055_Vector3_t *accel);

/**
 * @brief Read gyroscope data
 * @param dev Pointer to BNO055 handle
 * @param gyro Pointer to Vector3 structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadGyro(BNO055_Handle_t *dev, BNO055_Vector3_t *gyro);

/**
 * @brief Read magnetometer data
 * @param dev Pointer to BNO055 handle
 * @param mag Pointer to Vector3 structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadMag(BNO055_Handle_t *dev, BNO055_Vector3_t *mag);

/**
 * @brief Read linear acceleration (without gravity)
 * @param dev Pointer to BNO055 handle
 * @param lin_accel Pointer to Vector3 structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadLinearAccel(BNO055_Handle_t *dev, BNO055_Vector3_t *lin_accel);

/**
 * @brief Read gravity vector
 * @param dev Pointer to BNO055 handle
 * @param gravity Pointer to Vector3 structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadGravity(BNO055_Handle_t *dev, BNO055_Vector3_t *gravity);

/**
 * @brief Read temperature
 * @param dev Pointer to BNO055 handle
 * @param temp Pointer to temperature variable
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_ReadTemp(BNO055_Handle_t *dev, int8_t *temp);

/**
 * @brief Read calibration status
 * @param dev Pointer to BNO055 handle
 * @param status Pointer to CalibStatus structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_GetCalibStatus(BNO055_Handle_t *dev, BNO055_CalibStatus_t *status);

/**
 * @brief Check if sensor is fully calibrated
 * @param dev Pointer to BNO055 handle
 * @return true if fully calibrated
 */
bool BNO055_IsFullyCalibrated(BNO055_Handle_t *dev);

/**
 * @brief Read calibration offsets
 * @param dev Pointer to BNO055 handle
 * @param offsets Pointer to Offsets structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_GetCalibOffsets(BNO055_Handle_t *dev, BNO055_Offsets_t *offsets);

/**
 * @brief Write calibration offsets
 * @param dev Pointer to BNO055 handle
 * @param offsets Pointer to Offsets structure
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_SetCalibOffsets(BNO055_Handle_t *dev, const BNO055_Offsets_t *offsets);

/**
 * @brief Get system status
 * @param dev Pointer to BNO055 handle
 * @param status Pointer to status variable
 * @param error Pointer to error variable
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_GetSystemStatus(BNO055_Handle_t *dev, BNO055_SysStatus_t *status, BNO055_SysError_t *error);

/**
 * @brief Configure axis remapping
 * @param dev Pointer to BNO055 handle
 * @param config Axis map config value
 * @param sign Axis sign value
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_SetAxisRemap(BNO055_Handle_t *dev, uint8_t config, uint8_t sign);

/**
 * @brief Software reset
 * @param dev Pointer to BNO055 handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_Reset(BNO055_Handle_t *dev);

/**
 * @brief Run self-test
 * @param dev Pointer to BNO055 handle
 * @param result Pointer to store result (bit 0=ACC, 1=MAG, 2=GYR, 3=MCU)
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BNO055_SelfTest(BNO055_Handle_t *dev, uint8_t *result);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */

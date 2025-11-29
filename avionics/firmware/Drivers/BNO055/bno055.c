/**
 * @file bno055.c
 * @brief Bosch BNO055 9-DOF Absolute Orientation Sensor Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 */

#include "bno055.h"
#include <string.h>

/*============================================================================
 * PRIVATE DEFINES
 *============================================================================*/

#define BNO055_I2C_TIMEOUT      100

/* Scale factors (default units) */
#define BNO055_ACCEL_SCALE      100.0f      /* 1 m/s² = 100 LSB */
#define BNO055_GYRO_SCALE       16.0f       /* 1 °/s = 16 LSB */
#define BNO055_MAG_SCALE        16.0f       /* 1 µT = 16 LSB */
#define BNO055_EULER_SCALE      16.0f       /* 1° = 16 LSB */
#define BNO055_QUAT_SCALE       16384.0f    /* 1 = 2^14 LSB */

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

static HAL_StatusTypeDef BNO055_WriteReg(BNO055_Handle_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, data, 2, BNO055_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BNO055_ReadReg(BNO055_Handle_t *dev, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, &reg, 1, BNO055_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr << 1, value, 1, BNO055_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BNO055_ReadRegs(BNO055_Handle_t *dev, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, &reg, 1, BNO055_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr << 1, buffer, len, BNO055_I2C_TIMEOUT);
}

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

HAL_StatusTypeDef BNO055_Init(BNO055_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    HAL_StatusTypeDef status;
    
    if (dev == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }
    
    /* Clear handle */
    memset(dev, 0, sizeof(BNO055_Handle_t));
    
    dev->hi2c = hi2c;
    dev->i2c_addr = addr;
    
    /* Wait for sensor to boot */
    HAL_Delay(650);
    
    /* Read chip ID */
    status = BNO055_ReadReg(dev, BNO055_REG_CHIP_ID, &dev->chip_id);
    if (status != HAL_OK) return status;
    
    if (dev->chip_id != BNO055_CHIP_ID) {
        return HAL_ERROR;
    }
    
    /* Read software revision */
    status = BNO055_ReadReg(dev, BNO055_REG_SW_REV_LSB, &dev->sw_rev_lsb);
    if (status != HAL_OK) return status;
    status = BNO055_ReadReg(dev, BNO055_REG_SW_REV_MSB, &dev->sw_rev_msb);
    if (status != HAL_OK) return status;
    status = BNO055_ReadReg(dev, BNO055_REG_BL_REV, &dev->bl_rev);
    if (status != HAL_OK) return status;
    
    /* Set to config mode */
    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != HAL_OK) return status;
    HAL_Delay(25);
    
    /* Reset */
    status = BNO055_WriteReg(dev, BNO055_REG_SYS_TRIGGER, 0x20);
    if (status != HAL_OK) return status;
    HAL_Delay(650);
    
    /* Wait for chip ID to be readable again */
    uint8_t id = 0;
    uint32_t timeout = HAL_GetTick() + 1000;
    while (id != BNO055_CHIP_ID) {
        BNO055_ReadReg(dev, BNO055_REG_CHIP_ID, &id);
        if (HAL_GetTick() > timeout) {
            return HAL_TIMEOUT;
        }
        HAL_Delay(10);
    }
    
    /* Set to normal power mode */
    status = BNO055_SetPowerMode(dev, BNO055_POWER_NORMAL);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    /* Set page 0 */
    status = BNO055_WriteReg(dev, BNO055_REG_PAGE_ID, 0);
    if (status != HAL_OK) return status;
    
    /* Set output units:
     * - Acceleration: m/s²
     * - Gyro: deg/s
     * - Euler: degrees
     * - Temperature: Celsius
     */
    status = BNO055_WriteReg(dev, BNO055_REG_UNIT_SEL, 0x00);
    if (status != HAL_OK) return status;
    
    /* Set to NDOF mode (full 9-DOF fusion) */
    status = BNO055_SetMode(dev, BNO055_MODE_NDOF);
    if (status != HAL_OK) return status;
    HAL_Delay(20);
    
    dev->op_mode = BNO055_MODE_NDOF;
    dev->initialized = true;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SetMode(BNO055_Handle_t *dev, BNO055_OpMode_t mode)
{
    HAL_StatusTypeDef status;
    
    if (dev == NULL) return HAL_ERROR;
    
    status = BNO055_WriteReg(dev, BNO055_REG_OPR_MODE, (uint8_t)mode);
    if (status != HAL_OK) return status;
    
    /* Mode switch delay */
    if (mode == BNO055_MODE_CONFIG) {
        HAL_Delay(19);
    } else {
        HAL_Delay(7);
    }
    
    dev->op_mode = mode;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SetPowerMode(BNO055_Handle_t *dev, BNO055_PowerMode_t mode)
{
    if (dev == NULL) return HAL_ERROR;
    return BNO055_WriteReg(dev, BNO055_REG_PWR_MODE, (uint8_t)mode);
}

HAL_StatusTypeDef BNO055_ReadAll(BNO055_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[45];
    
    if (dev == NULL || !dev->initialized) return HAL_ERROR;
    
    /* Read all data from ACC to GRV (0x08-0x33) = 44 bytes */
    status = BNO055_ReadRegs(dev, BNO055_REG_ACC_X_LSB, buffer, 44);
    if (status != HAL_OK) return status;
    
    /* Parse accelerometer (0x08-0x0D) */
    int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);
    dev->accel.x = ax / BNO055_ACCEL_SCALE;
    dev->accel.y = ay / BNO055_ACCEL_SCALE;
    dev->accel.z = az / BNO055_ACCEL_SCALE;
    
    /* Parse magnetometer (0x0E-0x13) */
    int16_t mx = (int16_t)((buffer[7] << 8) | buffer[6]);
    int16_t my = (int16_t)((buffer[9] << 8) | buffer[8]);
    int16_t mz = (int16_t)((buffer[11] << 8) | buffer[10]);
    dev->mag.x = mx / BNO055_MAG_SCALE;
    dev->mag.y = my / BNO055_MAG_SCALE;
    dev->mag.z = mz / BNO055_MAG_SCALE;
    
    /* Parse gyroscope (0x14-0x19) */
    int16_t gx = (int16_t)((buffer[13] << 8) | buffer[12]);
    int16_t gy = (int16_t)((buffer[15] << 8) | buffer[14]);
    int16_t gz = (int16_t)((buffer[17] << 8) | buffer[16]);
    dev->gyro.x = gx / BNO055_GYRO_SCALE;
    dev->gyro.y = gy / BNO055_GYRO_SCALE;
    dev->gyro.z = gz / BNO055_GYRO_SCALE;
    
    /* Parse Euler angles (0x1A-0x1F) */
    int16_t heading = (int16_t)((buffer[19] << 8) | buffer[18]);
    int16_t roll = (int16_t)((buffer[21] << 8) | buffer[20]);
    int16_t pitch = (int16_t)((buffer[23] << 8) | buffer[22]);
    dev->euler.heading = heading / BNO055_EULER_SCALE;
    dev->euler.roll = roll / BNO055_EULER_SCALE;
    dev->euler.pitch = pitch / BNO055_EULER_SCALE;
    
    /* Parse quaternion (0x20-0x27) */
    int16_t qw = (int16_t)((buffer[25] << 8) | buffer[24]);
    int16_t qx = (int16_t)((buffer[27] << 8) | buffer[26]);
    int16_t qy = (int16_t)((buffer[29] << 8) | buffer[28]);
    int16_t qz = (int16_t)((buffer[31] << 8) | buffer[30]);
    dev->quaternion.w = qw / BNO055_QUAT_SCALE;
    dev->quaternion.x = qx / BNO055_QUAT_SCALE;
    dev->quaternion.y = qy / BNO055_QUAT_SCALE;
    dev->quaternion.z = qz / BNO055_QUAT_SCALE;
    
    /* Parse linear acceleration (0x28-0x2D) */
    int16_t lax = (int16_t)((buffer[33] << 8) | buffer[32]);
    int16_t lay = (int16_t)((buffer[35] << 8) | buffer[34]);
    int16_t laz = (int16_t)((buffer[37] << 8) | buffer[36]);
    dev->linear_accel.x = lax / BNO055_ACCEL_SCALE;
    dev->linear_accel.y = lay / BNO055_ACCEL_SCALE;
    dev->linear_accel.z = laz / BNO055_ACCEL_SCALE;
    
    /* Parse gravity vector (0x2E-0x33) */
    int16_t grx = (int16_t)((buffer[39] << 8) | buffer[38]);
    int16_t gry = (int16_t)((buffer[41] << 8) | buffer[40]);
    int16_t grz = (int16_t)((buffer[43] << 8) | buffer[42]);
    dev->gravity.x = grx / BNO055_ACCEL_SCALE;
    dev->gravity.y = gry / BNO055_ACCEL_SCALE;
    dev->gravity.z = grz / BNO055_ACCEL_SCALE;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadEuler(BNO055_Handle_t *dev, BNO055_Euler_t *euler)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];
    
    if (dev == NULL || euler == NULL) return HAL_ERROR;
    
    status = BNO055_ReadRegs(dev, BNO055_REG_EUL_HEADING_LSB, buffer, 6);
    if (status != HAL_OK) return status;
    
    int16_t heading = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t roll = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t pitch = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    euler->heading = heading / BNO055_EULER_SCALE;
    euler->roll = roll / BNO055_EULER_SCALE;
    euler->pitch = pitch / BNO055_EULER_SCALE;
    
    dev->euler = *euler;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadQuaternion(BNO055_Handle_t *dev, BNO055_Quaternion_t *quat)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[8];
    
    if (dev == NULL || quat == NULL) return HAL_ERROR;
    
    status = BNO055_ReadRegs(dev, BNO055_REG_QUA_W_LSB, buffer, 8);
    if (status != HAL_OK) return status;
    
    int16_t w = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t x = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t y = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t z = (int16_t)((buffer[7] << 8) | buffer[6]);
    
    quat->w = w / BNO055_QUAT_SCALE;
    quat->x = x / BNO055_QUAT_SCALE;
    quat->y = y / BNO055_QUAT_SCALE;
    quat->z = z / BNO055_QUAT_SCALE;
    
    dev->quaternion = *quat;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadAccel(BNO055_Handle_t *dev, BNO055_Vector3_t *accel)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];
    
    if (dev == NULL || accel == NULL) return HAL_ERROR;
    
    status = BNO055_ReadRegs(dev, BNO055_REG_ACC_X_LSB, buffer, 6);
    if (status != HAL_OK) return status;
    
    int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    accel->x = x / BNO055_ACCEL_SCALE;
    accel->y = y / BNO055_ACCEL_SCALE;
    accel->z = z / BNO055_ACCEL_SCALE;
    
    dev->accel = *accel;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadGyro(BNO055_Handle_t *dev, BNO055_Vector3_t *gyro)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];
    
    if (dev == NULL || gyro == NULL) return HAL_ERROR;
    
    status = BNO055_ReadRegs(dev, BNO055_REG_GYR_X_LSB, buffer, 6);
    if (status != HAL_OK) return status;
    
    int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    gyro->x = x / BNO055_GYRO_SCALE;
    gyro->y = y / BNO055_GYRO_SCALE;
    gyro->z = z / BNO055_GYRO_SCALE;
    
    dev->gyro = *gyro;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadMag(BNO055_Handle_t *dev, BNO055_Vector3_t *mag)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];
    
    if (dev == NULL || mag == NULL) return HAL_ERROR;
    
    status = BNO055_ReadRegs(dev, BNO055_REG_MAG_X_LSB, buffer, 6);
    if (status != HAL_OK) return status;
    
    int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    mag->x = x / BNO055_MAG_SCALE;
    mag->y = y / BNO055_MAG_SCALE;
    mag->z = z / BNO055_MAG_SCALE;
    
    dev->mag = *mag;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadLinearAccel(BNO055_Handle_t *dev, BNO055_Vector3_t *lin_accel)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];
    
    if (dev == NULL || lin_accel == NULL) return HAL_ERROR;
    
    status = BNO055_ReadRegs(dev, BNO055_REG_LIA_X_LSB, buffer, 6);
    if (status != HAL_OK) return status;
    
    int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    lin_accel->x = x / BNO055_ACCEL_SCALE;
    lin_accel->y = y / BNO055_ACCEL_SCALE;
    lin_accel->z = z / BNO055_ACCEL_SCALE;
    
    dev->linear_accel = *lin_accel;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadGravity(BNO055_Handle_t *dev, BNO055_Vector3_t *gravity)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];
    
    if (dev == NULL || gravity == NULL) return HAL_ERROR;
    
    status = BNO055_ReadRegs(dev, BNO055_REG_GRV_X_LSB, buffer, 6);
    if (status != HAL_OK) return status;
    
    int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    gravity->x = x / BNO055_ACCEL_SCALE;
    gravity->y = y / BNO055_ACCEL_SCALE;
    gravity->z = z / BNO055_ACCEL_SCALE;
    
    dev->gravity = *gravity;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadTemp(BNO055_Handle_t *dev, int8_t *temp)
{
    if (dev == NULL || temp == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status = BNO055_ReadReg(dev, BNO055_REG_TEMP, (uint8_t*)temp);
    if (status == HAL_OK) {
        dev->temperature = *temp;
    }
    return status;
}

HAL_StatusTypeDef BNO055_GetCalibStatus(BNO055_Handle_t *dev, BNO055_CalibStatus_t *status)
{
    uint8_t calib;
    HAL_StatusTypeDef ret;
    
    if (dev == NULL || status == NULL) return HAL_ERROR;
    
    ret = BNO055_ReadReg(dev, BNO055_REG_CALIB_STAT, &calib);
    if (ret != HAL_OK) return ret;
    
    status->system = (calib >> 6) & 0x03;
    status->gyro = (calib >> 4) & 0x03;
    status->accel = (calib >> 2) & 0x03;
    status->mag = calib & 0x03;
    
    dev->calib_status = *status;
    
    return HAL_OK;
}

bool BNO055_IsFullyCalibrated(BNO055_Handle_t *dev)
{
    BNO055_CalibStatus_t status;
    if (BNO055_GetCalibStatus(dev, &status) != HAL_OK) {
        return false;
    }
    return (status.system == 3 && status.gyro == 3 && 
            status.accel == 3 && status.mag == 3);
}

HAL_StatusTypeDef BNO055_GetCalibOffsets(BNO055_Handle_t *dev, BNO055_Offsets_t *offsets)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[22];
    BNO055_OpMode_t prev_mode;
    
    if (dev == NULL || offsets == NULL) return HAL_ERROR;
    
    /* Must be in config mode to read offsets */
    prev_mode = dev->op_mode;
    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != HAL_OK) return status;
    HAL_Delay(25);
    
    /* Read all offset registers */
    status = BNO055_ReadRegs(dev, BNO055_REG_ACC_OFFSET_X_LSB, buffer, 22);
    if (status != HAL_OK) {
        BNO055_SetMode(dev, prev_mode);
        return status;
    }
    
    offsets->acc_offset_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    offsets->acc_offset_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    offsets->acc_offset_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    offsets->mag_offset_x = (int16_t)((buffer[7] << 8) | buffer[6]);
    offsets->mag_offset_y = (int16_t)((buffer[9] << 8) | buffer[8]);
    offsets->mag_offset_z = (int16_t)((buffer[11] << 8) | buffer[10]);
    offsets->gyr_offset_x = (int16_t)((buffer[13] << 8) | buffer[12]);
    offsets->gyr_offset_y = (int16_t)((buffer[15] << 8) | buffer[14]);
    offsets->gyr_offset_z = (int16_t)((buffer[17] << 8) | buffer[16]);
    offsets->acc_radius = (int16_t)((buffer[19] << 8) | buffer[18]);
    offsets->mag_radius = (int16_t)((buffer[21] << 8) | buffer[20]);
    
    dev->offsets = *offsets;
    
    /* Restore previous mode */
    BNO055_SetMode(dev, prev_mode);
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SetCalibOffsets(BNO055_Handle_t *dev, const BNO055_Offsets_t *offsets)
{
    HAL_StatusTypeDef status;
    BNO055_OpMode_t prev_mode;
    
    if (dev == NULL || offsets == NULL) return HAL_ERROR;
    
    /* Must be in config mode to write offsets */
    prev_mode = dev->op_mode;
    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != HAL_OK) return status;
    HAL_Delay(25);
    
    /* Write all offset registers */
    uint8_t buffer[22];
    buffer[0] = offsets->acc_offset_x & 0xFF;
    buffer[1] = (offsets->acc_offset_x >> 8) & 0xFF;
    buffer[2] = offsets->acc_offset_y & 0xFF;
    buffer[3] = (offsets->acc_offset_y >> 8) & 0xFF;
    buffer[4] = offsets->acc_offset_z & 0xFF;
    buffer[5] = (offsets->acc_offset_z >> 8) & 0xFF;
    buffer[6] = offsets->mag_offset_x & 0xFF;
    buffer[7] = (offsets->mag_offset_x >> 8) & 0xFF;
    buffer[8] = offsets->mag_offset_y & 0xFF;
    buffer[9] = (offsets->mag_offset_y >> 8) & 0xFF;
    buffer[10] = offsets->mag_offset_z & 0xFF;
    buffer[11] = (offsets->mag_offset_z >> 8) & 0xFF;
    buffer[12] = offsets->gyr_offset_x & 0xFF;
    buffer[13] = (offsets->gyr_offset_x >> 8) & 0xFF;
    buffer[14] = offsets->gyr_offset_y & 0xFF;
    buffer[15] = (offsets->gyr_offset_y >> 8) & 0xFF;
    buffer[16] = offsets->gyr_offset_z & 0xFF;
    buffer[17] = (offsets->gyr_offset_z >> 8) & 0xFF;
    buffer[18] = offsets->acc_radius & 0xFF;
    buffer[19] = (offsets->acc_radius >> 8) & 0xFF;
    buffer[20] = offsets->mag_radius & 0xFF;
    buffer[21] = (offsets->mag_radius >> 8) & 0xFF;
    
    /* Write byte by byte */
    for (int i = 0; i < 22; i++) {
        status = BNO055_WriteReg(dev, BNO055_REG_ACC_OFFSET_X_LSB + i, buffer[i]);
        if (status != HAL_OK) {
            BNO055_SetMode(dev, prev_mode);
            return status;
        }
    }
    
    dev->offsets = *offsets;
    
    /* Restore previous mode */
    BNO055_SetMode(dev, prev_mode);
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_GetSystemStatus(BNO055_Handle_t *dev, BNO055_SysStatus_t *status, BNO055_SysError_t *error)
{
    uint8_t stat, err;
    HAL_StatusTypeDef ret;
    
    if (dev == NULL) return HAL_ERROR;
    
    ret = BNO055_ReadReg(dev, BNO055_REG_SYS_STATUS, &stat);
    if (ret != HAL_OK) return ret;
    
    ret = BNO055_ReadReg(dev, BNO055_REG_SYS_ERR, &err);
    if (ret != HAL_OK) return ret;
    
    if (status) *status = (BNO055_SysStatus_t)stat;
    if (error) *error = (BNO055_SysError_t)err;
    
    dev->sys_status = (BNO055_SysStatus_t)stat;
    dev->sys_error = (BNO055_SysError_t)err;
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SetAxisRemap(BNO055_Handle_t *dev, uint8_t config, uint8_t sign)
{
    HAL_StatusTypeDef status;
    BNO055_OpMode_t prev_mode;
    
    if (dev == NULL) return HAL_ERROR;
    
    prev_mode = dev->op_mode;
    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != HAL_OK) return status;
    HAL_Delay(25);
    
    status = BNO055_WriteReg(dev, BNO055_REG_AXIS_MAP_CONFIG, config);
    if (status != HAL_OK) {
        BNO055_SetMode(dev, prev_mode);
        return status;
    }
    
    status = BNO055_WriteReg(dev, BNO055_REG_AXIS_MAP_SIGN, sign);
    
    BNO055_SetMode(dev, prev_mode);
    
    return status;
}

HAL_StatusTypeDef BNO055_Reset(BNO055_Handle_t *dev)
{
    HAL_StatusTypeDef status;
    
    if (dev == NULL) return HAL_ERROR;
    
    status = BNO055_WriteReg(dev, BNO055_REG_SYS_TRIGGER, 0x20);
    if (status != HAL_OK) return status;
    
    HAL_Delay(650);
    
    /* Wait for chip ID */
    uint8_t id = 0;
    uint32_t timeout = HAL_GetTick() + 1000;
    while (id != BNO055_CHIP_ID) {
        BNO055_ReadReg(dev, BNO055_REG_CHIP_ID, &id);
        if (HAL_GetTick() > timeout) {
            return HAL_TIMEOUT;
        }
        HAL_Delay(10);
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SelfTest(BNO055_Handle_t *dev, uint8_t *result)
{
    HAL_StatusTypeDef status;
    BNO055_OpMode_t prev_mode;
    
    if (dev == NULL || result == NULL) return HAL_ERROR;
    
    prev_mode = dev->op_mode;
    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != HAL_OK) return status;
    HAL_Delay(25);
    
    /* Trigger self-test */
    status = BNO055_WriteReg(dev, BNO055_REG_SYS_TRIGGER, 0x01);
    if (status != HAL_OK) {
        BNO055_SetMode(dev, prev_mode);
        return status;
    }
    
    HAL_Delay(400);
    
    /* Read result */
    status = BNO055_ReadReg(dev, BNO055_REG_ST_RESULT, result);
    
    BNO055_SetMode(dev, prev_mode);
    
    return status;
}

/**
 * @file mpu6886.h
 * @brief Component to work with MPU6886
 *
 * Include this header file to use the component.
 *
 * (C) 2019 - Timothee Cruse <timothee.cruse@gmail.com>
 * This code is licensed under the MIT License.
 */

#ifndef _MPU6886_H_
#define _MPU6886_H_

#include "freertos/FreeRTOS.h"
#include "iot_i2c.h"

typedef int32_t mpu6886_err_t;

#define MPU6886_I2C_ADDRESS             ( 0x68 )

/**
 * @brief   MPU6886 FAILURE CODES
 */
#define MPU6886_SUCCESS             ( 0 )     /** Operation completed successfully. */
#define MPU6886_FAIL                ( 1 )     /** Operation failed. */
#define MPU6886_OTHER_ERROR         ( 2 )     /** Other error. */


/**
 * @brief   MPU6886 FAILURE CODES
 */

#define MPU6886_REG_XG_OFFS_TC_H        ( 0x04 )
#define MPU6886_REG_XG_OFFS_TC_L        ( 0x05 )
#define MPU6886_REG_YG_OFFS_TC_H        ( 0x07 )
#define MPU6886_REG_YG_OFFS_TC_L        ( 0x08 )
#define MPU6886_REG_ZG_OFFS_TC_H        ( 0x0A )
#define MPU6886_REG_ZG_OFFS_TC_L        ( 0x0B )

#define MPU6886_REG_SELF_TEST_X_ACCEL   ( 0x0D )
#define MPU6886_REG_SELF_TEST_Y_ACCEL   ( 0x0E )
#define MPU6886_REG_SELF_TEST_Z_ACCEL   ( 0x0F )

#define MPU6886_REG_XG_OFFS_USRH        ( 0x13 )
#define MPU6886_REG_XG_OFFS_USRL        ( 0x14 )
#define MPU6886_REG_YG_OFFS_USRH        ( 0x15 )
#define MPU6886_REG_YG_OFFS_USRL        ( 0x16 ) 
#define MPU6886_REG_ZG_OFFS_USRH        ( 0x17 )
#define MPU6886_REG_ZG_OFFS_USRL        ( 0x18 )
#define MPU6886_REG_SMPLRT_DIV          ( 0x19 )

#define MPU6886_REG_CONFIG              ( 0x1A )
#define MPU6886_REG_GYRO_CONFIG         ( 0x1B )
#define MPU6886_REG_ACCEL_CONFIG_1      ( 0x1C )
#define MPU6886_REG_ACCEL_CONFIG_2      ( 0x1D )
#define MPU6886_REG_LP_MODE_CFG         ( 0x1E )

#define MPU6886_REG_ACCEL_WOM_X_THR     ( 0x20 )
#define MPU6886_REG_ACCEL_WOM_Y_THR     ( 0x21 )
#define MPU6886_REG_ACCEL_WOM_Z_THR     ( 0x22 )
#define MPU6886_REG_FIFO_EN             ( 0x23 )

#define MPU6886_REG_FSYNC_INT           ( 0x36 )

#define MPU6886_REG_INT_PIN_CFG         ( 0x37 )
#define MPU6886_REG_INT_ENABLE          ( 0x38 )
#define MPU6886_REG_FIFO_WM_INT_STATUS  ( 0x39 )
#define MPU6886_REG_INT_STATUS          ( 0x3A )
#define MPU6886_REG_ACCEL_XOUT_H        ( 0x3B )
#define MPU6886_REG_ACCEL_XOUT_L        ( 0x3C )
#define MPU6886_REG_ACCEL_YOUT_H        ( 0x3D )
#define MPU6886_REG_ACCEL_YOUT_L        ( 0x3E )
#define MPU6886_REG_ACCEL_ZOUT_H        ( 0x3F )
#define MPU6886_REG_ACCEL_ZOUT_L        ( 0x40 )
#define MPU6886_REG_TEMP_OUT_H          ( 0x41 )
#define MPU6886_REG_TEMP_OUT_L          ( 0x42 )
#define MPU6886_REG_GYRO_XOUT_H         ( 0x43 )
#define MPU6886_REG_GYRO_XOUT_L         ( 0x44 )
#define MPU6886_REG_GYRO_YOUT_H         ( 0x45 )
#define MPU6886_REG_GYRO_YOUT_L         ( 0x46 )
#define MPU6886_REG_GYRO_ZOUT_H         ( 0x47 )
#define MPU6886_REG_GYRO_ZOUT_L         ( 0x48 )

#define MPU6886_REG_SELF_TEST_X_GYRO    ( 0x50 )
#define MPU6886_REG_SELF_TEST_Y_GYRO    ( 0x51 )
#define MPU6886_REG_SELF_TEST_Z_GYRO    ( 0x52 )
#define MPU6886_REG_E_ID0               ( 0x53 )
#define MPU6886_REG_E_ID1               ( 0x54 )
#define MPU6886_REG_E_ID2               ( 0x55 )
#define MPU6886_REG_E_ID3               ( 0x56 )
#define MPU6886_REG_E_ID4               ( 0x57 )
#define MPU6886_REG_E_ID5               ( 0x58 )
#define MPU6886_REG_E_ID6               ( 0x59 )

#define MPU6886_REG_FIFO_WM_TH1         ( 0x60 )
#define MPU6886_REG_FIFO_WM_TH2         ( 0x61 )

#define MPU6886_REG_SIGNAL_PATH_RESET   ( 0x68 )
#define MPU6886_REG_ACCEL_INTEL_CTRL    ( 0x69 )
#define MPU6886_REG_USER_CTRL           ( 0x6A )
#define MPU6886_REG_PWR_MGMT_1          ( 0x6B )
#define MPU6886_REG_PWR_MGMT_2          ( 0x6C )

#define MPU6886_REG_I2C_IF              ( 0x70 )
#define MPU6886_REG_FIFO_COUNTH         ( 0x72 )
#define MPU6886_REG_FIFO_COUNTL         ( 0x73 )
#define MPU6886_REG_FIFO_R_W            ( 0x74 )
#define MPU6886_REG_WHO_AM_I            ( 0x75 )
#define MPU6886_REG_XA_OFFSET_H         ( 0x77 )
#define MPU6886_REG_XA_OFFSET_L         ( 0x78 )
#define MPU6886_REG_YA_OFFSET_H         ( 0x7A )
#define MPU6886_REG_YA_OFFSET_L         ( 0x7B )
#define MPU6886_REG_ZA_OFFSET_H         ( 0x7D )
#define MPU6886_REG_ZA_OFFSET_L         ( 0x7E )

#define MPU6886_RESET_VALUE_WHO_AM_I    ( 0x19 )

//#define G (9.8)
#define MPU6886_RtA                     ( 57.324841 )
#define MPU6886_AtR    	                ( 0.0174533	)
#define MPU6886_Gyro_Gr	                ( 0.0010653 )

enum MPU6886_ENUM_ACCEL_SCALE {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum MPU6886_ENUM_GYRO_SCALE {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

extern enum MPU6886_ENUM_GYRO_SCALE MPU6886GyroScale;
extern enum MPU6886_ENUM_ACCEL_SCALE MPU6886AccelScale;

/**
 * @brief   Initialize MPU6886
 * *
 * @return  MPU6886_SUCCESS success
 *          MPU6886_FAIL failed
 */
mpu6886_err_t eMPU6886Init( IotI2CHandle_t const handle );

/**
 * @brief   Get accelerometer data
 *
 * @param   ax: pointer to X value
 *          ay: pointer to Y value
 *          az: pointer to Z value
 * 
 * @return  MPU6886_SUCCESS success
 *          MPU6886_FAIL failed
 */
mpu6886_err_t eGetMPU6886AccelData( float * ax, float * ay, float * az );

/**
 * @brief   Get gyro data
 *
 * @param   gx: pointer to X value
 *          gy: pointer to Y value
 *          gz: pointer to Z value
 * 
 * @return  MPU6886_SUCCESS success
 *          MPU6886_FAIL failed
 */
mpu6886_err_t eGetMPU6886GyroData( float * gx, float * gy, float * gz );

/**
 * @brief   Get temp data
 *
 * @param   t: pointer to temp value
 * 
 * @return  MPU6886_SUCCESS success
 *          MPU6886_FAIL failed
 */
mpu6886_err_t eGetMPU6886TempData( float * t );

/**
 * @brief   Get MahonyAHRS data
 *
 * @param   pitch: pointer to pitch value
 *          roll: pointer to roll value
 *          yaw: pointer to yaw value
 * 
 * @return  MPU6886_SUCCESS success
 *          MPU6886_FAIL failed
 */
mpu6886_err_t eGetMPU6886AhrsData( float * pitch, float * roll, float * yaw );

#endif // _MPU6886_H_

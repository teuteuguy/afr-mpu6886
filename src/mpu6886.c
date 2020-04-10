/**
 * @file mpu6886.c
 * @brief Component to work with MPU6886
 *
 * (C) 2020 - Timothee Cruse <timothee.cruse@gmail.com>
 * This code is licensed under the MIT License.
 */

#include "mpu6886.h"

#include "common_io_helpers.h"
#include "MahonyAHRS.h"

#define LIBRARY_LOG_LEVEL IOT_LOG_INFO
#define LIBRARY_LOG_NAME  "mpu6886"
#include "iot_logging_setup.h"

/*-----------------------------------------------------------*/

static IotI2CHandle_t prvMPU6886I2CHandle = NULL;

#define MPU6886_INIT_REG_COUNT 13
static const uint8_t prvMPU6886InitRegisterDefaultList[ MPU6886_INIT_REG_COUNT * 2 ] = {
    MPU6886_REG_PWR_MGMT_1,     ( 0x00 ),
    MPU6886_REG_PWR_MGMT_1,     ( 0x01 << 7 ),
    MPU6886_REG_PWR_MGMT_1,     ( 0x01 << 0 ),
    MPU6886_REG_ACCEL_CONFIG_1, ( 0x10 ),
    MPU6886_REG_GYRO_CONFIG,    ( 0x18 ),
    MPU6886_REG_CONFIG,         ( 0x01 ),
    MPU6886_REG_SMPLRT_DIV,     ( 0x05 ),
    MPU6886_REG_INT_ENABLE,     ( 0x00 ),
    MPU6886_REG_ACCEL_CONFIG_2, ( 0x00 ),
    MPU6886_REG_USER_CTRL,      ( 0x00 ),
    MPU6886_REG_FIFO_EN,        ( 0x00 ),
    MPU6886_REG_INT_PIN_CFG,    ( 0x22 ),
    MPU6886_REG_INT_ENABLE,     ( 0x01 )
};

enum MPU6886_ENUM_GYRO_SCALE MPU6886GyroScale = GFS_2000DPS;
enum MPU6886_ENUM_ACCEL_SCALE MPU6886AccelScale = AFS_8G;

float prvMPU6886AccelRes, prvMPU6886GyroRes;

/*-----------------------------------------------------------*/

void prvGetMPU6886GyroRes( void );
void prvGetMPU6886AccelRes( void );

mpu6886_err_t prvGetMPU6886AccelAdc( int16_t * ax, int16_t * ay, int16_t * az );
mpu6886_err_t prvGetMPU6886GyroAdc( int16_t * gx, int16_t * gy, int16_t * gz );
mpu6886_err_t prvGetMPU6886TempAdc( int16_t * t );

/*-----------------------------------------------------------*/

mpu6886_err_t eMPU6886Init( IotI2CHandle_t const handle )
{
    mpu6886_err_t e = MPU6886_FAIL;
    prvMPU6886I2CHandle = handle;

    uint8_t tempdata[ 2 ] = { 0 };
    uint8_t error_count = 0;
    uint8_t i;

    tempdata[ 0 ] = MPU6886_REG_WHO_AM_I;
    e = eReadI2CBytes( handle, MPU6886_I2C_ADDRESS, tempdata, 1 );
    if ( e != COMMON_IO_SUCCESS || tempdata[ 0 ] != MPU6886_RESET_VALUE_WHO_AM_I )
    {
        IotLogError( "eMPU6886Init: Error (%i) reading MPU6886_REG_WHO_AM_I(%#04x): %#04x vs. %#04x", e, MPU6886_REG_WHO_AM_I, MPU6886_RESET_VALUE_WHO_AM_I, tempdata[ 0 ] );
        return MPU6886_FAIL;
    }

    IotLogDebug( "eMPU6886Init: Read MPU6886_REG_WHO_AM_I(%#04x): %#04x vs. %#04x", MPU6886_REG_WHO_AM_I, tempdata[ 0 ], MPU6886_RESET_VALUE_WHO_AM_I );    
    IotLogDebug( "eMPU6886Init: Setting inits" );    

    for (i = 0; i < MPU6886_INIT_REG_COUNT * 2; i += 2)
    {
        vTaskDelay( 10 / portTICK_PERIOD_MS );
        
        tempdata[ 0 ] = prvMPU6886InitRegisterDefaultList[ i ];
        tempdata[ 1 ] = prvMPU6886InitRegisterDefaultList[ i + 1 ];
        e = eWriteI2CBytes( handle, MPU6886_I2C_ADDRESS, tempdata, 2 );
        if ( e != COMMON_IO_SUCCESS )
        {
            IotLogError( "eMPU6886Init: Error writting %#04x register", prvMPU6886InitRegisterDefaultList[ i ] );
            error_count++;
        }
        else
        {
            IotLogDebug( "eMPU6886Init: Wrote %#04x @ %#04x", tempdata[ 0 ], tempdata[ 1 ] );
        }
        
    }

    vTaskDelay( 100 / portTICK_PERIOD_MS );
    
    prvGetMPU6886GyroRes();
    prvGetMPU6886AccelRes();

    if (error_count == 0)
    {
        IotLogDebug( "eMPU6886Init: MPU6886 initialized" );
        return MPU6886_SUCCESS;
    }
    else
    {
        IotLogError( "eMPU6886Init: %d errors found while initializing MPU6886", error_count );
        return MPU6886_FAIL;
    }

    return MPU6886_FAIL;
}

/*-----------------------------------------------------------*/

void prvGetMPU6886GyroRes( void )
{
    switch ( MPU6886GyroScale )
    {
        // Possible gyro scales (and their register bit settings) are:
        case GFS_250DPS:
            prvMPU6886GyroRes = 250.0 / 32768.0;
            break;
        case GFS_500DPS:
            prvMPU6886GyroRes = 500.0 / 32768.0;
            break;
        case GFS_1000DPS:
            prvMPU6886GyroRes = 1000.0 / 32768.0;
            break;
        case GFS_2000DPS:
            prvMPU6886GyroRes = 2000.0 / 32768.0;
            break;
    }
}

/*-----------------------------------------------------------*/

void prvGetMPU6886AccelRes( void )
{
    switch ( MPU6886AccelScale )
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            prvMPU6886AccelRes = 2.0 / 32768.0;
            break;
        case AFS_4G:
            prvMPU6886AccelRes = 4.0 / 32768.0;
            break;
        case AFS_8G:
            prvMPU6886AccelRes = 8.0 / 32768.0;
            break;
        case AFS_16G:
            prvMPU6886AccelRes = 16.0 / 32768.0;
            break;
    }
}

/*-----------------------------------------------------------*/


mpu6886_err_t eGetMPU6886AccelData( float * ax, float * ay, float * az )
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    mpu6886_err_t e = prvGetMPU6886AccelAdc( &accX, &accY, &accZ );

    *ax = ( float )accX * prvMPU6886AccelRes;
    *ay = ( float )accY * prvMPU6886AccelRes;
    *az = ( float )accZ * prvMPU6886AccelRes;

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t eGetMPU6886GyroData( float * gx, float * gy, float * gz )
{
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    mpu6886_err_t e = prvGetMPU6886GyroAdc( &gyroX, &gyroY, &gyroZ );

    *gx = (float)gyroX * prvMPU6886GyroRes;
    *gy = (float)gyroY * prvMPU6886GyroRes;
    *gz = (float)gyroZ * prvMPU6886GyroRes;

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t eGetMPU6886TempData( float * t )
{
    int16_t temp = 0;
    mpu6886_err_t e = prvGetMPU6886TempAdc( &temp );

    *t = (float)temp / 326.8 + 25.0;

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t prvGetMPU6886AccelAdc( int16_t * ax, int16_t  *ay, int16_t * az )
{
    uint8_t buf[ 6 ] = { 0 };
    buf[ 0 ] = MPU6886_REG_ACCEL_XOUT_H;

    mpu6886_err_t e = eReadI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, buf, 6 );

    *ax = ( ( int16_t )buf[ 0 ] << 8 ) | buf[ 1 ];
    *ay = ( ( int16_t )buf[ 2 ] << 8 ) | buf[ 3 ];
    *az = ( ( int16_t )buf[ 4 ] << 8 ) | buf[ 5 ];

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t prvGetMPU6886GyroAdc( int16_t * gx, int16_t * gy, int16_t * gz )
{
    uint8_t buf[ 6 ] = { 0 };
    buf[ 0 ] = MPU6886_REG_GYRO_XOUT_H;

    mpu6886_err_t e = eReadI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, buf, 6 );

    *gx = ( ( int16_t )buf[ 0 ] << 8 ) | buf[ 1 ];
    *gy = ( ( int16_t )buf[ 2 ] << 8 ) | buf[ 3 ];
    *gz = ( ( int16_t )buf[ 4 ] << 8 ) | buf[ 5 ];

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t prvGetMPU6886TempAdc( int16_t * t )
{
    uint8_t buf[ 2 ] = { 0 };
    buf[ 0 ] = MPU6886_REG_TEMP_OUT_H;

    mpu6886_err_t e = eReadI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, buf, 2 );

    *t = ( ( uint16_t )buf[ 0 ] << 8 ) | buf[ 1 ];

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t eGetMPU6886AhrsData( float * pitch, float * roll, float * yaw )
{
    mpu6886_err_t e = MPU6886_FAIL;

    float accX = 0; 
    float accY = 0;
    float accZ = 0;

    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;

    e = eGetMPU6886GyroData( &gyroX, &gyroY, &gyroZ );
    if ( e != MPU6886_SUCCESS ) {
        return e;
    }

    e = eGetMPU6886AccelData( &accX, &accY, &accZ );
    if ( e != MPU6886_SUCCESS ) {
        return e;
    }

    float deg_to_rad = 0.017453292519943295769236907684886f;
    MahonyAHRSupdateIMU( gyroX * deg_to_rad,
                        gyroY * deg_to_rad,
                        gyroZ * deg_to_rad,
                        accX, accY, accZ, 
                        pitch, roll, yaw );

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t eSetMPU6886FIFOEnable( bool enableflag )
{
    mpu6886_err_t e = MPU6886_FAIL;
    uint8_t regdata[ 2 ] = { 0 };

    if (enableflag)
    {
        regdata[ 0 ] = MPU6886_REG_FIFO_EN;
        regdata[ 1 ] = 0x0C;
        e = eWriteI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, regdata, 2 );
        if ( e != COMMON_IO_SUCCESS )
        {
            return e;
        }
        
        regdata[ 0 ] = MPU6886_REG_USER_CTRL;
        regdata[ 1 ] = 0x40;
        e = eWriteI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, regdata, 2 );
        if ( e != COMMON_IO_SUCCESS )
        {
            return e;
        }
    }
    else
    {
        regdata[ 0 ] = MPU6886_REG_FIFO_EN;
        regdata[ 1 ] = 0x00;
        e = eWriteI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, regdata, 2 );
        if ( e != COMMON_IO_SUCCESS )
        {
            return e;
        }
        regdata[ 0 ] = MPU6886_REG_USER_CTRL;
        regdata[ 1 ] = 0x00;
        e = eWriteI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, regdata, 2 );
        if (e != COMMON_IO_SUCCESS)
        {
            return e;
        }
    }

    return e;
}

/*-----------------------------------------------------------*/


mpu6886_err_t eReadMPU6886FIFO( uint8_t * rd_data )
{
    rd_data[ 0 ] = MPU6886_REG_FIFO_R_W;
    return eReadI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, rd_data, 1 );
}

/*-----------------------------------------------------------*/


mpu6886_err_t eReadMPU6886FIFOBuff( uint8_t * rd_data , uint16_t length )
{
    rd_data[ 0 ] = MPU6886_REG_FIFO_R_W;
    return eReadI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, rd_data, length );
}

/*-----------------------------------------------------------*/

mpu6886_err_t eReadMPU6886FIFOCount( uint16_t * rd_data )
{
    uint8_t temp[2] = { MPU6886_REG_FIFO_COUNTH, 0 };
    mpu6886_err_t e = eReadI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, temp, 2 );
    *rd_data = temp[ 0 ];
    *rd_data <<= 8;
    *rd_data |= temp[ 1 ];
    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t eSetMPU6886GyroFsr( enum MPU6886_ENUM_GYRO_SCALE scale )
{
    unsigned char regdata[ 2 ] = { MPU6886_REG_GYRO_CONFIG, ( scale << 3 ) };
    
    mpu6886_err_t e = eWriteI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, regdata, 2 );

    vTaskDelay( 10 / portTICK_PERIOD_MS );

    MPU6886GyroScale = scale;
    prvGetMPU6886GyroRes();

    return e;
}

/*-----------------------------------------------------------*/

mpu6886_err_t eSetMPU6886AccelFsr( enum MPU6886_ENUM_ACCEL_SCALE scale )
{
    unsigned char regdata[ 2 ] = { MPU6886_REG_ACCEL_CONFIG_1, ( scale << 3 ) };
    
    mpu6886_err_t e = eWriteI2CBytes( prvMPU6886I2CHandle, MPU6886_I2C_ADDRESS, regdata, 2 );
    
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    
    MPU6886AccelScale = scale;
    prvGetMPU6886AccelRes();

    return e;
}

/*-----------------------------------------------------------*/



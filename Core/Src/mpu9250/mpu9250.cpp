#include <mpu9250/mpu9250.hpp>

/**
 * @brief   Constructor for the IMU class.
 *
 * Takes the HAL I2C Handle
 */
MPU9250::MPU9250(I2C_HandleTypeDef *t_hi2c)
        : m_hi2c_(t_hi2c)
{
    begin();
}

/**
 * @brief Constructor for the IMU class that accepts an address
 */
MPU9250::MPU9250(I2C_HandleTypeDef *t_hi2c, uint8_t t_mpu9250_address)
        : m_hi2c_(t_hi2c), m_mpu9250_address_(t_mpu9250_address)
{
    begin();
}

/**
 * @brief   Initialize the IMU sensor configuration.
 *
 * Sets up the IMU sensor by configuring registers and settings
 * for accelerometer, gyroscope, and magnetometer communication.
 */
void MPU9250::begin() const
{
    // Reset the IMU
    uint8_t reset = 0x80;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(m_hi2c_,
            m_mpu9250_address_ << byte, pwr_mgmt_1, byte, &reset, byte,
            default_timeout);
    HAL_Delay(100);
    if (status == HAL_ERROR) {
        return;
    }

    // Wake up the IMU
    uint8_t wakeup = 0;
    status = HAL_I2C_Mem_Write(m_hi2c_, m_mpu9250_address_ << byte, pwr_mgmt_1,
            byte, &wakeup, byte, default_timeout);
    HAL_Delay(100);
    if (status == HAL_ERROR) {
        return;
    }

    // Select the stable clock
    uint8_t stableClock = 0x01;
    status = HAL_I2C_Mem_Write(m_hi2c_, m_mpu9250_address_ << byte, pwr_mgmt_1,
            byte, &stableClock, byte, default_timeout);
    HAL_Delay(100);
    if (status == HAL_ERROR) {
        return;
    }

    // Configure the Gyroscope
    uint8_t set_500_dps = 0x08;
    status = HAL_I2C_Mem_Write(m_hi2c_, m_mpu9250_address_ << byte, gyro_config,
            byte, &set_500_dps, byte, default_timeout);
    if (status == HAL_ERROR) {
        return;
    }

    // Configure the Accelerometer
    uint8_t set_4g = 0x08;
    status = HAL_I2C_Mem_Write(m_hi2c_, m_mpu9250_address_ << byte,
            accel_config, byte, &set_4g, byte, default_timeout);
    if (status == HAL_ERROR) {
        return;
    }

    // Enable bypass
    uint8_t bypass = 0x02;
    status = HAL_I2C_Mem_Write(m_hi2c_, m_mpu9250_address_ << byte, int_pin_cfg,
            byte, &bypass, byte, default_timeout);
    HAL_Delay(100);
    if (status == HAL_ERROR) {
        return;
    }

    // Configure the Magnetmeteor (Turn if off)
    uint8_t turnOff = 0x00;
    status = HAL_I2C_Mem_Write(m_hi2c_, ak8963_address << byte, ak8963_cntl,
            byte, &turnOff, byte, default_timeout);
    HAL_Delay(100);
    if (status == HAL_ERROR) {
        return;
    }

    // Turn on the Magnetmeteor (Continuous 16-Bit Mode)
    uint8_t turnOn = 0x16;
    status = HAL_I2C_Mem_Write(m_hi2c_, ak8963_address << byte, ak8963_cntl,
            byte, &turnOn, byte, default_timeout);
    if (status == HAL_ERROR) {
        return;
    }

}

/**
 * @brief   Read sensor data from IMU
 *          accelerometer[0] = <read accelerometer X value>;
 *          accelerometer[1] = <read accelerometer Y value>;
 *          accelerometer[2] = <read accelerometer Z value>;
 *          gyroscope[0] = <read gyroscope X value>;
 *          gyroscope[1] = <read gyroscope Y value>;
 *          gyroscope[2] = <read gyroscope Z value>;
 *          magnetometer[0] = <read magnetometer X value>;
 *          magnetometer[1] = <read magnetometer Y value>;
 *          magnetometer[2] = <read magnetometer Z value>;
 */
void MPU9250::read()
{
    read_accelerometer();
    read_gyroscope();
    read_magnetometer();
}

void MPU9250::read_accelerometer()
{
    /* Read accelerometer data (Big Endian Order) */
    uint8_t raw_accel_data[6] =
        { 0, 0, 0, 0, 0, 0 };
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_hi2c_,
            m_mpu9250_address_ << byte, accel_xout_h, byte, raw_accel_data, 6,
            default_timeout);
    if (status == HAL_ERROR) {
        return;
    }

    // Converting Raw Accel Data to Readable data
    m_accelerometer_.x = static_cast<int16_t>((raw_accel_data[0] << bits_per_byte)
            | (raw_accel_data[1] & byte_mask));
    m_accelerometer_.y = static_cast<int16_t>((raw_accel_data[2] << bits_per_byte)
            | (raw_accel_data[3] & byte_mask));
    m_accelerometer_.z = static_cast<int16_t>((raw_accel_data[4] << bits_per_byte)
            | (raw_accel_data[5] & byte_mask));
}

void MPU9250::read_gyroscope()
{
    /* Read gyroscope data (Big Endian Order) */
    uint8_t raw_gyro_data[6] =
        { 0, 0, 0, 0, 0, 0 };
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_hi2c_,
            m_mpu9250_address_ << byte, gyro_xout_h, byte, raw_gyro_data, 6,
            default_timeout);
    if (status == HAL_ERROR) {
        return;
    }

    // Converting Raw Gyro Data to Readable data
    m_gyroscope_.x = static_cast<int16_t>((raw_gyro_data[0] << bits_per_byte)
            | (raw_gyro_data[1] & byte_mask));
    m_gyroscope_.y = static_cast<int16_t>((raw_gyro_data[2] << bits_per_byte)
            | (raw_gyro_data[3] & byte_mask));
    m_gyroscope_.z = static_cast<int16_t>((raw_gyro_data[4] << bits_per_byte)
            | (raw_gyro_data[5] & byte_mask));
}

void MPU9250::read_magnetometer()
{
    /* Read magnetometer data one-burst (Little Endian Order) */
    uint8_t st1 = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_hi2c_, ak8963_address << byte,
            ak8963_st1, byte, &st1, byte, default_timeout);
    if ((status == HAL_ERROR) || !(st1 & ak8963_data_ready)) {
        return;
    }

    uint8_t raw_mag_data[7] =
        { 0, 0, 0, 0, 0, 0, 0 };
    status = HAL_I2C_Mem_Read(m_hi2c_, ak8963_address << byte, ak8963_xout_l,
            byte, raw_mag_data, 7, default_timeout);

    uint8_t st2 = raw_mag_data[6];
    if (!(st2 & ak8963_magnetic_overflow)) {
        // Converting Raw Mag Data to Readable data
        m_magnetometer_.x =
                static_cast<int16_t>((raw_mag_data[1] << bits_per_byte)
                        | (raw_mag_data[0] & byte_mask));
        m_magnetometer_.y =
                static_cast<int16_t>((raw_mag_data[3] << bits_per_byte)
                        | (raw_mag_data[2] & byte_mask));
        m_magnetometer_.z =
                static_cast<int16_t>((raw_mag_data[5] << bits_per_byte)
                        | (raw_mag_data[4] & byte_mask));
    }
}

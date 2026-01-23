/*
 * mpu9250_register_map.hpp
 *
 *      Author: Luis
 */

#ifndef MPU9250_REGISTER_MAP_HPP
#define MPU9250_REGISTER_MAP_HPP

#include <cstdint>

// magnetometer constants
constexpr uint8_t ak8963_data_ready = 0x01;
constexpr uint8_t ak8963_magnetic_overflow = 0x08;

/** imu constants */
constexpr uint16_t time_delay_ms = 1000;
constexpr size_t accel_mag_data_size = 12;
constexpr float math_pi = 3.1416f;
constexpr float deg_to_rad = math_pi / 180.0f;
constexpr float rad_to_deg = 180.0f / math_pi;
constexpr float sensors_gravity_std = 9.807f;
constexpr float gyro_max_threshold_rps = 2200.0f;
constexpr float accel_max_threshold_mps2 = 17.6f;
constexpr int byte_shift_amount = 8;
constexpr int half_word_shift_amount = 16;
constexpr int three_byte_shift_amount = 24;
constexpr int x_axis = 0;
constexpr int y_axis = 1;
constexpr int z_axis = 2;
constexpr int bits_per_byte = 8;
constexpr int byte_mask = 0xff;
constexpr float accel_scale = 0.3f;
constexpr uint8_t byte = 1;

// magnetometer registers
constexpr uint8_t ak8963_who_am_i = 0x00; // should return 0x48
constexpr uint8_t ak8963_info = 0x01;
constexpr uint8_t ak8963_st1 = 0x02; // data ready status bit 0
constexpr uint8_t ak8963_xout_l = 0x03;
constexpr uint8_t ak8963_xout_h = 0x04;
constexpr uint8_t ak8963_yout_l = 0x05;
constexpr uint8_t ak8963_yout_h = 0x06;
constexpr uint8_t ak8963_zout_l = 0x07;
constexpr uint8_t ak8963_zout_h = 0x08;
constexpr uint8_t ak8963_st2 = 0x09; // data overflow bit 3 and data read error status bit 2
constexpr uint8_t ak8963_cntl = 0x0a; // power down, single-measurement, self-test, fuse rom modes
constexpr uint8_t ak8963_astc = 0x0c; // self test control
constexpr uint8_t ak8963_i2cdis = 0x0f; // i2c disable
constexpr uint8_t ak8963_asax = 0x10; // fuse rom x-axis sensitivity
constexpr uint8_t ak8963_asay = 0x11; // fuse rom y-axis sensitivity
constexpr uint8_t ak8963_asaz = 0x12; // fuse rom z-axis sensitivity

constexpr uint8_t self_test_x_gyro = 0x00;
constexpr uint8_t self_test_y_gyro = 0x01;
constexpr uint8_t self_test_z_gyro = 0x02;

constexpr uint8_t self_test_x_accel = 0x0d;
constexpr uint8_t self_test_y_accel = 0x0e;
constexpr uint8_t self_test_z_accel = 0x0f;

constexpr uint8_t self_test_a = 0x10;

constexpr uint8_t xg_offset_h = 0x13;
constexpr uint8_t xg_offset_l = 0x14;
constexpr uint8_t yg_offset_h = 0x15;
constexpr uint8_t yg_offset_l = 0x16;
constexpr uint8_t zg_offset_h = 0x17;
constexpr uint8_t zg_offset_l = 0x18;
constexpr uint8_t smplrt_div = 0x19;
constexpr uint8_t mpu_config = 0x1a;
constexpr uint8_t gyro_config = 0x1b;
constexpr uint8_t accel_config = 0x1c;
constexpr uint8_t accel_config2 = 0x1d;
constexpr uint8_t lp_accel_odr = 0x1e;
constexpr uint8_t wom_thr = 0x1f;

constexpr uint8_t mot_dur = 0x20;
constexpr uint8_t zmot_thr = 0x21;
constexpr uint8_t zrmot_dur = 0x22;

constexpr uint8_t fifo_en = 0x23;
constexpr uint8_t i2c_mst_ctrl = 0x24;
constexpr uint8_t i2c_slv0_addr = 0x25;
constexpr uint8_t i2c_slv0_reg = 0x26;
constexpr uint8_t i2c_slv0_ctrl = 0x27;
constexpr uint8_t i2c_slv1_addr = 0x28;
constexpr uint8_t i2c_slv1_reg = 0x29;
constexpr uint8_t i2c_slv1_ctrl = 0x2a;
constexpr uint8_t i2c_slv2_addr = 0x2b;
constexpr uint8_t i2c_slv2_reg = 0x2c;
constexpr uint8_t i2c_slv2_ctrl = 0x2d;
constexpr uint8_t i2c_slv3_addr = 0x2e;
constexpr uint8_t i2c_slv3_reg = 0x2f;
constexpr uint8_t i2c_slv3_ctrl = 0x30;
constexpr uint8_t i2c_slv4_addr = 0x31;
constexpr uint8_t i2c_slv4_reg = 0x32;
constexpr uint8_t i2c_slv4_do = 0x33;
constexpr uint8_t i2c_slv4_ctrl = 0x34;
constexpr uint8_t i2c_slv4_di = 0x35;
constexpr uint8_t i2c_mst_status = 0x36;
constexpr uint8_t int_pin_cfg = 0x37;
constexpr uint8_t int_enable = 0x38;
constexpr uint8_t dmp_int_status = 0x39;
constexpr uint8_t int_status = 0x3a;

constexpr uint8_t accel_xout_h = 0x3b;
constexpr uint8_t accel_xout_l = 0x3c;
constexpr uint8_t accel_yout_h = 0x3d;
constexpr uint8_t accel_yout_l = 0x3e;
constexpr uint8_t accel_zout_h = 0x3f;
constexpr uint8_t accel_zout_l = 0x40;

constexpr uint8_t temp_out_h = 0x41;
constexpr uint8_t temp_out_l = 0x42;

constexpr uint8_t gyro_xout_h = 0x43;
constexpr uint8_t gyro_xout_l = 0x44;
constexpr uint8_t gyro_yout_h = 0x45;
constexpr uint8_t gyro_yout_l = 0x46;
constexpr uint8_t gyro_zout_h = 0x47;
constexpr uint8_t gyro_zout_l = 0x48;

constexpr uint8_t ext_sens_data_00 = 0x49;
constexpr uint8_t ext_sens_data_01 = 0x4a;
constexpr uint8_t ext_sens_data_02 = 0x4b;
constexpr uint8_t ext_sens_data_03 = 0x4c;
constexpr uint8_t ext_sens_data_04 = 0x4d;
constexpr uint8_t ext_sens_data_05 = 0x4e;
constexpr uint8_t ext_sens_data_06 = 0x4f;
constexpr uint8_t ext_sens_data_07 = 0x50;
constexpr uint8_t ext_sens_data_08 = 0x51;
constexpr uint8_t ext_sens_data_09 = 0x52;
constexpr uint8_t ext_sens_data_10 = 0x53;
constexpr uint8_t ext_sens_data_11 = 0x54;
constexpr uint8_t ext_sens_data_12 = 0x55;
constexpr uint8_t ext_sens_data_13 = 0x56;
constexpr uint8_t ext_sens_data_14 = 0x57;
constexpr uint8_t ext_sens_data_15 = 0x58;
constexpr uint8_t ext_sens_data_16 = 0x59;
constexpr uint8_t ext_sens_data_17 = 0x5a;
constexpr uint8_t ext_sens_data_18 = 0x5b;
constexpr uint8_t ext_sens_data_19 = 0x5c;
constexpr uint8_t ext_sens_data_20 = 0x5d;
constexpr uint8_t ext_sens_data_21 = 0x5e;
constexpr uint8_t ext_sens_data_22 = 0x5f;
constexpr uint8_t ext_sens_data_23 = 0x60;

constexpr uint8_t mot_detect_status = 0x61;

constexpr uint8_t i2c_slv0_do = 0x63;
constexpr uint8_t i2c_slv1_do = 0x64;
constexpr uint8_t i2c_slv2_do = 0x65;
constexpr uint8_t i2c_slv3_do = 0x66;

constexpr uint8_t i2c_mst_delay_ctrl = 0x67;
constexpr uint8_t signal_path_reset = 0x68;
constexpr uint8_t mot_detect_ctrl = 0x69;
constexpr uint8_t user_ctrl = 0x6a;
constexpr uint8_t pwr_mgmt_1 = 0x6b;
constexpr uint8_t pwr_mgmt_2 = 0x6c;

constexpr uint8_t dmp_bank = 0x6d;
constexpr uint8_t dmp_rw_pnt = 0x6e;
constexpr uint8_t dmp_reg = 0x6f;
constexpr uint8_t dmp_reg_1 = 0x70;
constexpr uint8_t dmp_reg_2 = 0x71;

constexpr uint8_t fifo_counth = 0x72;
constexpr uint8_t fifo_countl = 0x73;
constexpr uint8_t fifo_r_w = 0x74;

constexpr uint8_t who_am_i_mpu9250 = 0x75;

constexpr uint8_t xa_offset_h = 0x77;
constexpr uint8_t xa_offset_l = 0x78;
constexpr uint8_t ya_offset_h = 0x7a;
constexpr uint8_t ya_offset_l = 0x7b;
constexpr uint8_t za_offset_h = 0x7d;
constexpr uint8_t za_offset_l = 0x7e;


#endif /* MPU9250_REGISTER_MAP_HPP */

#include "pico-icm20948.h"

// "static" with function(s) means not to tell these functions to linker
// so, we can't call these function from other source files.

int8_t icm20948_init(icm20948_config_t* config) {
    uint8_t reg[2], buf;

    config->semphrTake();
    // wake up accel/gyro!
    // first write register then, write value
    reg[0] = PWR_MGMT_1; reg[1] = 0x00;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // switch to user bank to 0
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // auto select clock source
    reg[0] = PWR_MGMT_1; reg[1] = 0x01;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // disable accel/gyro once
    reg[0] = PWR_MGMT_2; reg[1] = 0x3F;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);
    config->delayFunc(10);

    // enable accel/gyro (again)
    reg[0] = PWR_MGMT_2; reg[1] = 0x00;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // check if the accel/gyro could be accessed
    reg[0] = WHO_AM_I_ICM20948;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 1, true, config->i2cTimeout_us);
    i2c_read_timeout_per_char_us(config->i2c, config->addr_accel_gyro, &buf, 1, false, config->i2cTimeout_us);
#ifndef NDEBUG    
    printf("AG. WHO_AM_I: 0x%X\n", buf);
#endif
    if (buf != 0xEA) {
        config->semphrGive();
        return -1;
    }

    // switch to user bank 2
    reg[0] = REG_BANK_SEL; reg[1] = 0x20;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // gyro config
    //
    // set full scale to 1000+-dps
    // set noise bandwidth to 
    // smaller bandwidth means lower noise level & slower max sample rate
    reg[0] = GYRO_CONFIG_1; reg[1] = 0x29;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);
    //
    // set gyro output data rate to 20Hz
    // output_data_rate = 1.125kHz / (1 + GYRO_SMPLRT_DIV)
    // 1125 / 57 = 20
    reg[0] = GYRO_SMPLRT_DIV; reg[1] = 0x38;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // accel config
    //
    // set full scale to +-4g
    // set noise bandwidth to 136Hz
    reg[0] = ACCEL_CONFIG; reg[1] = 0x11;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);
    //
    // set accel output data rate to 20Hz
    // output_data_rate = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    // 16 bits for ACCEL_SMPLRT_DIV
    reg[0] = ACCEL_SMPLRT_DIV_2; reg[1] = 0x38;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // switch to user bank to 0
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // wake up mag! (INT_PIN_CFG, BYPASS_EN = 1)
    reg[0] = INT_PIN_CFG; reg[1] = 0x02;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, reg, 2, false, config->i2cTimeout_us);

    // check if the mag could be accessed
    reg[0] = 0x01;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_mag, reg, 1, true, config->i2cTimeout_us);
    i2c_read_timeout_per_char_us(config->i2c, config->addr_mag, &buf, 1, false, config->i2cTimeout_us);
#ifndef NDEBUG
    printf("MAG. WHO_AM_I: 0x%X\n", buf);
#endif
    if (buf != 0x09) {
        config->semphrGive();
        return -1;
    }

    // config mag
    //
    // set mag mode, to measure continuously in 100Hz
    reg[0] = AK09916_CNTL2; reg[1] = 0x08;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_mag, reg, 2, false, config->i2cTimeout_us);
    config->semphrGive();

    return 0;
}

void icm20948_set_mag_rate(icm20948_config_t* config, uint8_t mode) {
    // Single measurement              : mode = 0
    // Continuous measurement in  10 Hz: mode = 10
    // Continuous measurement in  20 Hz: mode = 20
    // Continuous measurement in  50 Hz: mode = 50
    // Continuous measurement in 100 Hz: mode = 100
    uint8_t reg[2];

    switch (mode) {
    case 0:
        // single shot
        // after measure, transits to power-down mode automatically
        reg[1] = 0x01;
        break;
    case 10:
        // 10Hz continuous
        reg[1] = 0x02;
        break;
    case 20:
        // 20Hz continuous
        reg[1] = 0x04;
        break;
    case 50:
        // 50Hz continuous
        reg[1] = 0x06;
        break;
    case 100:
        // 100Hz continuous
        reg[1] = 0x08;
        break;
    default:
#ifndef NDEBUG
        printf("error at icm20948_set_mag_mode: wrong mode %d\n", mode);
#endif
        return;
    }

    reg[0] = AK09916_CNTL2;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_mag, reg, 2, false, config->i2cTimeout_us);

    return;
}

void icm20948_read_raw_accel(icm20948_config_t* config, int16_t accel[3]) {
    config->semphrTake();

    uint8_t buf[6];

    // accel: 2 bytes each axis
    uint8_t reg = ACCEL_XOUT_H;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, &reg, 1, true, config->i2cTimeout_us);
    i2c_read_timeout_per_char_us(config->i2c, config->addr_accel_gyro, buf, 6, false, config->i2cTimeout_us);

    config->semphrGive();

    for (uint8_t i = 0; i < 3; i++) accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);

    return;
}

void icm20948_read_raw_gyro(icm20948_config_t* config, int16_t gyro[3]) {
    config->semphrTake();

    uint8_t buf[6];

    // gyro: 2byte each axis
    uint8_t reg = GYRO_XOUT_H;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, &reg, 1, true, config->i2cTimeout_us);
    i2c_read_timeout_per_char_us(config->i2c, config->addr_accel_gyro, buf, 6, false, config->i2cTimeout_us);

    config->semphrGive();

    for (uint8_t i = 0; i < 3; i++) gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);

    return;
}

void icm20948_read_raw_temp(icm20948_config_t* config, int16_t* temp) {
    config->semphrTake();

    uint8_t reg = TEMP_OUT_H, buf[2];
    i2c_write_timeout_per_char_us(config->i2c, config->addr_accel_gyro, &reg, 1, true, config->i2cTimeout_us);
    i2c_read_timeout_per_char_us(config->i2c, config->addr_accel_gyro, buf, 2, false, config->i2cTimeout_us);

    config->semphrGive();

    *temp = (buf[0] << 8 | buf[1]);

    return;
}

void icm20948_read_raw_mag(icm20948_config_t* config, int16_t mag[3]) {
    config->semphrTake();

    uint8_t buf[8];

    uint8_t reg = AK09916_XOUT_L;
    i2c_write_timeout_per_char_us(config->i2c, config->addr_mag, &reg, 1, true, config->i2cTimeout_us);
    i2c_read_timeout_per_char_us(config->i2c, config->addr_mag, buf, 8, false, config->i2cTimeout_us);

    config->semphrGive();

    for (int i = 0; i < 3; i++) mag[i] = (buf[(i * 2) + 1] << 8 | buf[(i * 2)]);

#ifndef NDEBUG
    if ((buf[6] & 0x08) == 0x08) printf("mag: ST1: Sensor overflow\n");

    // printf below works only if we read 0x10
    //if ((buf[0] & 0x01) == 0x01) printf("mag: ST1: Data overrun\n");
    //if ((buf[0] & 0x02) != 0x02) printf("mag: ST1: Data is NOT ready\n");
#endif

    return;
}

void icm20948_cal_gyro(icm20948_config_t* config, int16_t gyro_bias[3]) {
    int16_t buf[3] = { 0 };
    int32_t bias[3] = { 0 };

    for (uint8_t i = 0; i < 200; i++) {
        icm20948_read_raw_gyro(config, buf);
        for (uint8_t j = 0; j < 3; j++) {
            bias[j] += buf[j];
        }
        config->delayFunc(25);
    }
    for (uint8_t i = 0; i < 3; i++) gyro_bias[i] = (int16_t)(bias[i] / 200);

    return;
}

void icm20948_read_cal_gyro(icm20948_config_t* config, int16_t gyro[3], int16_t bias[3]) {
    icm20948_read_raw_gyro(config, gyro);
    for (uint8_t i = 0; i < 3; i++) gyro[i] -= bias[i];
    return;
}

void icm20948_cal_accel(icm20948_config_t* config, int16_t accel_bias[3]) {
    int16_t buf[3] = { 0 };
    int32_t bias[3] = { 0 };

    for (uint8_t i = 0; i < 200; i++) {
        icm20948_read_raw_accel(config, buf);
        for (uint8_t j = 0; j < 3; j++) {
            if (j == 2) bias[j] += (buf[j] - 16384);
            else bias[j] += buf[j];
        }
        config->delayFunc(25);
    }
    for (uint8_t i = 0; i < 3; i++) accel_bias[i] = (int16_t)(bias[i] / 200);
    return;
}

void icm20948_read_cal_accel(icm20948_config_t* config, int16_t accel[3], int16_t bias[3]) {
    icm20948_read_raw_accel(config, accel);
    for (uint8_t i = 0; i < 3; i++) accel[i] -= bias[i];
    return;
}

void icm20948_cal_mag_simple(icm20948_config_t* config, int16_t mag_bias[3]) {
    int16_t buf[3] = { 0 }, max[3] = { 0 }, min[3] = { 0 };
#ifndef NDEBUG
    printf("mag calibration: \nswing sensor for 360 deg\n");
#endif
    for (uint8_t i = 0; i < 1000; i++) {
        icm20948_read_raw_mag(config, buf);
        for (uint8_t j = 0; j < 3; j++) {
            if (buf[j] > max[j]) max[j] = buf[j];
            if (buf[j] < min[j]) min[j] = buf[j];
        }
        config->delayFunc(10);
    }
    for (uint8_t i = 0; i < 3; i++) mag_bias[i] = (max[i] + min[i]) / 2;
    return;
}

void icm20948_read_cal_mag(icm20948_config_t* config, int16_t mag[3], int16_t bias[3]) {
    icm20948_read_raw_mag(config, mag);
    for (uint8_t i = 0; i < 3; i++) mag[i] -= bias[i];
    return;
}

void icm20948_read_temp_c(icm20948_config_t* config, float* temp) {
    int16_t tmp;
    icm20948_read_raw_temp(config, &tmp);
    // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21
    *temp = (((float)tmp - 21.0f) / 333.87) + 21.0f;
    return;
}
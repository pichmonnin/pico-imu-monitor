#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <pico/error.h>
#include <pico/stdio.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>
#include <pico/types.h>
#include <string>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <array>
#define MENU_CLEAR "\033[2J\033[H" 

//IMU Micro

#define ICM_ADDRESS 0x68
#define REG_ACCEL_CONFIG0 0x1B
#define REG_GYRO_CONFIG0 0x1C
#define REG_PWR_MGMTO 0x10
#define ACC_X 0x00
#define REG_WHO_AM_I 0x72

//Base Register Address 

#define IPREG_SYS1_BASE 0xA400
#define IPREG_SYS2_BASE 0xA500

//LPF Data 
#define GYRO_UI_LPFBW_SEL_ADDR (IPREG_SYS1_BASE + 0xAC)
#define ACCEL_UI_LPFBW_SEL_ADDR (IPREG_SYS2_BASE  + 0x83)

#define IPREG_ADDR_15_8 0x7C // high byte of 16-bit IPREG Address 
#define IPREG_ADDR_0_7 0x7D // Low byte of 16-bit IPREG Address 
#define IREG_DATA 0x7E //data byte read/write 
#define REG_MISC2 0x7F // status : bit 0 = IREG_DATA ready

//IMU Micro

constexpr float G_TO_MS2 = 9.8066f;
constexpr float DEG_TO_RAD = 3.14159265 / 180.0f; 
constexpr float Gyro_Scale = 16.384f;
constexpr float Accel_Scale = 2048.0f;
const int LED_PIN = 25;
const int SDA_PIN = 0; 
const int SCL_PIN = 1;


struct GyroBias {
    float bgyro_x , bgyro_y , bgyro_z;
};

struct GyroCalibratedData{
    float cgyro_x , cgyro_y , cgyro_z;
};

struct RawIMUData{ 
    float raw_acc_x , raw_acc_y , raw_acc_z;
    float raw_gyro_x , raw_gyro_y , raw_gyro_z;
};

struct DPSIMUData{
    float dps_accel_x , dps_accel_y , dps_accel_z; 
    float dps_gyro_x  , dps_gyro_y , dps_gyro_z;
};

struct IMUDataSI {
    float accel_si_x , accel_si_y , accel_si_z; 
    float gyro_si_x , gyro_si_y , gyro_si_z;
};
//IMU Driver

//Define the register of the sensor
void icm_write_reg(uint8_t reg , uint8_t value){
    uint8_t packet[2] = {reg , value};
    i2c_write_blocking(i2c0 , ICM_ADDRESS  , packet , 2 , false);
}


uint8_t read_imu_reg(uint8_t reg){
    uint8_t value = 0; 
    i2c_write_blocking(i2c0 , ICM_ADDRESS , &reg  , 1 , true);
    i2c_read_blocking(i2c0 , ICM_ADDRESS , &value , 1 , false);
    return value;
//
}
void icm_write_ipreg(uint16_t addr , uint8_t value)
{
    icm_write_reg(IPREG_ADDR_15_8 , static_cast<uint8_t>(addr >> 8));
    icm_write_reg(IPREG_ADDR_0_7 , static_cast<uint8_t>(addr & 0xFF));

    icm_write_reg(IREG_DATA , value);

    for (int  i = 0 ; i < 20 ; i++){
        uint8_t misc2 = read_imu_reg(REG_MISC2);
        if (misc2 & 0x01) break;
        sleep_us(10);
    }
}
uint8_t icm_read_ipreg(uint16_t addr){
    icm_write_reg(IPREG_ADDR_15_8 , static_cast<uint8_t>(addr >> 8));
    icm_write_reg(IPREG_ADDR_0_7 , static_cast<uint8_t>(addr & 0xFF));

    for (int i = 0 ; i < 20 ; i++){
        uint8_t misc2 = read_imu_reg(REG_MISC2);
        if (misc2 & 0x01) break; 
        sleep_us(10);
    }
    return read_imu_reg(IREG_DATA);
}

void print_lpf_setting(){
    uint8_t gyro_lpf = icm_read_ipreg(GYRO_UI_LPFBW_SEL_ADDR) & 0x07; 
    uint8_t accel_lpf = icm_read_ipreg(ACCEL_UI_LPFBW_SEL_ADDR) & 0x07;
    std::cout << "Gyro_UI_LPFBW_SEL:" <<static_cast<int>(gyro_lpf) << std::endl;
    std::cout << "ACCEL_UI_LPFBW_SEL:" <<static_cast<int>(accel_lpf) << std::endl;
}

void read_filtered_data(uint8_t *buf12){
    uint8_t reg = ACC_X;
    i2c_write_blocking(i2c0, ICM_ADDRESS ,&reg, 1, true);
    i2c_read_blocking(i2c0,ICM_ADDRESS, buf12, 12, false);
}

void imu_cfg(){
    uint8_t accel_cfg = read_imu_reg(REG_ACCEL_CONFIG0);
    uint8_t gyro_cfg  = read_imu_reg(REG_GYRO_CONFIG0);
}
/// Imu_Driver

RawIMUData gather_data(const uint8_t *buf12){
    int16_t accel_x_g = static_cast<int16_t>((buf12[1] << 8 ) | buf12[0]);
    int16_t accel_y_g = static_cast<int16_t>((buf12[3] << 8 ) | buf12[2]);
    int16_t accel_z_g = static_cast<int16_t>((buf12[5] << 8 ) | buf12[4]);
    
    int16_t gyro_x_g = static_cast<int16_t>((buf12[7] << 8 ) | buf12[6]);
    int16_t gyro_y_g = static_cast<int16_t>((buf12[9] << 8 ) | buf12[8]);
    int16_t gyro_z_g = static_cast<int16_t>((buf12[11] << 8 ) | buf12[10]);

    RawIMUData data; 
    data.raw_acc_x = accel_x_g;
    data.raw_acc_y = accel_y_g;
    data.raw_acc_z = accel_z_g;
    data.raw_gyro_x = gyro_x_g;
    data.raw_gyro_y = gyro_y_g;
    data.raw_gyro_z = gyro_z_g;
    return data;
}

DPSIMUData RawDataToDPSData(RawIMUData& raw_data){
    DPSIMUData DPS_data; 
    DPS_data.dps_accel_x = raw_data.raw_acc_x / Accel_Scale ;
    DPS_data.dps_accel_y = raw_data.raw_acc_y / Accel_Scale;
    DPS_data.dps_accel_z = raw_data.raw_acc_z / Accel_Scale;
    
    DPS_data.dps_gyro_x = raw_data.raw_gyro_x / Gyro_Scale;
    DPS_data.dps_gyro_y = raw_data.raw_gyro_y / Gyro_Scale;
    DPS_data.dps_gyro_z = raw_data.raw_gyro_z / Gyro_Scale;
    return DPS_data;
}

IMUDataSI DPSDataToSIData(DPSIMUData& dps_data){
    IMUDataSI SI_data;
    SI_data.accel_si_x = dps_data.dps_accel_x * G_TO_MS2;
    SI_data.accel_si_y = dps_data.dps_accel_y * G_TO_MS2;
    SI_data.accel_si_z = dps_data.dps_accel_z * G_TO_MS2;
    
    SI_data.gyro_si_x = dps_data.dps_gyro_x * DEG_TO_RAD;
    SI_data.gyro_si_y = dps_data.dps_gyro_y * DEG_TO_RAD;
    SI_data.gyro_si_z = dps_data.dps_gyro_z * DEG_TO_RAD;

    return SI_data;
}
void print_SI_IMU(const uint8_t *buf12){
    auto raw_data = gather_data(buf12);
    auto dps_data = RawDataToDPSData(raw_data);
    auto si_data = DPSDataToSIData(dps_data);
            std::cout << "\r" << "Accel :" << std::fixed  << std::setprecision(2) 
                << std::setw(7) <<"(X) :"<< si_data.accel_si_x  << " "
                << std::setw(7) <<"(Y) :"<< si_data.accel_si_y << " "
                << std::setw(7) <<"(Z) :"<< si_data.accel_si_z << " | "
                <<"Gyro : "
                << std::setw(7) << "(X) :"<< si_data.gyro_si_x <<  " "
                << std::setw(7) << "(Y) :"<< si_data.gyro_si_y << " "
                << std::setw(7) << "(Z) :"<< si_data.gyro_si_z 
                << "      " << std::flush; 
}
void print_menu(){
    printf(MENU_CLEAR);
    printf("=== PICO CONTROL MENU ===\r\n");
    printf("1. Show sensor data\r\n");
    printf("q. Quit to menu\r\n");
    printf("========================\r\n");
    printf("Select option: ");
}

void GyroCalibration(){
   RawIMUData data;
   const uint32_t sample = 10000;
   int64_t sum_x = 0 , sum_y = 0 , sum_z = 0 ;
   printf("Keep the sensor still.....\n");
   sleep_ms(2000);

   absolute_time_t start = get_absolute_time();

   for (uint32_t i =0 ; i < sample ; i++){
       sum_x += data.raw_gyro_x;
       sum_y += data.raw_gyro_y;
       sum_z += data.raw_gyro_z;

       sleep_us(1000);
   }
   GyroBias data_bias; 
   data_bias.bgyro_x = static_cast<float>(sum_x) / static_cast<float>(sample);
   data_bias.bgyro_y = static_cast<float>(sum_y) /static_cast<float>(sample); 
   data_bias.bgyro_z = static_cast<float>(sum_z) / static_cast<float>(sample);
   //Getting the Scale Factor
   DPSIMUData data_gyro_s;
   std::array<std::array<float, 3>, 3 >Scale_gyro_inv = {{
       {1/data_gyro_s.dps_gyro_x , 0.0 , 0.0},
       {0.0 , 1/data_gyro_s.dps_gyro_y , 0.0},
       {0.0 , 0.0 , 1/data_gyro_s.dps_gyro_z}
   }};

    //Getting the Misalignment Matrix
   std::array<std::array<float, 3>, 3 > M_gyro_bias = {{
        {1.0f , 0.0f , 0.0f},
        {0.0f , 1.0f , 0.0f},
        {0.0f , 0.0f , 1.0f}
   }};
   GyroCalibratedData cal_data;
   cal_data.cgyro_x = Scale_gyro_inv[0][0] * M_gyro_bias[0][0] * data.raw_gyro_x  - data_bias.bgyro_x; 
   cal_data.cgyro_y = Scale_gyro_inv[1][1] * M_gyro_bias[1][1] * data.raw_gyro_y  - data_bias.bgyro_y; 
   cal_data.cgyro_z = Scale_gyro_inv[2][2] * M_gyro_bias[2][2] * data.raw_gyro_z  - data_bias.bgyro_z; 
}

int main(){
    stdio_init_all();
    i2c_init(i2c0 , 400 * 1000 ); //400 Khz
    gpio_set_function(SDA_PIN , GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN , GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    sleep_ms(50);

    icm_write_reg(REG_PWR_MGMTO,0x0F);
    icm_write_reg(REG_ACCEL_CONFIG0 , 0x19);
    icm_write_reg(REG_GYRO_CONFIG0 , 0x19);
    sleep_ms(500);
    while(!stdio_usb_connected()){
      sleep_ms(100);
    }
    sleep_ms(300);
    uint8_t buf12[12];
    bool show_sensor = false;
    print_menu();

//Safety Check

    while (true)
    {
        read_filtered_data(buf12);
        sleep_ms(500);
        imu_cfg();
        int c = getchar_timeout_us(0);
        if (c  != PICO_ERROR_TIMEOUT){
            char cmd = static_cast<char>(c);
            if (cmd =='1'){
                show_sensor = true;
            }
            if (cmd =='q' || cmd == 'Q'){
                show_sensor = false;
                print_menu();
            }
        }
    if (show_sensor){
        print_SI_IMU(buf12);
    }
    sleep_ms(100);

    }
        return 0;
}

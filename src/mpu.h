#include <Arduino.h>
#include "MPU9250.h"
#pragma once

class SkateMPU {
    public:
        void setup();
        void loop() {
            if (_mpu.update()) 
                print_roll_pitch_yaw();
        }
    private:
        MPU9250 _mpu;
        void print_roll_pitch_yaw() {
            Serial.print("Yaw, Pitch, Roll: ");
            Serial.print(_mpu.getYaw(), 2);
            Serial.print(", ");
            Serial.print(_mpu.getPitch(), 2);
            Serial.print(", ");
            Serial.println(_mpu.getRoll(), 2);
        }
};

inline void SkateMPU::setup() {
    Wire.begin();
    delay(2000);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!_mpu.setup(0x68, setting)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(2000);
        }
    }
}


void setup_mpu() {
    
}


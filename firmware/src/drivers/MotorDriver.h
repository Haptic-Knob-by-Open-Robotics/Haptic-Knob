#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

class MotorDriver {
    public:
        MotorDriver(int uh, int ul, int vh, int vl, int wh, int wl, int en);

        bool init(float v_supply, float v_limit);
        void enable();
        void disable();

        void setPWM(float ua, float ub, float uc);

        BLDCDriver6PWM& raw();

    private:
        BLDCDriver6PWM _driver;
        bool _ok = false;
};
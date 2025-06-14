// #include "HAS_TelescopeController.h"
#include "config.h"
#include "io.h"
#include "encoder.h"
bool g_decLimLo = false;
bool g_decLimHi = false;

/// @brief Triggered when the lower declination limit is reached.
void decLimLoISR(){
    g_decLimLo = true;
}

/// @brief Triggered when the upper declination limit is reached.
void decLimHiISR(){
    g_decLimHi = true;
}

namespace io{
    /// @brief get the current position in degrees in the motor reference frame.
    /// @param ra The RA stepper motor.
    /// @param dec The Dec stepper motor.
    pos::Position getMotorPositions(Stepper& ra, Stepper& dec){
        pos::Position pos;
        pos.frame = MOTOR;
        pos.ra = ra.getPulseCount()/ra.getPulsesPerDeg();
        pos.dec = -dec.getPulseCount()/dec.getPulsesPerDeg();
        return pos;
}

    /// @brief set up the limit switches.
    void setupLimits(){
        pinMode(DI_DEC_LIM_LO, INPUT_PULLUP);
        pinMode(DI_DEC_LIM_HI, INPUT_PULLUP);
        pinMode(DI_RA_LIM_IDX, INPUT_PULLUP);
        // attachInterrupt(digitalPinToInterrupt(DI_DEC_LIM_LO), decLimLoISR, LOW);
        // attachInterrupt(digitalPinToInterrupt(DI_DEC_LIM_HI), decLimHiISR, LOW);
    }

    /// @brief stop the declination motor if a limit switch is triggered.
    /// @param dec the declination stepper motor.
    void limitStop(Stepper& dec){
        if(digitalRead(DI_DEC_LIM_LO) && dec.getDirection() == DREVERSE){
            dec.stop();
            // g_decLimLo = false;
        }
        if(digitalRead(DI_DEC_LIM_HI) && dec.getDirection() == DFORWARD){
            dec.stop();
            // g_decLimHi = false;
        }

    }

    bool isHome(){
        // The following line does not match the original logic. 
        // The re-wiring has changed the input signals.
        // This code does give the correct DEC HOME, but the RA link is wrong.
        if(digitalRead(DI_DEC_LIM_LO) && digitalRead(DI_RA_LIM_IDX)) return true; // second test was NOT
        else return false;
    }

    bool decLimCheck(int dir){
        if(dir == DREVERSE && digitalRead(DI_DEC_LIM_LO)) return true;
        if(dir == DFORWARD && digitalRead(DI_DEC_LIM_HI)) return true;
        return false;
    }
}
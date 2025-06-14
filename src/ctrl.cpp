#include <Arduino.h>
#include "pos.h"
#include "ctrl.h"
#include "enums.h"
#include "utils.h"
#include "stepper.h"
// #include "io.h"

// extern bool g_isSlewing;

namespace ctrl {
    bool isHoming = false;
  // pos::Position localPosition;
    

    /// @brief check if the target position is reachable.
    /// @param target the target position
    /// @return a string containing the error message if the target is not reachable, or "0" if it is.
    String checkTargetReachable(pos::Position target) {
        String str;
        //target reachable
        if (true) str = "0";
        // target below horizon
        else if (false) str = "1 ERROR: OBJECT BELOW HORIZON#";
        // target not reachable
        else str = "2 ERROR: OBJECT NOT REACHABLE#";
        return str;
    }

    scopeState getScopeStatus(io::Stepper& ra, io::Stepper& dec){
        if (!ra.getEnabled() && !dec.getEnabled()) return IDLE;
        else if (!dec.getEnabled() && ra.getFrequency()== trackRateHz && ra.getDirection() == FORWARD) return TRACKING;
        else return SLEWING;
    }


    /// @brief Stop all movement of the telescope
    /// @param ra the RA stepper motor
    /// @param dec the Dec stepper motor
    void stopAllMovement(io::Stepper& ra, io::Stepper& dec) {
        ra.stop();
        dec.stop();
        // io::stopMotors();
    }

    /// @brief Stop the motors if the current location is below the horizon
    /// @param currentLocation the current location of the telescope
    /// @param ra the RA stepper motor
    /// @param dec the Dec stepper motor
    void horizonStop(pos::FrameSet& currentLocation, io::Stepper& ra, io::Stepper& dec, autoManualMode ctrlMode){
        // const double threshold = 0.5;
        static int raTripDir = ra.getDirection();
        static int decTripDir = dec.getDirection();
        static bool isBelowHorizon = false;
        if (ctrlMode == MANUAL) return;
        if (currentLocation.getCoord(ALTAZ, ALT) < minAltitude) {
            if (!isBelowHorizon) {
                raTripDir = ra.getDirection();
                decTripDir = dec.getDirection();
                isBelowHorizon = true;
            }
            if (raTripDir == ra.getDirection()) ra.stop();
            if (decTripDir == dec.getDirection()) dec.stop();
        }
        else {
            isBelowHorizon = false;
        }
    }

    /// @brief Move the telescope to the target position
    /// @param currentLocation the current location of the telescope
    /// @param targetPosition the target position
    /// @param ra the RA stepper
    /// @param dec the DEC stepper
    void move(pos::FrameSet& currentLocation, pos::Position& targetPosition, io::Stepper& ra, io::Stepper& dec) {
        ra.setFrequency(ra.getMaxFrequency());
        dec.setFrequency(ra.getMaxFrequency());
        double deltaRa = -wrap180(targetPosition.ra - currentLocation.getCoord(SKY, RA));
        double deltaDec = -wrap180(targetPosition.dec - currentLocation.getCoord(SKY, DECL));
        double trackingOffset = abs(deltaRa)*trackRateHz/double(ra.getFrequency());
        double deltaRaTotal = wrap180(deltaRa + trackingOffset);
        ra.runAngle(deltaRaTotal);
        dec.runAngle(deltaDec);
    }

    void moveHome(io::Stepper& ra, io::Stepper& dec){
        //Serial.println("ra.getPulseCount: " + String(ra.getPulseCount()));
        // if (ra.getPulseCount() == 0) return;
        // direction dir = (ra.getPulseCount() > 0) ? FORWARD : REVERSE;
        dec.setFrequency(dec.getMaxFrequency());
        dec.run(FORWARD);
        isHoming = true;
    }
    void homeStop(io::Stepper& ra, io::Stepper& dec){
        if(isHoming){
            if (digitalRead(DI_DEC_LIM_LO)) dec.stop();
            if (digitalRead(DI_RA_LIM_IDX)) ra.stop();
            if(digitalRead(DI_DEC_LIM_LO) && !digitalRead(DI_RA_LIM_IDX)) isHoming = false;
        }
    

    }

    bool getHoming(){return isHoming;}
}
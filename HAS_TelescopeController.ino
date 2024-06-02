/**@file HAS_TelescopeController.ino */

#include <avr/wdt.h>
#include "src/config.h"
#include "src/comms.h"
#include "src/ctrl.h"
#include "src/pos.h"
#include "src/io.h"
#include "src/stepper.h"
#include "src/ui.h"
#include "src/utils.h"

//TODO: move variables to appropriate modules. Ideally only setup and loop should be in this file
String buffer ="";
command currentCmd;
String coordString;
// bool g_isSlewing = false;
bool initialSync = false;
uint32_t maxFreqRa = 30000;
uint32_t maxFreqDec = 50000;
stepperCalibration raCal = {29918.22352,-0.4805,32558};
stepperCalibration decCal = {99603.48705,-1.2116,74717};
namespace pos{
    const Position homePosition = {BASE, 0.00, 47.2536};
    Position targetPosition = {SKY, 0, 0};
    FrameSet currentLocation;
}

namespace ctrl{
    autoManualMode ctrlMode;
    trackMode trkMode;
}
io::Stepper raStp;
io::Stepper decStp;
ui::HandheldController hhc;
ui::Display disp;
////////////////////////////////////////////////////////////////////////////////
/// Main Program
////////////////////////////////////////////////////////////////////////////////


/// @brief Main program entry point.
void setup() {
    io::setupLimits();
    ctrl::ctrlMode = MANUAL;
    ctrl::trkMode = NO_TRACK;
    hhc.initButtons(AI_POT_SPEED, DI_MUX_SIG, DO_MUX_ADDR_A, DO_MUX_ADDR_B, DO_MUX_ADDR_C);
    disp.init();
    Serial.begin(9600);
    // Serial1.begin(9600);
    wdt_enable(WDTO_2S); // Enable Watchdog Timer, 8s

    raStp.init(DO_RA_STP_DIR, PWM_RA_STP_PUL, maxFreqRa, false, raCal);
    decStp.init(DO_DEC_STP_DIR, PWM_DEC_STP_PUL, maxFreqDec, true, decCal);
}

/// @brief Main loop
void loop() {

    pos::SiderealTime::update(); // Update the sidereal time
    pos::currentLocation.updateSiderealTime(pos::SiderealTime::getValue()); // Pass the sidereal time to the current location
    pos::currentLocation.updatePosition(io::getMotorPositions(raStp, decStp)); // Update the current location from the motor positions
    hhc.updateButtons();
    disp.updateStates(hhc,initialSync,io::isHome());
    disp.show(hhc,pos::currentLocation.getPosition(SKY),ctrl::getHoming());
    ctrl::ctrlMode = (disp.getAutoManState()) ? AUTO : MANUAL;
    ctrl::trkMode = (disp.getTrackState()) ? TRACK : NO_TRACK;

    long int rampingCountMax = 500000;

    // DEC Plus ramping variables
    bool rampingActiveDECPlus = false;
    bool rampingTriggerDECPlus = false;
    long int rampingCounterDECPlus = 0;
    
    double slewRateHzDEC = 0;

    // DEC Minus ramping variables
    bool rampingActiveDECMinus = false;
    bool rampingTriggerDECMinus = false;
    long int rampingCounterDECMinus = 0;
    
    // RA Plus ramping variables
    bool rampingActiveRAPlus = false;
    bool rampingTriggerRAPlus = false;
    long int rampingCounterRAPlus = 0;
    
    double slewRateHzRA = 0;

    // DEC Minus ramping variables
    bool rampingActiveRAMinus = false;
    bool rampingTriggerRAMinus = false;
    long int rampingCounterRAMinus = 0;

    if(ctrl::trkMode == TRACK && ctrl::getScopeStatus(raStp, decStp) == IDLE){
            raStp.run(FORWARD, ctrl::trackRateHz);
    }

    if (comms::readStringUntilChar(buffer, '#')) {
        currentCmd = comms::parseCommand(buffer);
        switch (currentCmd) {
            case GET_RA:
            comms::sendReply(comms::double2RaStr(pos::currentLocation.getCoord(SKY, RA)));
            buffer = "";
            break;

            case GET_DEC:
            comms::sendReply(comms::double2DecStr(pos::currentLocation.getCoord(SKY, DECL)));
            buffer = "";
            break;

            case SYNC_POSITION:
            if (!initialSync){
                pos::SiderealTime::sync(pos::homePosition, pos::targetPosition);
                pos::currentLocation.initialiseSiderealTime(pos::SiderealTime::getValue());
                initialSync = true;
                disp.setDisplayMode(COORDS);
                // disp.setTrackState(true);
            }
            pos::currentLocation.syncTo(pos::targetPosition);

            comms::sendReply("SYNCED TO " + 
                            comms::double2RaStr(pos::currentLocation.getCoord(SKY, RA)) + " " +
                            comms::double2DecStr(pos::currentLocation.getCoord(SKY, DECL)));
            buffer = "";
            break;

            case SLEW_TO_TARGET:
            switch (ctrl::ctrlMode){
                case AUTO:
                comms::sendReply(ctrl::checkTargetReachable(pos::targetPosition));
                ctrl::move(pos::currentLocation, pos::targetPosition,raStp, decStp);
                // g_isSlewing = true;
                break;
                case MANUAL:
                comms::sendReply("2: Telescope in Manual mode");
                break;
            }
            buffer = "";
            break;

            case STOP_SLEW:
            switch (ctrl::ctrlMode){
                case AUTO:
                ctrl::stopAllMovement(raStp, decStp);
                break;
                case MANUAL:
                break;
            }
            buffer = "";
            break;

            case SET_TARGET_RA:
            coordString = comms::extractCoord(buffer);
            pos::targetPosition.ra = comms::raStr2Double(coordString);
            comms::sendReply("1"); //add checking of of parsed string later
            buffer = "";
            Serial1.println("SET_TARGET_RA:");
            Serial1.println(pos::targetPosition.ra);
            break;

            case SET_TARGET_DEC:
            coordString = comms::extractCoord(buffer);
            pos::targetPosition.dec = comms::decStr2Double(coordString);
            comms::sendReply("1"); //add checking of of parsed string later
            buffer = "";
            Serial1.println("SET_TARGET_DEC:");
            Serial1.println(pos::targetPosition.dec);
            break;

            default:
            // Serial1.println("WARN: invalid command received");
            buffer = "";
            break;
        }
    
    }

    if(ctrl::ctrlMode == MANUAL){
        double slewRateHz = 50000;
        double maxSlewRateHz = 50000;

        if (hhc.getPotValue() < 256) {
            slewRateHz = 10000;
        } else if (hhc.getPotValue() < 512) {
            slewRateHz = 20000;
        } else if (hhc.getPotValue() < 768) {
            slewRateHz = 35000;
        } else {
            slewRateHz = 50000;
        }
        
        // 0.095*hhc.getPotValue()*hhc.getPotValue(); //quadradic curve allows for fine control at low end, while still allowing fast slew at high end
        DisplayMode dispMode = disp.getDisplayMode();
        // digitalWrite(DO_RA_EN,isRaPul);
        // digitalWrite(DO_DEC_EN,isDecPul);
        if(ctrl::getHoming())disp.setTrackState(false);
        if(hhc.getBtnGoToRise()) {
            ctrl::moveHome(raStp,decStp);
            tone(PWM_BZR,NOTE_C6,BEEP_TIME);
        }

        // Set ramping rates, if required.

        // DEC Plus ramping
        if(hhc.getBtnDecPlusRise()) {
            rampingActiveDECPlus = true;
        }
        if(rampingTriggerDECPlus && !rampingActiveDECPlus) {
            rampingActiveDECPlus = true;
            rampingTriggerDECPlus = false;
            rampingCounterDECPlus = rampingCountMax;
        }
        if(rampingActiveDECPlus && (rampingCounterDECPlus > 1)) {
            slewRateHzDEC = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterDECPlus);
            rampingCounterDECPlus = rampingCounterDECPlus - 1;
        } else if(rampingActiveDECPlus) {
            rampingActiveDECPlus = false;
            slewRateHzDEC = maxSlewRateHz;
        }

        // DEC Minus ramping
        if(hhc.getBtnDecMinusRise()) {
            rampingActiveDECMinus = true;
        }
        if(rampingTriggerDECMinus && !rampingActiveDECMinus) {
            rampingActiveDECMinus = true;
            rampingTriggerDECMinus = false;
            rampingCounterDECMinus = rampingCountMax;
        }
        if(rampingActiveDECMinus && (rampingCounterDECMinus > 1)) {
            slewRateHzDEC = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterDECMinus);
            rampingCounterDECMinus = rampingCounterDECMinus - 1;
        } else if(rampingActiveDECMinus) {
            rampingActiveDECMinus = false;
            slewRateHzDEC = maxSlewRateHz;
        }

       // RA Plus ramping
        if(hhc.getBtnRAPlusRise()) {
            rampingActiveRAPlus = true;
        }
        if(rampingTriggerRAPlus && !rampingActiveRAPlus) {
            rampingActiveRAPlus = true;
            rampingTriggerRAPlus = false;
            rampingCounterRAPlus = rampingCountMax;
        }
        if(rampingActiveRAPlus && (rampingCounterRAPlus > 1)) {
            slewRateHzRA = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterRAPlus);
            rampingCounterRAPlus = rampingCounterRAPlus - 1;
        } else if(rampingActiveRAPlus) {
            rampingActiveRAPlus = false;
            slewRateHzRA = maxSlewRateHz;
        }

        // RA Minus ramping
        if(hhc.getBtnRAMinusRise()) {
            rampingActiveRAMinus = true;
        }
        if(rampingTriggerRAMinus && !rampingActiveRAMinus) {
            rampingActiveRAMinus = true;
            rampingTriggerRAMinus = false;
            rampingCounterRAMinus = rampingCountMax;
        }
        if(rampingActiveRAMinus && (rampingCounterRAMinus > 1)) {
            slewRateHzRA = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterRAMinus);
            rampingCounterRAMinus = rampingCounterRAMinus - 1;
        } else if(rampingActiveRAMinus) {
            rampingActiveRAMinus = false;
            slewRateHzRA = maxSlewRateHz;
        }
        // ramping code ends

        if(hhc.getBtnRaPlus() && (dispMode == COORDS || dispMode == SYNC) ){
            if (ctrl::getHoming()) raStp.run(REVERSE, raStp.getMaxFrequency());
            else raStp.run(REVERSE, slewRateHzRA/1.5);
        }
        else if(hhc.getBtnRaMinus() && (dispMode == COORDS || dispMode == SYNC)){
            if (ctrl::getHoming()) raStp.run(FORWARD, raStp.getMaxFrequency());
            else raStp.run(FORWARD, slewRateHzRA/1.5);
        }
        else if(ctrl::trkMode == TRACK){
            raStp.run(FORWARD, ctrl::trackRateHz);
        }
        else if (!ctrl::getHoming()){
            raStp.stop();
        }

        if(!hhc.getBtnDecPlus() && (dispMode == COORDS || dispMode == SYNC)){
            if (ctrl::getHoming()) decStp.run(FORWARD, decStp.getMaxFrequency());
            else decStp.run(FORWARD, slewRateHzDEC);
        }
        else if(hhc.getBtnDecMinus()&& (dispMode == COORDS || dispMode == SYNC)){
            // if (ctrl::getHoming()) decStp.run(REVERSE, decStp.getMaxFrequency());
            decStp.run(REVERSE, slewRateHzDEC);
        }
        else if (!ctrl::getHoming()){
            decStp.stop();
        }
    }
    
    static unsigned long prevMillis = millis();
    if(millis()-prevMillis>=1500){
        // Serial.println("AUTO: " + String(disp.getAutoManState()));
        // Serial.println("TRACK: " + String(disp.getTrackState()));
        // Serial.println("RUN_RA: " + String(raStp.getEnabled()));
        // Serial.println("RUN_DEC: " + String(decStp.getEnabled()));
        // Serial.println("RUN_DEC: " + String(hhc.getBtnDecMinus()));
        Serial.println("rampingTriggerRA: " + String(rampingTriggerRA));
        Serial.println("rampingTriggerDEC: " + String(rampingTriggerDEC));
        Serial.println("rampingCounterRA: " + String(rampingCounterRA));
        Serial.println("rampingCounterDEC: " + String(rampingCounterDEC));
        Serial.println("rampingActiveRA: " + String(rampingActiveRA));
        Serial.println("rampingActiveDEC: " + String(rampingActiveDEC));
        Serial.println("slewRateHzRA: " + String(slewRateHzRA));
        Serial.println("slewRateHzDEC: " + String(slewRateHzDEC));
        Serial.println("----------------");
        
        prevMillis = millis();
    }

    // Serial.println(disp.getAutoManState());
    ctrl::homeStop(raStp,decStp);
    // // // // // // // // // SAFTEY LIMITS // // // // // // // // // // // //
    // ctrl::horizonStop(pos::currentLocation, raStp, decStp, ctrl::ctrlMode);
    io::limitStop(decStp); //Should be last function called in loop to ensure limit switches will stop motors
    // // // // // // // // // // // // // // // // // // // // // // // // // /
    wdt_reset();
}

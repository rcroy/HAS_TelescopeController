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
    Position EncoderPosition = {SKY, 0, 0}; // This is used to store the position from the encoders.
}

namespace ctrl{
    autoManualMode ctrlMode;
    trackMode trkMode;
}
io::Stepper raStp;
io::Stepper decStp;
io::Encoder EncRA(DI_RA_ENC_A, DI_RA_ENC_B, 8192); // maxEdges is 2048 pulses x 4.
io::Encoder EncDEC(DI_DEC_ENC_A, DI_DEC_ENC_B, 25464); // maxEdges per 360 degrees.

ui::HandheldController hhc;
ui::Display disp;

// Declare a global pointers for the Encoder objects so that we can use them in the ISR.
io::Encoder* RA_encPtr = nullptr;
io::Encoder* DEC_encPtr = nullptr;

// Create Free Functions for the Encoder Interrupt Service Request functions.
// These functions will be called when the encoders detect a change in state.
void RA_countPulsesISR() {
    if (RA_encPtr) RA_encPtr->countPulses();
}

void DEC_countPulsesISR() {
    if (DEC_encPtr) DEC_encPtr->countPulses();
}

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
    wdt_enable(WDTO_2S); // Enable Watchdog Timer, 8s

    raStp.init(DO_RA_STP_DIR, PWM_RA_STP_PUL, maxFreqRa, false, raCal);
    decStp.init(DO_DEC_STP_DIR, PWM_DEC_STP_PUL, maxFreqDec, true, decCal);
    pinMode(DI_RA_ENC_A, INPUT_PULLUP); // Set up RA Encoder wiring.
    pinMode(DI_RA_ENC_B, INPUT_PULLUP);
    pinMode(DI_DEC_ENC_A, INPUT_PULLUP); 
    pinMode(DI_DEC_ENC_B, INPUT_PULLUP);
     
    // setup interrupts for the A wires of the two encoders, DEC and RA.
    RA_encPtr = &EncRA; // assign the Encoder object to the pointer
    DEC_encPtr = &EncDEC; // see the Free functions for ISR above.
    attachInterrupt(digitalPinToInterrupt(DI_RA_ENC_A), RA_countPulsesISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_DEC_ENC_A), DEC_countPulsesISR, CHANGE);
} 
 

/// @brief Main loop
void loop() {

    pos::SiderealTime::update(); // Update the sidereal time
    pos::currentLocation.updateSiderealTime(pos::SiderealTime::getValue()); // Pass the sidereal time to the current location
    pos::currentLocation.updatePosition(io::getMotorPositions(raStp, decStp)); // Update the current location from the motor positions
    //pos::EncoderPosition.updatePosition(io::getEncoderPositions()); // Update the encoder based position

    hhc.updateButtons();
    disp.updateStates(hhc,initialSync,io::isHome());
    disp.show(hhc,pos::currentLocation.getPosition(SKY),ctrl::getHoming());
    ctrl::ctrlMode = (disp.getAutoManState()) ? AUTO : MANUAL;
    ctrl::trkMode = (disp.getTrackState()) ? TRACK : NO_TRACK;


    /*
       RAMPING CODE
    */
    static int rampingCountMax = 30;

    // DEC Plus ramping variables
    static bool rampingActiveDECPlus = false;
    static bool rampingTriggerDECPlus = false;
    static long int rampingCounterDECPlus = 0;
    
    static double slewRateHzDEC = 0;

    // DEC Minus ramping variables
    static bool rampingActiveDECMinus = false;
    static bool rampingTriggerDECMinus = false;
    static long int rampingCounterDECMinus = 0;
    
    // RA Plus ramping variables
    static bool rampingActiveRAPlus = false;
    static bool rampingTriggerRAPlus = false;
    static long int rampingCounterRAPlus = 0;
    
    static double slewRateHzRA = 0;

    // DEC Minus ramping variables
    static bool rampingActiveRAMinus = false;
    static bool rampingTriggerRAMinus = false;
    static long int rampingCounterRAMinus = 0;

    static bool rampingActive;

    if (rampingActiveDECPlus || rampingActiveDECMinus || rampingActiveRAPlus || rampingActiveRAMinus) {
        rampingActive = true;
    } else {
        rampingActive = false;
    }

    // RAMPING CODE ends

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
        //double slewRateHz = 50000;
        double maxSlewRateHz = 50000;

        if (hhc.getPotValue() < 256 ) {
            maxSlewRateHz = 10000;
        } else if (hhc.getPotValue() < 512 ) {
            maxSlewRateHz = 20000;
        } else if (hhc.getPotValue() < 768 ) {
            maxSlewRateHz = 35000;
        } else {
            maxSlewRateHz = maxSlewRateHz;
        }
        
        DisplayMode dispMode = disp.getDisplayMode();
        if(ctrl::getHoming())disp.setTrackState(false);
        if(hhc.getBtnGoToRise()) {
            ctrl::moveHome(raStp,decStp);
            tone(PWM_BZR,NOTE_C6,BEEP_TIME);
        }

        // Set ramping rates, if required.

        // DEC Plus ramping
        if(hhc.getBtnDecPlusRise()) {
            rampingTriggerDECPlus = true;
            //Serial.println("DEC+ Step 1: rampingCounterDECPlus: " + String(rampingCounterDECPlus));
       }
        if(rampingTriggerDECPlus && !rampingActiveDECPlus) {
            rampingActiveDECPlus = true;
            rampingTriggerDECPlus = false;
            rampingCounterDECPlus = rampingCountMax;
            //Serial.println("DEC+ Step 2: rampingCounterDECPlus: " + String(rampingCounterDECPlus));
        }
        if(rampingActiveDECPlus && (rampingCounterDECPlus >= 1)) {
            slewRateHzDEC = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterDECPlus);
            rampingCounterDECPlus = rampingCounterDECPlus - 1;
            //Serial.println("DEC+ Step 3: rampingCounterDECPlus: " + String(rampingCounterDECPlus));
        } else if(rampingActiveDECPlus && (rampingCounterDECPlus < 1)) {
            rampingActiveDECPlus = false;
            slewRateHzDEC = maxSlewRateHz;
            //Serial.println("DEC+ Step 4: rampingCounterDECPlus: " + String(rampingCounterDECPlus));
        }

        // DEC Minus ramping
        if(hhc.getBtnDecMinusRise()) {
            rampingTriggerDECMinus = true;
            //Serial.println("DEC- Step 1: rampingCounterDECMinus: " + String(rampingCounterDECMinus));
        }
        if(rampingTriggerDECMinus && !rampingActiveDECMinus) {
            rampingActiveDECMinus = true;
            rampingTriggerDECMinus = false;
            rampingCounterDECMinus = rampingCountMax;
            //Serial.println("DEC- Step 2: rampingCounterDECMinus: " + String(rampingCounterDECMinus));
        }
        if(rampingActiveDECMinus && (rampingCounterDECMinus >= 1)) {
            slewRateHzDEC = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterDECMinus);
            rampingCounterDECMinus = rampingCounterDECMinus - 1;
            //Serial.println("DEC- Step 4: rampingCounterDECMinus: " + String(rampingCounterDECMinus));
        } else if(rampingActiveDECMinus && (rampingCounterDECMinus < 1)) {
            rampingActiveDECMinus = false;
            slewRateHzDEC = maxSlewRateHz;
            //Serial.println("DEC- Step 5: rampingCounterDECMinus: " + String(rampingCounterDECMinus));
        }

       // RA Plus ramping
        if(hhc.getBtnRaPlusRise()) {
            rampingTriggerRAPlus = true;
            //Serial.println("RA+ Step 1: rampingCounterRAPlus: " + String(rampingCounterRAPlus));
        }
        if(rampingTriggerRAPlus && !rampingActiveRAPlus) {
            rampingActiveRAPlus = true;
            rampingTriggerRAPlus = false;
            rampingCounterRAPlus = rampingCountMax;
            //Serial.println("RA+ Step 2: rampingCounterRAPlus: " + String(rampingCounterRAPlus));
        }
        if(rampingActiveRAPlus && (rampingCounterRAPlus >= 1)) {
            slewRateHzRA = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterRAPlus);
            rampingCounterRAPlus = rampingCounterRAPlus - 1;
            //Serial.println("RA+ Step 3: rampingCounterRAPlus: " + String(rampingCounterRAPlus));
        } else if(rampingActiveRAPlus) {
            rampingActiveRAPlus = false;
            slewRateHzRA = maxSlewRateHz;
            //Serial.println("RA+ Step 4: rampingCounterRAPlus: " + String(rampingCounterRAPlus));
        }
        //Serial.println("Post RA+: RampingActiveDECPlus: " + String(rampingActiveDECPlus));
        // RA Minus ramping
        if(hhc.getBtnRaMinusRise()) {
            rampingTriggerRAMinus = true;
            //Serial.println("RA- Step 1: rampingCounterRAMinus: " + String(rampingCounterRAMinus));
        }
        if(rampingTriggerRAMinus && !rampingActiveRAMinus) {
            rampingActiveRAMinus = true;
            rampingTriggerRAMinus = false;
            rampingCounterRAMinus = rampingCountMax;
            //Serial.println("RA- Step 2: rampingCounterRAMinus: " + String(rampingCounterRAMinus));
        }
        if(rampingActiveRAMinus && (rampingCounterRAMinus >= 1)) {
            slewRateHzRA = (maxSlewRateHz / rampingCountMax) * (rampingCountMax - rampingCounterRAMinus);
            rampingCounterRAMinus = rampingCounterRAMinus - 1;
            //Serial.println("RA- Step 3: rampingCounterRAMinus: " + String(rampingCounterRAMinus));
        } else if(rampingActiveRAMinus) {
            rampingActiveRAMinus = false;
            slewRateHzRA = maxSlewRateHz;
            //Serial.println("RA- Step 4: rampingCounterRAMinus: " + String(rampingCounterRAMinus));
        }
        //Serial.println("Post RA+: RampingActiveDECPlus: " + String(rampingActiveDECPlus)+", Ramping ends");
        // ramping code ends

        // Manual move button press code
        if(!hhc.getBtnRaPlus() && (dispMode == COORDS || dispMode == SYNC) ){
            if (ctrl::getHoming()) raStp.run(REVERSE, raStp.getMaxFrequency());
            else raStp.run(REVERSE, slewRateHzRA/1.5);
        }
        else if(!hhc.getBtnRaMinus() && (dispMode == COORDS || dispMode == SYNC)){
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
            //  if (ctrl::getHoming()) decStp.run(FORWARD, decStp.getMaxFrequency());
            decStp.run(FORWARD, slewRateHzDEC);
        }
        else if(!hhc.getBtnDecMinus()&& (dispMode == COORDS || dispMode == SYNC)){
            if (ctrl::getHoming()) decStp.run(REVERSE, decStp.getMaxFrequency());
            else decStp.run(REVERSE, slewRateHzDEC);
        }
        else if (!ctrl::getHoming()){
            decStp.stop();
        }
        // END Manual move button press code
    }

    /*
    static unsigned long prevMillis = millis();
    if(millis()-prevMillis>=1500){
        //Serial.println("rampingCounterDECPlus: " + String(rampingCounterDECPlus)+", rampingActiveDECPlus: " + String(rampingActiveDECPlus)) ;
        //Serial.println("rampingActiveDECPlus: " + String(rampingActiveDECPlus));
        //Serial.println("rampingCounterRA: " + String(rampingCounterRA));
        //Serial.println("rampingCounterDEC: " + String(rampingCounterDEC));
        //Serial.println("rampingActiveRA: " + String(rampingActiveRA));
        //Serial.println("rampingActiveDEC: " + String(rampingActiveDEC));
        Serial.println("slewRateHzRA: " + String(slewRateHzRA));
        Serial.println("slewRateHzDEC: " + String(slewRateHzDEC));
        Serial.println("RampingActive_DEC+,DEC-,RA+,RA-: " + String(rampingActiveDECPlus) + String(rampingActiveDECMinus) + String(rampingActiveRAPlus) + String(rampingActiveRAMinus));
        
        Serial.println("----------------");
        
        prevMillis = millis();
    }
    */


    /*
    //  Encoders
    EncRA.update();
    EncDEC.update();
    */

    /*
    // Messaging to the Serial Monitor
    */
    static unsigned long currentMillis;
    static unsigned long prevMillis = 0;
    currentMillis = millis();
    
    if(currentMillis - prevMillis >= 1000){

        String message = "RA Pulse:" +
                String(EncRA.getStepCount()) +
                " RA Edge: " +
                String(EncRA.getEdgeCount()) +
                " RA Rev: " +
                String(EncRA.getRevCount()) +
                " RA Dir: " +
                String(EncRA.getDirection() )+
                " RA Deg: " +
                String(EncRA.getDegrees() ) +
                " | " +
                " DEC Pulse:" +
                String(EncDEC.getStepCount()) +
                " DEC Edge: " +
                String(EncDEC.getEdgeCount()) +
                " DEC Rev: " +
                String(EncDEC.getRevCount()) +
                " DEC Dir: " +
                String(EncDEC.getDirection() )+
                " DEC Deg: " +
                String(EncDEC.getDegrees() ) +
                "\n";

        Serial.print(message);
        prevMillis = currentMillis;
    }

    ctrl::homeStop(raStp,decStp);
    // // // // // // // // // SAFTEY LIMITS // // // // // // // // // // // //
    // ctrl::horizonStop(pos::currentLocation, raStp, decStp, ctrl::ctrlMode);
    io::limitStop(decStp); //Should be last function called in loop to ensure limit switches will stop motors
    // // // // // // // // // // // // // // // // // // // // // // // // // /
    wdt_reset();
}

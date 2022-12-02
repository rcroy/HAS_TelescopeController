
#include "src/config.h"
#include "src/comms.h"
#include "src/ctrl.h"
#include "src/pos.h"
#include "src/io.h"
#include "src/stepper.h"
#include "src/utils.h"

//TODO: move variables to appropriate modules. Ideally only setup and loop should be in this file
String buffer ="";
command currentCmd;
String coordString;
// bool g_isSlewing = false;
bool initialSync = false;
double slewRateHz = 250.0;
int raDir;
int decDir;
bool isRaPul;
bool isDecPul;
bool isTrack;
int potVal;
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
    autoManualMode state = MANUAL;
        
}

io::Stepper raStp;
io::Stepper decStp;

////////////////////////////////////////////////////////////////////////////////
/// Main Program
////////////////////////////////////////////////////////////////////////////////


/// @brief Main program entry point.
void setup() {
    io::setupLimits();
    io::setupPinModes();
    Serial.begin(9600);
    Serial1.begin(9600);


    raStp.init(DO_RA_STP_DIR, PWM_RA_STP_PUL, maxFreqRa, false, raCal);
    decStp.init(DO_DEC_STP_DIR, PWM_DEC_STP_PUL, maxFreqDec, true, decCal);

}

/// @brief Main loop
void loop() {

    pos::SiderealTime::update(); // Update the sidereal time
    pos::currentLocation.updateSiderealTime(pos::SiderealTime::getValue()); // Pass the sidereal time to the current location
    pos::currentLocation.updatePosition(io::getMotorPositions(raStp, decStp)); // Update the current location from the motor positions
    ctrl::state = (digitalRead(DI_MODE) == LOW) ? AUTO : MANUAL;

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
            }
            pos::currentLocation.syncTo(pos::targetPosition);

            comms::sendReply("SYNCED TO " + 
                            comms::double2RaStr(pos::currentLocation.getCoord(SKY, RA)) + " " +
                            comms::double2DecStr(pos::currentLocation.getCoord(SKY, DECL)));
            buffer = "";
            break;

            case SLEW_TO_TARGET:
            switch (ctrl::state){
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
            switch (ctrl::state){
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
            //        Serial1.println("WARN: invalid command received");
            buffer = "";
            break;
        }
    
    }

    if(ctrl::state == MANUAL){
        potVal = analogRead(AI_POT);
        slewRateHz = 0.095*potVal*potVal; //quadradic curve allows for fine control at low end, while still allowing fast slew at high end
        // Serial.println(slewRateHz);
        raDir = !digitalRead(DI_RA_DIR);
        isRaPul = !digitalRead(DI_RA_PUL);
        decDir = !digitalRead(DI_DEC_DIR);
        isDecPul = !digitalRead(DI_DEC_PUL);
        isTrack = !digitalRead(DI_TRACK);

        // digitalWrite(DO_RA_EN,isRaPul);
        // digitalWrite(DO_DEC_EN,isDecPul);
        if(isRaPul){
            raStp.run(raDir, slewRateHz/2);
        }
        else if(isTrack){
            raStp.run(FORWARD, ctrl::trackRateHz);
        }
        else{
            raStp.stop();
        }
        if(isDecPul){
            decStp.run(decDir, slewRateHz);
        }
        else{
            decStp.stop();
        }

        // static unsigned long lastTime = 0;
        // if(millis() - lastTime > 1000){
        //     lastTime = millis();
        //     Serial1.println("RA: " + String(raStp.getPulseCount()) + " DEC: " + String(decStp.getPulseCount()) + " MODE: " + String(digitalRead(DI_MODE)));
        // }
    }
    // // // // // // // // // SAFTEY LIMITS // // // // // // // // // // // //
    ctrl::horizonStop(pos::currentLocation, raStp, decStp, ctrl::state);
    io::limitStop(decStp); //Should be last function called in loop to ensure limit switches will stop motors
    // // // // // // // // // // // // // // // // // // // // // // // // // /
}
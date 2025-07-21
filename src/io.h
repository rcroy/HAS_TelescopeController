#ifndef HAS_IO
#define HAS_IO

#include "pos.h"
#include "stepper.h"
#include "encoder.h"


/// @brief The io namespace contains functionality for initialising, reading, 
/// and controlling sensors and actuators.
namespace io{
    pos::Position getMotorPositions(Stepper& ra, Stepper& dec);
    pos::Position getEncoderPositions(Encoder& RA_enc, Encoder& DEC_enc);
    void setupLimits();
    void stopMotors();
    void limitStop(Stepper& dec);
    bool decLimCheck(int dir);
    bool isHome();

}
#endif

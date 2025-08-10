#include "encoder.h"
#include "enums.h"

namespace io{

    Encoder::Encoder(int pinA, int pinB, int maxEdges){
        this->pinA = pinA;
        this->pinB = pinB;
        edgesPerRev = maxEdges;
        degsPerEdge = 360/double(edgesPerRev);
        A = digitalRead(pinA);
        B = digitalRead(pinB);
        _A = A;
        _B = B;
        deg = 0.0;
        pulseCount = 0;
        edgeCount = 0;
        revCount = 0;
    }

    static void Encoder::countPulses(boolean ThisDec) {
        if(digitalRead(pinA) != digitalRead(pinB)) 
        {
            edgeCount--;
            if(ThisDec) {
                deg += degsPerEdge;
            } else {
                deg += -degsPerEdge;
            }

        } else {
            edgeCount++;
            if(ThisDec) {
                deg += -degsPerEdge;
            } else {
                deg += degsPerEdge;
            }
        }
    }

    void Encoder::reset(){
        edgeCount = 0;
        pulseCount = 0;
        revCount = 0;
        deg = 0.0;
    }

    
};

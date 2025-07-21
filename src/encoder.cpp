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

    static void Encoder::countPulses() {
        if(digitalRead(pinA) != digitalRead(pinB)) 
        {
            edgeCount--;
            deg += degsPerEdge;
        } else {
            edgeCount++;
            deg += -degsPerEdge;
        }
    }

    void Encoder::reset(){
        edgeCount = 0;
        pulseCount = 0;
        revCount = 0;
        deg = 0.0;
    }

    
};

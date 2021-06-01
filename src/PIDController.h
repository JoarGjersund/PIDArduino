#ifndef PIDControllerLib
#define PIDControllerLib

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define GRAPH     "graph"
#define NOGRAPH   "nograph"
#define VERBOSE   "verbose"
#define NOVERBOSE "noverbose"

class PIDController {
  public:
    // Constructor
    PIDController();

    // Methods - double
    int16_t compute(int16_t input, String graph = NOGRAPH, String verbose = NOVERBOSE);

    // Methods - void
    void begin();
    void tune(double _Kp, double _Ki, double _Kd);
    void limit(double min, double max);
    void setpoint(double newSetpoint);
    void minimize(double newMinimize);

    // Methods - double, getters
    int16_t getOutput();
  private:
    // Methods
    void printGraph(double sensorInput, String verbose);
    
    // Variables - long
    unsigned long lastTime;

    // Variables - double
    int16_t output;
    int16_t lastErr;
    float timeChanged;

    // Variables - double, error variables
    int16_t error;
    int16_t errSum;
    int16_t dErr;

    // Variables - bool
    bool doLimit;
    bool init;

    // Variables - double - tuining
    float Kp;
    float Ki;
    float Kd;
    float divisor;
    int16_t minOut;
    int16_t maxOut;
    int16_t setPoint;
};
#endif

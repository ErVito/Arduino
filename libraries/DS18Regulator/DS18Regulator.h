#ifndef DS18_REGULATOR_H
#define DS18_REGULATOR_H

#include <DallasTemperature.h>
#include <PID_v1.h>

/* CONSTANTS DECLARATIONS */

#define NUM_OF_REGULATORS 4
#define OK                0

/* ERRORS */
#define ERR_INSUFFICIENT_MEMORY     1  /* 0b00000001 */

/* PID COMPUTATION ERRORS    (0b0000001X for every X) */
#define ERR_SENSOR_DISCONNECTED     2  /* 0b00000010 */
#define ERR_PID_NOT_COMPUTED        3  /* 0b00000011 */

/* INVALID PARAMETERS ERRORS (0b000001XX for every X) */
#define ERR_PARAMETERS_NOT_VALID    4  /* 0b00000100 */
#define ERR_INDEX_NOT_VALID         5  /* 0b00000101 */
#define ERR_REGULATOR_DOESNT_EXIST  6  /* 0b00000110 */
#define ERR_RESOLUTION_NOT_VALID    7  /* 0b00000111 */

typedef struct {

    double  maxTemperature;     /* Maximum admissible value of temperature. */
    double  minTemperature;     /* Minimum admissible value of temperature. */
    double  dutyCycleOfPWM;     /* Output of the PID. */
    double  temperature;        /* Input of the PID.  */
    double  reference;          /* Setpoint of the PID (desired temperature). */
    PID    *pid;

} RegulatorStruct;

class DS18Regulator {

    /* ATTRIBUTES */
    private:
            DallasTemperature *sensors;
            RegulatorStruct   *regulators[NUM_OF_REGULATORS];

    /* FUNCTIONS */
    public:
            DS18Regulator(DallasTemperature *sensors);
            uint8_t newRegulator(uint8_t  sensorIndex,
                                 double   Kp,
                                 double   Ki,
                                 double   Kd,
                                 uint32_t computationPeriod);   /* Expressed in ms. */
            uint8_t computePID(uint8_t sensorIndex);

            /* SETTERS */
            uint8_t setPWMResolution(uint8_t regulatorIndex, uint8_t bits);
            uint8_t setReference(uint8_t regulatorIndex, double newReference);
            uint8_t setTempRange(uint8_t regulatorIndex, 
                                 double  newMinTemperature,
                                 double  newMaxTemperature);

            /* GETTERS */
            uint8_t getDutyCycle(uint8_t regulatorIndex, double *dutyCycle);
            uint8_t getMinTemp(uint8_t regulatorIndex, double *minTemperature);
            uint8_t getMaxTemp(uint8_t regulatorIndex, double *maxTemperature);
            uint8_t getReference(uint8_t regulatorIndex, double *reference);
            uint8_t getTemperature(uint8_t regulatorIndex, double *temperature);
};

#endif

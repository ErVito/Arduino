/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the  Free Software  Foundation, either  version 3 of the License, or
(at your option) any later version.

This  program  is  distributed  in the hope that  it  will be useful,
but  WITHOUT ANY  WARRANTY; without  even  the  implied  warranty  of
MERCHANTABILITY  or FITNESS  FOR  A  PARTICULAR  PURPOSE. See the GNU
General Public License for more details.

You should  have received  a copy  of the  GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
 _______________________________________________________________________
/                                                                       \
| Author:       Marco Vitetta (also known as ErVito)                    |
| E-mail:       ervito.development [at] (NO SPAM) gmail [dot] com       |
| WebSite:      http://ervito.altervista.org                            |
|_______________________________________________________________________|
|                                                                       |
| Versions:                                                             |
|   1.0.2       Few little improvements                                 |
|   1.0.1       Added setPIDGains() function and license specification  |
|   1.0.0       First public release                                    |
\_______________________________________________________________________/
*/

#include "DS18Regulator.h"

DS18Regulator::DS18Regulator(DallasTemperature *sensors) {
    this->sensors = sensors;
    
    for(uint8_t sensorIndex = 0; NUM_OF_REGULATORS > sensorIndex; sensorIndex++)
        regulators[sensorIndex] = NULL;
}

uint8_t DS18Regulator::newRegulator(uint8_t  sensorIndex,
                                    double   Kp,
                                    double   Ki,
                                    double   Kd,
                                    uint32_t computationPeriod) {   /* Expressed in ms */

    if (NUM_OF_REGULATORS <= sensorIndex  || regulators[sensorIndex])
        return ERR_INDEX_NOT_VALID;

    RegulatorStruct *regulator = (RegulatorStruct*)calloc(1, sizeof(RegulatorStruct));

    if (NULL == regulator)
        return ERR_INSUFFICIENT_MEMORY;

    regulator->maxTemperature = 3.4028235E+38;
    regulator->minTemperature = -3.4028235E+38;
    regulator->dutyCycleOfPWM = 0;
    regulator->temperature    = 0;
    regulator->reference      = 0;
    
    PID *regulatorPID         = new PID(&(regulator->temperature),
                                        &(regulator->dutyCycleOfPWM),
                                        &(regulator->reference),
                                        Kp,
                                        Ki,
                                        Kd,
                                        DIRECT);                    /* Control direction */

    if (NULL == regulatorPID) {
        free(regulator);
        return ERR_INSUFFICIENT_MEMORY;
    }

    regulatorPID->SetMode(AUTOMATIC);
    regulatorPID->SetOutputLimits(0, 255);
    regulatorPID->SetSampleTime(computationPeriod);
    
    regulator->pid          = regulatorPID;
    regulators[sensorIndex] = regulator;

    return OK;
}

uint8_t DS18Regulator::computePID(uint8_t sensorIndex) {
    if (NUM_OF_REGULATORS <= sensorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[sensorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    sensors->requestTemperaturesByIndex(sensorIndex);
    
    double temperature = sensors->getTempCByIndex(sensorIndex);

    if (DEVICE_DISCONNECTED_C != temperature)
        regulators[sensorIndex]->temperature = temperature;

    bool PIDComputed = regulators[sensorIndex]->pid->Compute();

    if (DEVICE_DISCONNECTED_C != temperature) {
        if (PIDComputed)
            return OK;
        else
            return ERR_PID_NOT_COMPUTED;
    }
    return ERR_SENSOR_DISCONNECTED;
}

/* SETTERS */

uint8_t DS18Regulator::setPIDGains(uint8_t regulatorIndex,
                                   double  Kp,
                                   double  Ki,
                                   double  Kd) {
    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    regulators[regulatorIndex]->pid->SetTunings(Kp, Ki, Kd);

    return OK;
}

uint8_t DS18Regulator::setPWMResolution(uint8_t regulatorIndex, uint8_t bits) {
    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    if (0 == bits || 12 < bits)
        return ERR_RESOLUTION_NOT_VALID;

    switch(bits) {
        case 1:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 1);    break;
        case 2:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 3);    break;
        case 3:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 7);    break;
        case 4:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 15);   break;
        case 5:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 31);   break;
        case 6:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 63);   break;
        case 7:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 127);  break;
        case 8:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 255);  break;
        case 9:  regulators[regulatorIndex]->pid->SetOutputLimits(0, 511);  break;
        case 10: regulators[regulatorIndex]->pid->SetOutputLimits(0, 1023); break;
        case 11: regulators[regulatorIndex]->pid->SetOutputLimits(0, 2047); break;
        case 12: regulators[regulatorIndex]->pid->SetOutputLimits(0, 4095); break;
    }
    return OK;
}

uint8_t DS18Regulator::setReference(uint8_t regulatorIndex, double newReference) {
    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    RegulatorStruct *regulator = regulators[regulatorIndex];

    if (newReference < regulator->minTemperature ||
        newReference > regulator->maxTemperature)
        return ERR_PARAMETERS_NOT_VALID;

    regulator->reference = newReference;

    return OK;
}

uint8_t DS18Regulator::setTempRange(uint8_t regulatorIndex, 
                                    double  newMinTemperature,
                                    double  newMaxTemperature) {

    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    if (newMinTemperature > newMaxTemperature)
        return ERR_PARAMETERS_NOT_VALID;

    RegulatorStruct *regulator = regulators[regulatorIndex];

    regulator->maxTemperature = newMaxTemperature;
    regulator->minTemperature = newMinTemperature;

    if (regulator->reference < newMinTemperature)
        regulator->reference = newMinTemperature;
    else
        if (regulator->reference > newMaxTemperature)
            regulator->reference = newMaxTemperature;

    return OK;
}

/* END SETTERS */

/* GETTERS */

uint8_t DS18Regulator::getDutyCycle(uint8_t regulatorIndex, double *dutyCycle) {
    if (regulatorIndex >= NUM_OF_REGULATORS)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    *dutyCycle = regulators[regulatorIndex]->dutyCycleOfPWM;

    return OK;
}

uint8_t DS18Regulator::getMinTemp(uint8_t regulatorIndex, double *minTemperature) {
    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    *minTemperature = regulators[regulatorIndex]->minTemperature;

    return OK;
}

uint8_t DS18Regulator::getMaxTemp(uint8_t regulatorIndex, double *maxTemperature) {
    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    *maxTemperature = regulators[regulatorIndex]->maxTemperature;

    return OK;
}

uint8_t DS18Regulator::getReference(uint8_t regulatorIndex, double *reference) {
    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    *reference = regulators[regulatorIndex]->reference;

    return OK;
}

uint8_t DS18Regulator::getTemperature(uint8_t regulatorIndex, double *temperature) {
    if (NUM_OF_REGULATORS <= regulatorIndex)
        return ERR_INDEX_NOT_VALID;

    if (NULL == regulators[regulatorIndex])
        return ERR_REGULATOR_DOESNT_EXIST;

    *temperature = regulators[regulatorIndex]->temperature;

    return OK;
}

/* END GETTERS */

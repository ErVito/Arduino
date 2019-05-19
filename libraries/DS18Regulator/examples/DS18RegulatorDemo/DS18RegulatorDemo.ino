#include <DallasTemperature.h>
#include "DS18Regulator.h"
#include <OneWire.h>

#ifndef ESP8266

/*  /¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯ Arduino/Genunino UNO ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯\ or /¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯ Arduino MEGA ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯\ */
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#include <TimerOne.h>
#elif defined(_SAM3XA_)                         /* Arduino DUE */
#include <DueTimer.h>
#endif

const uint8_t  HEATER              = 7;         /* Pin of the heater         */
const uint8_t  ONE_WIRE_BUS        = 8;         /* Pin of the DS18B20 sensor */
#else

/* ESP8266 */
extern "C" {
#include "user_interface.h"       // For the os_timer functions of the Non-OS SDK
}

const uint8_t  HEATER              = D7;        /* Pin of the heater         */
const uint8_t  ONE_WIRE_BUS        = D8;        /* Pin of the DS18B20 sensor */
#endif

/* COMMON CONSTANTS */
const double   KP                  = 1;
const double   KI                  = 0.1;
const double   KD                  = 0;
const uint32_t COMPUTATION_PERIOD  = 5000000;   /* Expressed in microseconds */

/* Period is reduced by factor  256 to pilot (switch
 * ON/OFF)  the   heater  with   maximum  resolution
 * admissible by the domain of the duty cycle (which 
 * assumes value between 0 and 255).                 */
const uint32_t TICK_PERIOD         = COMPUTATION_PERIOD / 256;

/* GLOBAL VARIABLES */
double         dutyCycle;
double         temperature;
uint8_t        PIDComputation;
uint32_t       previousComputation = 0;
uint32_t       ticks               = 0;
volatile bool  tickOccured         = false;

#ifdef ESP8266  /* ESP8266-specific global variables */
os_timer_t     timer;
#endif

OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DS18Regulator     temperatureController(&sensors);

void setup() {
  Serial.begin(115200);
  Serial.print("Initializing sensors...");

  sensors.begin();

  if(sensors.getDeviceCount() == 0) {
    Serial.println("failed.");
    while(1);
  }
  Serial.println("done.");

  temperatureController.newRegulator(0, KP, KI, KD, COMPUTATION_PERIOD / 1000);       /* Period expressed in milliseconds */
  temperatureController.setTempRange(0, 23.0, 28.0);
  temperatureController.setReference(0, 25.0);

#ifndef ESP8266

/*  /¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯ Arduino/Genunino UNO ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯\ or /¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯ Arduino MEGA ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯\ */
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Timer1.initialize(TICK_PERIOD);
#endif

  Timer1.attachInterrupt(tick);

#if defined(_SAM3XA_)                                                                 /* Arduino DUE */
  Timer1.start(TICK_PERIOD);
#endif

#else

  /* The reason why it has been chosen to use the os_timer
   * functions is that timer0 appears to be less precise. */
  os_timer_setfn(&timer, &tick, NULL);

  /* Control action will be repeated (due to the "true"
   * parameter).                                          */
  os_timer_arm(&timer, TICK_PERIOD / 1000, true);                                     /* Period expressed in milliseconds */

#endif
}

void loop() {
  if (tickOccured) {
    tickOccured = false;
    ticks++;

    if (ticks >= dutyCycle)
      digitalWrite(HEATER, LOW);

    /* The control law of the PID will be 
     * computed every 256 ticks (when the
     * COMPUTATION_PERIOD has elapsed).   */
    if (255 == ticks) {
      ticks = 0;
      PIDComputation = temperatureController.computePID(0);

      Serial.println("-------------------");
      Serial.print("Any problem? ");

      if (OK == PIDComputation)
        Serial.println("NO");
      else {
        Serial.println("YES");
        Serial.print("Reason: ");
        if (ERR_PID_NOT_COMPUTED == PIDComputation)
          Serial.println("computation period hasn't elapsed");
        else
          Serial.println("sensor disconnected");
      }

      temperatureController.getDutyCycle(0, &dutyCycle);
      temperatureController.getTemperature(0, &temperature);
      Serial.print("Error:  ");
      Serial.print(25 - temperature, 6);
      Serial.println(" °C");
      Serial.print("Duty:   ");
      Serial.println(dutyCycle, 0);
      Serial.print("Period: ");
      Serial.println(millis() - previousComputation);
      previousComputation = millis();

      if (dutyCycle > 0)
        digitalWrite(HEATER, HIGH);
    }
  }
}

void tick(void *) {
  tickOccured = true;
}

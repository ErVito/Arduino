# DS18Regulator: an Arduino library that implements a temperature regulator using a [PID](https://en.wikipedia.org/wiki/PID_controller).

This library is a wrapper around the [PID library](https://github.com/br3ttb/Arduino-PID-Library/) to easily manage a temperature control loop using a DS18 temperature sensor (one of those supported by the [DallasTemperature library](https://github.com/milesburton/Arduino-Temperature-Control-Library)) and an heater/cooler piloted by a [PWM](https://en.wikipedia.org/wiki/Pulse-width_modulation) signal.

## Contents

- [Usage](#usage)
- [Examples](#examples)
- [Versions](#versions)
- [Credits](#credits)
- [License](#license)

## Usage

The library allows to instantiate one object for each DallasTemperature one (basically one for each [OneWire](https://github.com/PaulStoffregen/OneWire) bus) through its constructor.
```
const uint8_t       ONE_WIRE_BUS = 8;         /* Pin of the DS18B20 sensor */

DallasTemperature   sensors(ONE_WIRE_BUS);
DS18Regulator       temperatureController(&sensors);
```

Each DS18Regulator object allows to manage a limited number of regulators which can be set by the constant _NUM\_OF\_REGULATORS_ into the header file ([DS18Regulator.h](https://github.com/ErVito/Arduino/blob/master/Libraries/DS18Regulator/DS18Regulator.h)).
Once the object has been instantiated, a new regulator can be declared using the _newRegulator()_ method which requires the index (on the [OneWire](https://github.com/PaulStoffregen/OneWire) bus) of the sensor used, the values of the [PID](https://en.wikipedia.org/wiki/PID_controller) gains and the computation period of the [PID](https://en.wikipedia.org/wiki/PID_controller) (equals to the period of the [PWM](https://en.wikipedia.org/wiki/Pulse-width_modulation) signal used to control the heater/cooler) expressed in milliseconds.
```
temperatureController.newRegulator(0, 1, 0.1, 0, 300000);
```

Now can be called the _computePID()_ method which requires only the index (on the [OneWire](https://github.com/PaulStoffregen/OneWire) bus) of the sensor used and it determines the duty cycle of the [PWM](https://en.wikipedia.org/wiki/Pulse-width_modulation) signal to be performed (which can be retrived with the getter method _getDutyCycle()_).
```
temperatureController.computePID(0);

double dutyCycle;

temperatureController.getDutyCycle(0, &dutyCycle);
```

## Examples
Within the [Examples](https://github.com/ErVito/Arduino/tree/master/libraries/DS18Regulator/examples) folder you can find a sketch that implements the control structure described above.

## Versions
1.0.1 - Added setPIDGains() function and license specification
1.0.0 - First public release

## Credits
This library has been written by ErVito using the library cited:
- [PID](https://github.com/br3ttb/Arduino-PID-Library/) by Brett Beauregard;
- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library) by Miles Burton and others;
- [OneWire](https://github.com/PaulStoffregen/OneWire) by Paul Stoffregen and others.

## License
This project is licensed under the GNU GPL v3 or any later leversion.
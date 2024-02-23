#include <sensor.h>
#include <logging.h>

class TemperatureSensor : public Sensor
{
public:
    TemperatureSensor(int pin, String name); // Constructor
    double getReading();                     // Method
};

// set up getReading method
double TemperatureSensor::getReading()
{
    Log.infoln("Getting temperature reading...");
    // for now, we don't have a temperature sensor, so we'll just return a number based on the sin of the current time
    double reading = abs(10 * sin(millis() / 10000.0)) + 10;
    return reading;
}

TemperatureSensor::TemperatureSensor(int pin, String name) : Sensor(pin, name)
{
    // nothing to do here
}
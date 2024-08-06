#include <Arduino.h>
#include <logging.h>

struct killswitchState
{
    int id;
    bool state;
    bool canToggle;
    unsigned long lastDebounceTime;
};

killswitchState killswitchesStates[2];

bool killswitch(int pin, int id, bool shouldToggle, int outputPin)
{
    bool kill = !digitalRead(pin);

    if (!shouldToggle)
    {
        digitalWrite(outputPin, kill);
        return kill;
    }

    killswitchState &ks = killswitchesStates[id];

    if ((millis() - ks.lastDebounceTime) > 1000)
    {
        // Toggle the state only if the button is pressed (reading == true) and
        // the last state was not pressed (ks.lastState == false)
        if (kill == true && ks.canToggle)
        {
            ks.canToggle = false;
            ks.state = !ks.state;
            ks.lastDebounceTime = millis();
        }
        else if (kill == false)
        {
            ks.canToggle = true;
        }
    }
    digitalWrite(outputPin, ks.state);
    return ks.state;
}

void setupKillswitch(int pin, int id, bool shouldToggle, int outputPin)
{
    pinMode(pin, INPUT_PULLUP);
    Log.infoln("A %s Killswitch configured on pin %d", shouldToggle ? "Toggling" : "Momentary", pin);
    // set the killswitch to the id in the vector
    if (shouldToggle)
    {
        killswitchesStates[id] = {id, false, true, 0};
    }

    if (outputPin != -1)
    {
        pinMode(outputPin, OUTPUT);
    }

    return;
}
#include <Arduino.h>
#include <ArduinoJson.h>
#include <logging.h>
#include <StreamUtils.h>

// create a buffer for the incoming data
// ReadBufferingStream buffered_stream(Serial, 64);

bool WasError = false;

JsonDocument input_comms()
{
    // get the time this function takes
    // unsigned long start = millis();
    static JsonDocument json_doc;
    String SerialData;

    if (Serial.available() > 0)
    {
        SerialData = Serial.readStringUntil('\n');
    }

    const auto deser_err = deserializeJson(json_doc, SerialData);

    if (SerialData.length() == 0)
    {
        if (WasError)
        {
            Serial.println(F("Failed to deserialize, reason: \"Empty input\""));
            WasError = true;
        }
        json_doc.clear();
        return (json_doc);
    }

    if (deser_err && !deser_err.EmptyInput)
    {
        if (WasError)
        {
            Serial.print(F("Failed to deserialize, reason: \""));
            Serial.print(deser_err.c_str());
            Serial.println('"');
        }
        json_doc.clear();
        WasError = true;
        return (json_doc);
    }

    // else if (deser_err.EmptyInput)
    // {
    //     // do nothing
    //     // Serial.println(F("Empty input"));
    //     json_doc.clear();
    //     return (json_doc);
    // }

    else
    {
        Serial.print(F("Recevied valid json document with "));
        Serial.print(json_doc.size());
        Serial.println(F(" elements."));
        Serial.println();
        WasError = false;
    }

    // unsigned long delta = millis() - start;
    // Serial.print("Time taken to process incoming data:");
    // Serial.println(delta);

    return json_doc;
}
// SPDX - FileCopyrightText : 2025 Stefan Krüger s-light.eu
// SPDX - License - Identifier : CC0
// written by Stefan Krüger
// license: https://creativecommons.org/publicdomain/zero/1.0/

/******************************************************************************

    MPR121_Simple.ino
        sketch for basic test with the MPR121 sensor breakout form adafruit.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino compatible (with serial port)
            LED on pin 13
            MPR121
                TWI (SDA & SCL)


    libraries used:
        ~ slight_MPR121
            written by stefan krueger (s-light),
                git@s-light.eu, http://s-light.eu, https://github.com/s-light/
            license: MIT

    written by stefan krueger (s-light),
        git@s-light.eu, http://s-light.eu, https://github.com/s-light/

******************************************************************************/

#include <Wire.h>
#include <slight_MPR121.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Info
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void sketchinfo_print(Print &out) {
    out.println();
    //             "|~~~~~~~~~|~~~~~~~~~|~~~..~~~|~~~~~~~~~|~~~~~~~~~|"
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|                       ^ ^                      |"));
    out.println(F("|                      (0,0)                     |"));
    out.println(F("|                      ( _ )                     |"));
    out.println(F("|                       \" \"                      |"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("| MPR121_Simple.ino"));
    out.println(F("|   sketch for simple test with the"));
    out.println(F("|   MPR121 sensor breakout from adafruit."));
    out.println(F("|"));
    out.println(F("|"));
    out.println(F("| dream on & have fun :-)"));
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|"));
    //out.println(F("| Version: Nov 11 2013  20:35:04"));
    out.print(F("| version: "));
    out.print(F(__DATE__));
    out.print(F("  "));
    out.print(F(__TIME__));
    out.println();
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println();

    //out.println(__DATE__); Nov 11 2013
    //out.println(__TIME__); 20:35:04
}


// Serial.print to Flash: Notepad++ Replace RegEx
//     Find what:        Serial.print(.*)\("(.*)"\);
//     Replace with:    Serial.print\1\(F\("\2"\)\);



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions (global)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Debug Output

unsigned long debugOut_LiveSign_TimeStamp_LastAction = 0;
const uint16_t debugOut_LiveSign_UpdateInterval = 1000; //ms

boolean debugOut_LiveSign_Serial_Enabled = true;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MPR121

// MPR121 TWI Addresses:
// connect ADDR pin to
//  GND,   3V,  SDA,  SCL
// 0x5A, 0x5B, 0x5C, 0x5D
// defaults to 0x5A
slight_MPR121 myTouchSensor(
    0x5A  // TWI address
);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MPR121

void myTouchSensor_init(Print &out) {
    out.println(F("setup MPR121:")); {
        out.println(F("\t connect to sensor ..."));
        bool ready = myTouchSensor.begin();
        out.print("\t ");
        if (!ready) {
            out.print("not ");
        }
        out.print("found!");
        out.println();

        myTouchSensor.touch_event_set_callback(touch_event);

    }
    out.println(F("\t finished."));
}

void touch_event(slight_MPR121 *instance) {
    Serial.print(F("touched: "));
    for (size_t i=0; i<12; i++) {
        if (instance->touch_status_get() & (1 << i)) {
            Serial.print("1");
        } else {
            Serial.print("0");
        }
    }
    Serial.println();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// setup
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise PINs
    // as of arduino 1.0.1 you can use INPUT_PULLUP

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise serial

        delay(2000);

        Serial.begin(115200);

        // Wait for Serial Connection to be Opened from Host or
        // timeout after 6second
        uint32_t timeStamp_Start = millis();
        while( (! Serial) && ( (millis() - timeStamp_Start) < 6000 ) ) {
            // nothing to do
        }
        Serial.println();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome
        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup MPR121
        myTouchSensor_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // go

        Serial.println(F("Loop:"));

} /** setup **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// main loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TouchSensor things

        myTouchSensor.update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // debug output

        if (
            (millis() - debugOut_LiveSign_TimeStamp_LastAction) >
            debugOut_LiveSign_UpdateInterval
        ) {
            debugOut_LiveSign_TimeStamp_LastAction = millis();

            if ( debugOut_LiveSign_Serial_Enabled ) {
                Serial.print(millis());
                Serial.print(F("ms;"));
                Serial.println();
            }
        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // other things

} /** loop **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// THE END
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

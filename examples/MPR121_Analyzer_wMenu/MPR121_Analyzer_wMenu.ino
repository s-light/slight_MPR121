/******************************************************************************

    MPR121_Analyzer_wMenu.ino
        sketch to test all functions of the MPR121 sensor.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino compatible (with serial port)
            LED on pin 13
            MPR121
                TWI (SDA & SCL)
                IRQ --> connected to SCK


    libraries used:
        ~ slight_MPR121
        ~ slight_DebugMenu
            written by stefan krueger (s-light),
                git@s-light.eu, http://s-light.eu, https://github.com/s-light/
            license: MIT

    written by stefan krueger (s-light),
        git@s-light.eu, http://s-light.eu, https://github.com/s-light/

    changelog / history
        16.06.2016 17:33 created (based on MPR121_Simple_wMenu.ino)
        16.06.2016 17:33 renamed/structured

    TO DO:
        ~ enjoy your life ;-)


******************************************************************************/
/******************************************************************************
    The MIT License (MIT)

    Copyright (c) 2016 Stefan Krüger

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
******************************************************************************/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Includes
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// use "file.h" for files in same directory as .ino
// #include "file.h"
// use <file.h> for files in library directory
// #include <file.h>

#include <slight_DebugMenu.h>

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
    out.println(F("| MPR121_Analyzer_wMenu.ino"));
    out.println(F("|   sketch to test all functions of the"));
    out.println(F("|   MPR121 sensor."));
    out.println(F("|"));
    out.println(F("| This Sketch has a debug-menu:"));
    out.println(F("| send '?'+Return for help"));
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

boolean infoled_state = 0;
const byte infoled_pin = 13; //D9

unsigned long debugOut_LiveSign_TimeStamp_LastAction = 0;
const uint16_t debugOut_LiveSign_UpdateInterval = 1000; //ms

boolean debugOut_LiveSign_Serial_Enabled = 0;
boolean debugOut_LiveSign_LED_Enabled = 1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu

// slight_DebugMenu(Stream &in_ref, Print &out_ref, uint8_t input_length_new);
slight_DebugMenu myDebugMenu(Serial, Serial, 15);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MPR121


unsigned long debugOut_SensorInfo_TimeStamp_LastAction = 0;
uint16_t debugOut_SensorInfo_UpdateInterval = 1000; //ms

boolean debugOut_SensorInfo_Serial_Enabled = 0;
boolean debugOut_SensorInfo_Terminal_Special = 0;

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
// debug things

// freeRam found at
// http://forum.arduino.cc/index.php?topic=183790.msg1362282#msg1362282
// posted by mrburnette
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu System

void debugOut_SensorInfo_Terminal_Type(Print &out) {
    if (debugOut_SensorInfo_Terminal_Special) {
        out.print(F("special"));
    } else {
        out.print(F("normal"));
    }
}

// Main Menu
void handleMenu_Main(slight_DebugMenu *pInstance) {
    Print &out = pInstance->get_stream_out_ref();
    char *command = pInstance->get_command_current_pointer();
    // out.print("command: '");
    // out.print(command);
    // out.println("'");
    switch (command[0]) {
        case 'h':
        case 'H':
        case '?': {
            // help
            out.println(F("____________________________________________________________"));
            out.println();
            out.println(F("Help for Commands:"));
            out.println();
            out.println(F("\t '?': this help"));
            out.println(F("\t 'i': sketch info"));
            out.println(F("\t 'Y': toggle DebugOut livesign print"));
            // out.println(F("\t 'Y': toggle DebugOut livesign LED"));
            // out.println(F("\t 'x': tests"));
            out.println(F("\t 'x': SensorInfo toggle"));
            out.print(F("\t 'x': SensorInfo set terminal type (0=normal 1=special) 'y1'; "));
            debugOut_SensorInfo_Terminal_Type(out);
            out.println();
            out.print(F("\t 'X': SensorInfo set update interval 'X1000'; "));
            out.print(debugOut_SensorInfo_UpdateInterval);
            out.println(F("ms"));
            // ------------------------------------------
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.1 Main Control Registers - Gain
            out.print(F("\t 'g': gain set [1..8] 's8'; "));
            myTouchSensor.gain_print(out);
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.5 Sensitivity Control Register
            out.print(F("\t 's': sensitivity set [1, 2, .., 64, 128] 's128'; "));
            myTouchSensor.sensitivity_print(out);
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.7 Sensor Input Enable Registers
            out.print(F("\t 'e': sensor input enable set sensor'e0:1'; "));
            // out.print(F("\t 'e': sensor input enable set'e255' (3=S1+S2 en); "));
            slight_DebugMenu::print_Binary_8(
                out,
                myTouchSensor.sensor_input_enable_get()
            );
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.10 Averaging and Sampling Configuration Register
            // avg_samples
            out.print(F("\t 'A': avg samples set [1, 2, .., 64, 128] 'A8'; "));
            myTouchSensor.avg_samples_print(out);
            out.println();
            // sample_time
            out.print(F("\t 'S': sample time set [1..255: 320, 640, 1280, 2560] 'S128'; "));
            myTouchSensor.sample_time_print(out);
            out.println();
            // cycle_time
            out.print(F("\t 'C': cycle time set [1..255: 35, 70, 105, 140] 'C70'; "));
            myTouchSensor.cycle_time_print(out);
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.11 Calibration Activate Register
            out.print(F("\t 'a': calibration activate sensor 'a1'; "));
            slight_DebugMenu::print_Binary_8(
                out,
                myTouchSensor.calibration_activate_get()
            );
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.14 Multiple Touch Configuration Register
            out.print(F("\t 'b': multiple touch blocked enable set 'b1'; "));
            out.print(
                myTouchSensor.multiple_touch_blocking_enable_get()
            );
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.18 Sensor Input Threshold Registers
            out.print(F("\t 't': threshold set (0..127) 't1:127'; "));
                out.print(F(" 1:"));
                out.print(myTouchSensor.sensor_input_threshold_get(1));
                out.print(F(" 2:"));
                out.print(myTouchSensor.sensor_input_threshold_get(2));
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.19 Sensor Input Noise Threshold Register
            out.print(F("\t 'n': noise threshold set (0..255) 't37'; "));
                myTouchSensor.sensor_input_noise_threshold_print(out);
                out.print(F("%"));
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.24 Sensor Input Base Count Registers
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // HW reset things
            out.println(F("\t 'r': soft reset of sensor 'r1'"));
            out.println(F("\t 'R': HW sensor reset "));
            // out.println();
            out.println(F("____________________________________________________________"));
        } break;
        case 'i': {
            sketchinfo_print(out);
        } break;
        case 'y': {
            out.println(F("\t toggle DebugOut livesign Serial:"));
            debugOut_LiveSign_Serial_Enabled = !debugOut_LiveSign_Serial_Enabled;
            out.print(F("\t debugOut_LiveSign_Serial_Enabled:"));
            out.println(debugOut_LiveSign_Serial_Enabled);
        } break;
        // case 'Y': {
        //     out.println(F("\t toggle DebugOut livesign LED:"));
        //     debugOut_LiveSign_LED_Enabled = !debugOut_LiveSign_LED_Enabled;
        //     out.print(F("\t debugOut_LiveSign_LED_Enabled:"));
        //     out.println(debugOut_LiveSign_LED_Enabled);
        // } break;
        // case 'x': {
        //     // get state
        //     out.println(F("__________"));
        //     out.println(F("Tests:"));
        //
        //     out.println(F("nothing to do."));
        //
        //     // uint16_t wTest = 65535;
        //     uint16_t wTest = atoi(&command[1]);
        //     out.print(F("wTest: "));
        //     out.print(wTest);
        //     out.println();
        //
        //     out.print(F("1: "));
        //     out.print((byte)wTest);
        //     out.println();
        //
        //     out.print(F("2: "));
        //     out.print((byte)(wTest>>8));
        //     out.println();
        //
        //     out.println();
        //
        //     // set leds
        //     myTouchSensor.led_output_control_set_led(7, 1);
        //     myTouchSensor.led_output_control_set_led(8, 0);
        //
        //     out.println(F("__________"));
        // } break;
        case 'x': {
            out.print(F("\t SensorInfo Serial '"));
            out.print(&command[1]);
            out.print(F("' "));
            // out.print(command[1], HEX);
            // out.print(F(" "));
            if (command[1] != '\0') {
                // handle set type
                out.print(F("set type: "));
                uint8_t value = atoi(&command[1]);
                out.print(value);
                out.print(F(" "));
                if (value == 1) {
                    debugOut_SensorInfo_Terminal_Special = 1;
                } else {
                    debugOut_SensorInfo_Terminal_Special = 0;
                }
                debugOut_SensorInfo_Terminal_Type(out);
                out.println();
            } else {
                // handle toggle
                out.print(F("toggle: "));
                debugOut_SensorInfo_Serial_Enabled = !debugOut_SensorInfo_Serial_Enabled;
                out.print(F("\t debugOut_SensorInfo_Serial_Enabled:"));
                out.print(debugOut_SensorInfo_Serial_Enabled);
                out.println();
                if (debugOut_SensorInfo_Serial_Enabled) {
                    myTouchSensor_debugOut_print_start(out);
                }
            }
        } break;
        case 'X': {
            out.print(F("\t SensorInfo set update interval: "));
            uint16_t value = atoi(&command[1]);
            out.print(value);
            debugOut_SensorInfo_UpdateInterval = value;
            out.println();
        } break;
        //---------------------------------------------------------------------
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.1 Main Control Registers - Gain
        case 'g': {
            out.print(F("\t gain set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" --> "));
            myTouchSensor.gain_set(value);
            myTouchSensor.gain_print(out);
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.5 Sensitivity Control Register
        case 's': {
            out.print(F("\t sensitivity set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" --> "));
            myTouchSensor.sensitivity_set(value);
            myTouchSensor.sensitivity_print(out);
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.7 Sensor Input Enable Registers
        case 'e': {
            out.print(F("\t sensor input enable set sensor "));
            // a0:1
            uint8_t input = atoi(&command[1]);
            uint8_t value = atoi(&command[3]);
            out.print(input);
            out.print(F(":"));
            out.print(value);
            myTouchSensor.sensor_input_enable_set_sensor(input, value);
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.10 Averaging and Sampling Configuration Register
        // avg_samples
        case 'A': {
            out.print(F("\t avg samples set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" --> "));
            myTouchSensor.avg_samples_set(value);
            myTouchSensor.avg_samples_print(out);
            out.println();
        } break;
        // sample_time
        case 'S': {
            out.print(F("\t sample time set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" --> "));
            myTouchSensor.sample_time_set(value);
            myTouchSensor.sample_time_print(out);
            out.println();
        } break;
        // cycle_time
        case 'C': {
            out.print(F("\t cycle time set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" --> "));
            myTouchSensor.cycle_time_set(value);
            myTouchSensor.cycle_time_print(out);
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.11 Calibration Activate Register
        case 'a': {
            out.print(F("\t calibration activate sensor set "));
            // a0
            uint8_t input = atoi(&command[1]);
            // uint8_t value = atoi(&command[3]);
            out.print(input);
            // out.print(F(":"));
            // out.print(value);
            myTouchSensor.calibration_activate_sensor(input);
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.14 Multiple Touch Configuration Register
        case 'b': {
            out.print(F("\t multiple touch blocked enable set "));
            uint8_t value = atoi(&command[1]);
            out.print(value);
            myTouchSensor.multiple_touch_blocking_enable_set(value);
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.18 Sensor Input Threshold Registers
        case 't': {
            out.print(F("\t threshold set "));
            // t0:127
            uint8_t input = atoi(&command[1]);
            uint8_t value = atoi(&command[3]);
            out.print(input);
            out.print(F(":"));
            out.print(value);
            myTouchSensor.sensor_input_threshold_set(input, value);
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 5.19 Sensor Input Noise Threshold Register
        case 'n': {
            out.print(F("\t noise threshold set "));
            // t0:127
            uint8_t value = atoi(&command[1]);
            out.print(value);
            myTouchSensor.sensor_input_noise_threshold_set(value);
            out.print(F(" --> "));
            myTouchSensor.sensor_input_noise_threshold_print(out);
            out.print(F("%"));
            out.println();
        } break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // HW reset things
        case 'r': {
            out.print(F("\t reset without loading defaults."));
            // out.print(F(" -- TODO --"));
            // out.println();
            myTouchSensor.sensor_HW_reset();
            out.print(F(". done"));
            out.println();
            out.print(F("\t write app settings: "));
            // myTouchSensor_init(out);
            myTouchSensor_write_appsettings(out);
            out.print(F("done"));
            out.println();
        } break;
        case 'R': {
            out.print(F("\t HW sensor reset .."));
            myTouchSensor.sensor_HW_reset();
            out.print(F("\t load app defaults: "));
            myTouchSensor.sensor_default_configuration();
            out.print(F(". done"));
            out.println();
            out.print(F("\t write app settings: "));
            myTouchSensor_init(out);
            out.print(F("done"));
            out.println();
        } break;
        //---------------------------------------------------------------------
        default: {
            if(strlen(command) > 0) {
                out.print(F("command '"));
                out.print(command);
                out.println(F("' not recognized. try again."));
            }
            pInstance->get_command_input_pointer()[0] = '?';
                pInstance->set_flag_EOC(true);
        }
    } // end switch

    // end Command Parser
}

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
        if (instance->sensor_input_status_get() & (1 << i)) {
            Serial.print("1");
        } else {
            Serial.print("0");
        }
    }
    Serial.println();
}


void myTouchSensor_debugOut_print_start(Print &out) {
    if (debugOut_SensorInfo_Terminal_Special) {
        out.println();
        // remember cursor:

    } else {
        out.println();
        out.println();
        out.println();
        out.println();
        out.print(F("NoiseFlag, "));
        out.print(F("BaseCount, "));
        out.print(F("deltaCount, "));
        out.print(F("calibration, "));
        out.println();
    }
}

void myTouchSensor_debugOut_print(Print &out) {
    if (debugOut_SensorInfo_Terminal_Special) {
        // set cursor back to start:

    }
    // Line 1
    // out.print(millis());
    // out.print(F("ms;"));
    // // out.print(F("  free RAM = "));
    // // out.print(freeRam());
    // // out.print(F(";"));
    // // out.println();
    // // global things:
    // out.print(F(" Noise_Flag: "));
    // slight_DebugMenu::print_Binary_8(
    //     out,
    //     myTouchSensor.noise_flags_get()
    // );
    // out.print(F(" sensitivity: "));
    // myTouchSensor.sensitivity_print(out, myTouchSensor.sensitivity_get());
    // out.println();
    // Line 2
    // out.println(F("\t list: BaseC, deltaC, cali"));
    // Line 3
    // out.print(F("\t\t1: "));
    uint8_t sensor = 1;
    out.print(
        myTouchSensor.noise_flag_sensor_get(sensor)
    );
    out.print(F(", "));
    out.print(
        myTouchSensor.sensor_input_base_count_get(sensor)
    );
    out.print(F(", "));
    out.print(
        myTouchSensor.sensor_input_delta_count_get(sensor)
    );
    out.print(F(", "));
    out.print(
        myTouchSensor.sensor_input_calibration_value_get(sensor)
    );
    out.print(F(", "));

    out.println();
    // // Line 4
    // out.print(F("\t\t2: "));
    // out.print(
    //     myTouchSensor.sensor_input_base_count_get(2)
    // );
    // out.print(F(", "));
    // out.print(
    //     myTouchSensor.sensor_input_delta_count_get(2)
    // );
    // out.print(F(", "));
    // out.print(
    //     myTouchSensor.sensor_input_calibration_value_get(2)
    // );
    // out.println();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// setup
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise PINs

        //LiveSign
        pinMode(infoled_pin, OUTPUT);
        digitalWrite(infoled_pin, HIGH);

        // as of arduino 1.0.1 you can use INPUT_PULLUP

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise serial

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // wait for arduino IDE to release all serial ports after upload.
            delay(2000);
        #endif

        Serial.begin(115200);

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // Wait for Serial Connection to be Opend from Host or
            // timeout after 6second
            uint32_t timeStamp_Start = millis();
            while( (! Serial) && ( (millis() - timeStamp_Start) < 6000 ) ) {
                // nothing to do
            }
        #endif

        Serial.println();

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome

        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup MPR121

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

        myTouchSensor_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // show serial commands

        myDebugMenu.set_callback(handleMenu_Main);
        myDebugMenu.begin(true);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // go

        Serial.println(F("Loop:"));

} /** setup **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// main loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // menu input
        myDebugMenu.update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TouchSensor things

        myTouchSensor.update();

        if ( debugOut_SensorInfo_Serial_Enabled ) {
            if (
                (millis() - debugOut_SensorInfo_TimeStamp_LastAction) >
                debugOut_SensorInfo_UpdateInterval
            ) {
                debugOut_SensorInfo_TimeStamp_LastAction = millis();

                myTouchSensor_debugOut_print(Serial);
            }
        }

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
                Serial.print(F("  free RAM = "));
                Serial.println(freeRam());
            }

            if ( debugOut_LiveSign_LED_Enabled ) {
                infoled_state = ! infoled_state;
                if (infoled_state) {
                    //set LED to HIGH
                    digitalWrite(infoled_pin, HIGH);
                } else {
                    //set LED to LOW
                    digitalWrite(infoled_pin, LOW);
                }
            }

        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // other things

} /** loop **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// THE END
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/******************************************************************************

    written by stefan krueger (s-light),
        git@s-light.eu, http://s-light.eu, https://github.com/s-light/

******************************************************************************/
/******************************************************************************
    The MIT License (MIT)

    Copyright (c) 2015 Stefan Krüger

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


/** Includes Core Arduino functionality **/
#if ARDUINO
    #if ARDUINO < 100
        #include <WProgram.h>
    #else
        #include <Arduino.h>
    #endif
#endif

#include <Arduino.h>
#include <Wire.h>
#include "slight_MPR121.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// slight_MPR121 functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// slight_MPR121::slight_MPR121(const uint8_t interrupt_pin_new) {
//
// }

slight_MPR121::slight_MPR121(
    uint8_t twi_address_new,
    const uint8_t interrupt_pin_new
) :
    twi_address(twi_address_new),
    interrupt_pin(interrupt_pin_new)
{
    ready = false;

    touch_status = 0;
    touch_status_old = 0;

    timestamp_lastread = 0;
    update_interval = 90;
}

slight_MPR121::slight_MPR121(
    uint8_t twi_address_new
) :
    twi_address(twi_address_new),
    interrupt_pin(NO_PIN)
{
    ready = false;

    touch_status = 0;
    touch_status_old = 0;

    timestamp_lastread = 0;
    update_interval = 90;
}


bool slight_MPR121::begin() {
    if (ready == false) {
        // setup TWI
        Wire.begin();

        // set ready
        // otherwise write and read commands are not executed
        ready = true;

        // reset all registers
        // takes ~300ms
        soft_reset();

        // check if reading form sensor is working
        uint8_t reg_value = read_register(REG_Global_Config1);
        // should default to 0x24 after soft_reset
        // https://www.nxp.com/docs/en/data-sheet/MPR121.pdf#page=14&zoom=180,-292,767
        if (reg_value != 0x24) {
            Serial.println("reading from MPR121 register returned wrong value: ");
            Serial.println("  expected: 0x24");
            Serial.print("  read: 0x");
            Serial.print(reg_value);
            Serial.println("");
            ready = false;
            return false;
        }

        configuration_load_defaults();
    }
    return ready;
}

void slight_MPR121::update() {
    touch_status_update();
}

void slight_MPR121::configuration_load_defaults() {
    // setup 'good' starting configuration
    // calculate and set auto config values for 3.3V
    auto_config_calculate_values_and_set(33);
    // setup auto config to enabled with good defaults
    // auto_config_load_recommend_config();
    auto_config_load_recommend_config(&Serial);
}


void slight_MPR121::update_interval_set_autofit() {
    // TODO(s-light): implement update calculation with help from datasheet
    // 90ms is fine for default configuration??
    update_interval = 90;
}

void slight_MPR121::update_interval_set(uint32_t interval) {
    update_interval = interval;
}

uint32_t slight_MPR121::update_interval_get() {
    return update_interval;
}


void slight_MPR121::touch_event_set_callback(
    callback_t callback_function
) {
    callback_touch_event = callback_function;
}

// private
void slight_MPR121::touch_status_update() {
    // check if sensor is present
    if (ready) {
        // poll sensor every 90ms
        // at default configuration
        // this is the cycle time till all sensors are read.
        uint32_t duration = millis() - timestamp_lastread;
        if (duration > update_interval) {
            timestamp_lastread =  millis();

            // get current state
            uint16_t touch_status_raw = touch_status_read();
            // check for overcurrentflag
            if (touch_status_raw & touch_status_overcurrent_flag_mask) {
                touch_status_overcurrent_flag = true;
            } else  {
                touch_status_overcurrent_flag = false;
            }
            // filter overcurrentflag out
            touch_status = touch_status_raw & touch_status_electrode_mask;

            // filter for changes
            if (touch_status != touch_status_old) {
                touch_status_old = touch_status;

                // change in touch detected
                touch_event_callback();
                // Serial.print(F("touched: "));
                // for (size_t i=0; i<8; i++) {
                //     if (touch_status & (1 << i)) {
                //         Serial.print("1");
                //     } else {
                //         Serial.print("0");
                //     }
                // }
                // Serial.println();
            }  // filter for changes
        }  // update_interval
    }  // if ready
}


void slight_MPR121::touch_event_callback() {
    if (callback_touch_event) {
        callback_touch_event(this);
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// register helper
// capitle title numers are referencing to:
// http://www.nxp.com/files/sensors/doc/data_sheet/MPR121.pdf
// additional have a look at the Application Notes - they contain more info:
// http://www.nxp.com/pages/proximity-capacitive-touch-sensor-controller:MPR121?tab=Documentation_Tab&linkline=Application%20Notes
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.2 Touch Status Registers
uint16_t slight_MPR121::touch_status_read() {
    // read register
    uint16_t reg = read_register16bit(REG_Touch_Status0);
    return reg;
}

uint16_t slight_MPR121::touch_status_get() {
    // returns internally stored touch_status.
    // this will be updated in the 'update()' call.
    return touch_status;
}

bool slight_MPR121::touch_status_get_overcurrent_flag() {
    // returns internally stored touch_status_overcurrent_flag.
    // this will be updated in the 'update()' call.
    return touch_status_overcurrent_flag;
}

bool slight_MPR121::touch_status_get(uint8_t electrode) {
    electrode = electrode_bounded(electrode);
    uint8_t value = 0;
    // isolate
    value = touch_status & (1 << electrode);

    // convert to bool
    bool result = false;
    if (value > 0) {
        result = true;
    }
    return result;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.12 Out-Of-Range Status Registers

uint16_t slight_MPR121::outofrange_status_get_raw() {
    // read register
    uint16_t reg_value = read_register16bit(REG_OutOfRange_Status0);
    return reg_value;
}

uint16_t slight_MPR121::outofrange_status_get() {
    uint16_t reg_value = outofrange_status_get_raw();
    // isolate
    reg_value = reg_value & outofrange_electrode_mask;
    return reg_value;
}

bool slight_MPR121::outofrange_status_get_autoconfiguration_fail_flag() {
    uint16_t status_raw = outofrange_status_get_raw();
    bool fail_flag = false;
    fail_flag = status_raw & outofrange_ACFF_mask;
    return fail_flag;
}

bool slight_MPR121::outofrange_status_get_autoreconfiguration_fail_flag() {
    uint16_t status_raw = outofrange_status_get_raw();
    bool fail_flag = false;
    fail_flag = status_raw & outofrange_ARFF_mask;
    return fail_flag;
}

bool slight_MPR121::outofrange_status_get(uint8_t electrode) {
    electrode = electrode_bounded(electrode);

    uint8_t value = 0;
    // isolate single electrode
    value = outofrange_status_get() & (1 << electrode);

    // convert to bool
    bool result = false;
    if (value > 0) {
        result = true;
    }
    return result;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.3 Electrode Filtered Data Register
uint16_t slight_MPR121::electrode_filterd_data_get(uint8_t electrode) {
    electrode = electrode_bounded(electrode);
    // read register
    uint8_t reg = REG_Electrode_0_Filterd_Data_L + (electrode*2);
    uint16_t reg_value = read_register16bit(reg);
    return reg_value;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.4 Baseline Value Register
uint8_t slight_MPR121::electrode_baseline_value_get_raw(uint8_t electrode) {
    electrode = electrode_bounded(electrode);
    // read register
    uint8_t reg = REG_Electrode_0_Baseline_Value + electrode;
    uint8_t reg_value = read_register(reg);
    return reg_value;
}

uint16_t slight_MPR121::electrode_baseline_value_get(uint8_t electrode) {
    uint8_t reg_value = electrode_baseline_value_get_raw(electrode);
    // convert to 10bit value
    // so it is compatible with the Filterd_Data values
    uint16_t value = reg_value << 2;
    return value;
}

void slight_MPR121::electrode_baseline_value_set_raw(
    uint8_t electrode,
    uint8_t value
) {
    electrode = electrode_bounded(electrode);
    uint8_t reg = REG_Electrode_0_Baseline_Value + electrode;
    switch_mode_temporarily_to_allow_write();
    write_register(reg, value);
    switch_mode_restore();
}

void slight_MPR121::electrode_baseline_value_set(
    uint8_t electrode,
    uint16_t value
) {
    // convert to internal 8bit MSB value
    uint8_t value_raw = (uint8_t)(value >> 2);
    electrode_baseline_value_set_raw(electrode, value_raw);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.5 Baseline Filtering Control Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.6 Touch / Release Threshold
uint8_t slight_MPR121::electrode_touch_threshold_get(uint8_t electrode) {
    electrode = electrode_bounded(electrode);
    // read register
    uint8_t reg = REG_Electrode_0_Touch_Threshold + (electrode*2);
    uint8_t reg_value = read_register(reg);
    return reg_value;
}

void slight_MPR121::electrode_touch_threshold_set(
    uint8_t electrode, uint8_t value
) {
    electrode = electrode_bounded(electrode);
    uint8_t reg = REG_Electrode_0_Touch_Threshold + (electrode*2);
    switch_mode_temporarily_to_allow_write();
    write_register(reg, value);
    switch_mode_restore();
}

uint8_t slight_MPR121::electrode_release_threshold_get(uint8_t electrode) {
    electrode = electrode_bounded(electrode);
    // read register
    uint8_t reg = REG_Electrode_0_Release_Threshold + (electrode*2);
    uint8_t reg_value = read_register(reg);
    return reg_value;
}

void slight_MPR121::electrode_release_threshold_set(
    uint8_t electrode, uint8_t value
) {
    electrode = electrode_bounded(electrode);
    uint8_t reg = REG_Electrode_0_Release_Threshold + (electrode*2);
    switch_mode_temporarily_to_allow_write();
    write_register(reg, value);
    switch_mode_restore();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.7 Debounce Register
uint8_t slight_MPR121::electrode_debounce_touch_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Debounce,
        electrode_debounce_touch_mask,
        electrode_debounce_touch_shift
    );
    return reg_value;
}

void slight_MPR121::electrode_debounce_touch_set(uint8_t value) {
    // write register
    write_register_part(
        REG_Debounce,
        electrode_debounce_touch_mask,
        electrode_debounce_touch_shift,
        value
    );
}

uint8_t slight_MPR121::electrode_debounce_release_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Debounce,
        electrode_debounce_release_mask,
        electrode_debounce_release_shift
    );
    return reg_value;
}

void slight_MPR121::electrode_debounce_release_set(uint8_t value) {
    // write register
    write_register_part(
        REG_Debounce,
        electrode_debounce_release_mask,
        electrode_debounce_release_shift,
        value
    );
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.8 Filter and Global CDC CDT Configuration

// REG_Global_Config0
uint8_t slight_MPR121::global_config_charge_discharge_current_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Global_Config0,
        global_config_charge_discharge_current_mask,
        global_config_charge_discharge_current_shift
    );
    return reg_value;
}

void slight_MPR121::global_config_charge_discharge_current_set(uint8_t value) {
    // write register
    write_register_part(
        REG_Global_Config0,
        global_config_charge_discharge_current_mask,
        global_config_charge_discharge_current_shift,
        value
    );
}

slight_MPR121::first_filter_iterations_t slight_MPR121::global_config_first_filter_iterations_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Global_Config0,
        global_config_first_filter_iterations_mask,
        global_config_first_filter_iterations_shift
    );
    return (first_filter_iterations_t)reg_value;
}

void slight_MPR121::global_config_first_filter_iterations_set(
    first_filter_iterations_t value
) {
    // write register
    write_register_part(
        REG_Global_Config0,
        global_config_first_filter_iterations_mask,
        global_config_first_filter_iterations_shift,
        (uint8_t)value
    );
}

void slight_MPR121::global_config_first_filter_iterations_set(uint8_t value) {
    // write register
    global_config_first_filter_iterations_set(
        first_filter_iterations_convert(value)
    );
}

// REG_Global_Config1
slight_MPR121::global_config_esi_t slight_MPR121::global_config_electrode_sample_interval_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Global_Config1,
        global_config_electrode_sample_interval_mask,
        global_config_electrode_sample_interval_shift
    );
    return (global_config_esi_t)reg_value;
}

void slight_MPR121::global_config_electrode_sample_interval_set(global_config_esi_t value) {
    // write register
    write_register_part(
        REG_Global_Config1,
        global_config_electrode_sample_interval_mask,
        global_config_electrode_sample_interval_shift,
        (uint8_t)value
    );
}

void slight_MPR121::global_config_electrode_sample_interval_set(uint8_t value) {
    // write register
    global_config_electrode_sample_interval_set(
        global_config_electrode_sample_interval_convert(value)
    );
}


slight_MPR121::global_config_sfi_t slight_MPR121::global_config_second_filter_iterations_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Global_Config1,
        global_config_second_filter_iterations_mask,
        global_config_second_filter_iterations_shift
    );
    return (global_config_sfi_t)reg_value;
}

void slight_MPR121::global_config_second_filter_iterations_set(uint8_t value) {
    global_config_second_filter_iterations_set(
        global_config_second_filter_iterations_convert(value)
    );
}

void slight_MPR121::global_config_second_filter_iterations_set(
    slight_MPR121::global_config_sfi_t value
) {
    // write register
    write_register_part(
        REG_Global_Config1,
        global_config_second_filter_iterations_mask,
        global_config_second_filter_iterations_shift,
        (uint8_t)value
    );
}

void slight_MPR121::global_config_second_filter_iterations_print(
    Print &out,
    slight_MPR121::global_config_sfi_t value
) {
    // out.print(F("baseline "));
    switch (value) {
        case sfi_4: {
            out.print(F("4"));
        } break;
        case sfi_6: {
            out.print(F("6"));
        } break;
        case sfi_10: {
            out.print(F("10"));
        } break;
        case sfi_18: {
            out.print(F("18"));
        } break;
    }
}

void slight_MPR121::global_config_second_filter_iterations_print(Print &out) {
    global_config_second_filter_iterations_print(
        out,
        global_config_second_filter_iterations_get()
    );
}

slight_MPR121::global_config_sfi_t slight_MPR121::global_config_second_filter_iterations_convert(
    uint8_t value
) {
    global_config_sfi_t result = sfi_6;
    switch (value) {
        case 0: {
            result = sfi_4;
        } break;
        case 1: {
            result = sfi_6;
        } break;
        case 2: {
            result = sfi_10;
        } break;
        case 3: {
            result = sfi_18;
        } break;
        default: {
            result = sfi_6;
        } break;
    }
    return result;
}





slight_MPR121::charge_discharge_time_t slight_MPR121::global_config_charge_discharge_time_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Global_Config1,
        global_config_charge_discharge_time_mask,
        global_config_charge_discharge_time_shift
    );
    return (charge_discharge_time_t)reg_value;
}

void slight_MPR121::global_config_charge_discharge_time_set(charge_discharge_time_t value) {
    // write register
    write_register_part(
        REG_Global_Config1,
        global_config_charge_discharge_time_mask,
        global_config_charge_discharge_time_shift,
        (uint8_t)value
    );
}

void slight_MPR121::global_config_charge_discharge_time_set(uint8_t value) {
    global_config_charge_discharge_time_set(
        charge_discharge_time_convert(value)
    );
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.9 Electrode Charge Current Register
uint8_t slight_MPR121::electrode_charge_current_get(uint8_t electrode) {
    electrode = electrode_bounded(electrode);
    // read register
    uint8_t reg = REG_Electrode_0_Current + (electrode);
    uint8_t reg_value = read_register(reg);
    return reg_value;
}

void slight_MPR121::electrode_charge_current_set(
    uint8_t electrode, uint8_t value
) {
    electrode = electrode_bounded(electrode);
    value = value_limit(
        electrode_charge_current_mask,
        electrode_charge_current_shift,
        value
    );
    uint8_t reg = REG_Electrode_0_Current + (electrode);
    switch_mode_temporarily_to_allow_write();
    write_register(reg, value);
    switch_mode_restore();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.10 Electrode Charge Time Register
uint8_t slight_MPR121::electrode_charge_time_get(uint8_t electrode) {
    electrode = electrode_bounded(electrode);
    // read register
    uint8_t reg = REG_Electrode_0_1_Time + (electrode);
    uint8_t mask = electrode_charge_time_even_mask;
    uint8_t shift = electrode_charge_time_even_shift;
    // check if we have a even or odd electrode number
    if (electrode % 2) {
        // odd (1,3,5,7,11)
        // change mask
        mask = electrode_charge_time_odd_mask;
        shift = electrode_charge_time_odd_shift;
    } else {
        // even (0,2,4,6,8,10,12)
    }
    uint8_t reg_value = read_register_part(reg, mask, shift);
    return reg_value;
}

void slight_MPR121::electrode_charge_time_set(
    uint8_t electrode,
    uint8_t value
) {
    electrode = electrode_bounded(electrode);
    value = value_limit(
        electrode_charge_time_even_mask,
        electrode_charge_time_even_shift,
        value
    );
    uint8_t reg = REG_Electrode_0_1_Time + (electrode);
    uint8_t mask = electrode_charge_time_even_mask;
    uint8_t shift = electrode_charge_time_even_shift;
    // check if we have a even or odd electrode number
    if (electrode % 2) {
        // odd (1,3,5,7,11)
        // change mask
        mask = electrode_charge_time_odd_mask;
        shift = electrode_charge_time_odd_shift;
    } else {
        // even (0,2,4,6,8,10,12)
    }
    write_register_part(reg, mask, shift, value);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.11 Electrode Configuration Register

void slight_MPR121::electrode_config_set(
    uint8_t value
) {
    // write register
    // switch_mode_temporarily_to_allow_write();
    write_register(
        REG_Electrode_Configuration,
        value
    );
    // switch_mode_restore();
}

uint8_t slight_MPR121::electrode_config_get() {
    // read register
    uint8_t value = read_register(
        REG_Electrode_Configuration
    );
    return value;
}

// electrode enable
slight_MPR121::electrode_config_electrode_enable_t slight_MPR121::electrode_config_electrode_enable_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Electrode_Configuration,
        electrode_config_electrode_enable_mask,
        electrode_config_electrode_enable_shift
    );
    return (electrode_config_electrode_enable_t)reg_value;
}

void slight_MPR121::electrode_config_electrode_enable_set(
    slight_MPR121::electrode_config_electrode_enable_t value
) {
    // write register
    write_register_part(
        REG_Electrode_Configuration,
        electrode_config_electrode_enable_mask,
        electrode_config_electrode_enable_shift,
        (uint8_t)value,
        false // disable switch_to_stopmode
    );
}

void slight_MPR121::electrode_config_electrode_enable_set(uint8_t value) {
    electrode_config_electrode_enable_set(
        electrode_config_electrode_enable_convert(value)
    );
}


void slight_MPR121::electrode_config_electrode_enable_print(
    Print &out,
    slight_MPR121::electrode_config_electrode_enable_t value
) {
    // out.print(F("electrode enable "));
    switch (value) {
        case electrode_config_electrode_enable_off: {
            out.print(F("off"));
        } break;
        case electrode_config_electrode_enable_ELE0: {
            out.print(F("ELE0"));
        } break;
        case electrode_config_electrode_enable_ELE0_ELE1:
        case electrode_config_electrode_enable_ELE0_ELE2:
        case electrode_config_electrode_enable_ELE0_ELE3:
        case electrode_config_electrode_enable_ELE0_ELE4:
        case electrode_config_electrode_enable_ELE0_ELE5:
        case electrode_config_electrode_enable_ELE0_ELE6:
        case electrode_config_electrode_enable_ELE0_ELE7:
        case electrode_config_electrode_enable_ELE0_ELE8:
        case electrode_config_electrode_enable_ELE0_ELE9:
        case electrode_config_electrode_enable_ELE0_ELE10:
        case electrode_config_electrode_enable_ELE0_ELE11: {
            out.print(F("ELE0 - "));
            switch (value) {
                case electrode_config_electrode_enable_off: {
                    // should never happen
                } break;
                case electrode_config_electrode_enable_ELE0: {
                    // should never happen
                } break;
                case electrode_config_electrode_enable_ELE0_ELE1: {
                    out.print(F("ELE1"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE2: {
                    out.print(F("ELE2"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE3: {
                    out.print(F("ELE3"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE4: {
                    out.print(F("ELE4"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE5: {
                    out.print(F("ELE5"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE6: {
                    out.print(F("ELE6"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE7: {
                    out.print(F("ELE7"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE8: {
                    out.print(F("ELE8"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE9: {
                    out.print(F("ELE9"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE10: {
                    out.print(F("ELE10"));
                } break;
                case electrode_config_electrode_enable_ELE0_ELE11: {
                    out.print(F("ELE11"));
                } break;
            }
        } break;
    }
}

void slight_MPR121::electrode_config_electrode_enable_print(Print &out) {
    electrode_config_electrode_enable_print(
        out,
        electrode_config_electrode_enable_get()
    );
}

slight_MPR121::electrode_config_electrode_enable_t slight_MPR121::electrode_config_electrode_enable_convert(
    uint8_t value
) {
    electrode_config_electrode_enable_t result = electrode_config_electrode_enable_off;
    switch (value) {
        case 0: {
            result = electrode_config_electrode_enable_off;
        } break;
        case 1: {
            result = electrode_config_electrode_enable_ELE0;
        } break;
        case 2: {
            result = electrode_config_electrode_enable_ELE0_ELE1;
        } break;
        case 3: {
            result = electrode_config_electrode_enable_ELE0_ELE2;
        } break;
        case 4: {
            result = electrode_config_electrode_enable_ELE0_ELE3;
        } break;
        case 5: {
            result = electrode_config_electrode_enable_ELE0_ELE4;
        } break;
        case 6: {
            result = electrode_config_electrode_enable_ELE0_ELE5;
        } break;
        case 7: {
            result = electrode_config_electrode_enable_ELE0_ELE6;
        } break;
        case 8: {
            result = electrode_config_electrode_enable_ELE0_ELE7;
        } break;
        case 9: {
            result = electrode_config_electrode_enable_ELE0_ELE8;
        } break;
        case 10: {
            result = electrode_config_electrode_enable_ELE0_ELE9;
        } break;
        case 11: {
            result = electrode_config_electrode_enable_ELE0_ELE10;
        } break;
        case 12: {
            result = electrode_config_electrode_enable_ELE0_ELE11;
        } break;
        default: {
            result = electrode_config_electrode_enable_ELE0_ELE11;
        } break;
    }
    return result;
}

// proximity enable
slight_MPR121::electrode_config_proximity_enable_t slight_MPR121::electrode_config_proximity_enable_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Electrode_Configuration,
        electrode_config_proximity_enable_mask,
        electrode_config_proximity_enable_shift
    );
    return (electrode_config_proximity_enable_t)reg_value;
}

void slight_MPR121::electrode_config_proximity_enable_set(
    slight_MPR121::electrode_config_proximity_enable_t value
) {
    // write register
    write_register_part(
        REG_Electrode_Configuration,
        electrode_config_proximity_enable_mask,
        electrode_config_proximity_enable_shift,
        (uint8_t)value,
        false // disable switch_to_stopmode
    );
}

void slight_MPR121::electrode_config_proximity_enable_set(uint8_t value) {
    electrode_config_proximity_enable_set(
        electrode_config_proximity_enable_convert(value)
    );
}


void slight_MPR121::electrode_config_proximity_enable_print(
    Print &out,
    slight_MPR121::electrode_config_proximity_enable_t value
) {
    // out.print(F("Proximity enable "));
    switch (value) {
        case electrode_config_proximity_enable_off: {
            out.print(F("off"));
        } break;
        case electrode_config_proximity_enable_ELE0_ELE1: {
            out.print(F("ELE0 - ELE1"));
        } break;
        case electrode_config_proximity_enable_ELE0_ELE3: {
            out.print(F("ELE0 - ELE3"));
        } break;
        case electrode_config_proximity_enable_ELE0_ELE11: {
            out.print(F("ELE0 - ELE11"));
        } break;
    }
}

void slight_MPR121::electrode_config_proximity_enable_print(Print &out) {
    electrode_config_proximity_enable_print(
        out,
        electrode_config_proximity_enable_get()
    );
}

slight_MPR121::electrode_config_proximity_enable_t slight_MPR121::electrode_config_proximity_enable_convert(
    uint8_t value
) {
    electrode_config_proximity_enable_t result = electrode_config_proximity_enable_off;

    if (value < 1) {
        result = electrode_config_proximity_enable_off;
    } else {
        if (value < 3) {
            result = electrode_config_proximity_enable_ELE0_ELE1;
        } else {
            if (value < 5) {
                result = electrode_config_proximity_enable_ELE0_ELE3;
            } else {
                result = electrode_config_proximity_enable_ELE0_ELE11;
            }
        }
    }

    return result;
}

// Baseline Tracking (= Calibration Lock)
slight_MPR121::electrode_config_baseline_tracking_t slight_MPR121::electrode_config_baseline_tracking_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_Electrode_Configuration,
        electrode_config_baseline_tracking_mask,
        electrode_config_baseline_tracking_shift
    );
    return (electrode_config_baseline_tracking_t)reg_value;
}

void slight_MPR121::electrode_config_baseline_tracking_set(
    slight_MPR121::electrode_config_baseline_tracking_t value
) {
    // write register
    write_register_part(
        REG_Electrode_Configuration,
        electrode_config_baseline_tracking_mask,
        electrode_config_baseline_tracking_shift,
        (uint8_t)value,
        false // disable switch_to_stopmode
    );
}

void slight_MPR121::electrode_config_baseline_tracking_set(uint8_t value) {
    electrode_config_baseline_tracking_set(
        electrode_config_baseline_tracking_convert(value)
    );
}


void slight_MPR121::electrode_config_baseline_tracking_print(
    Print &out,
    slight_MPR121::electrode_config_baseline_tracking_t value
) {
    // out.print(F("baseline tracking "));
    switch (value) {
        case electrode_config_baseline_tracking_no_change: {
            out.print(F("enabled - no change"));
        } break;
        case electrode_config_baseline_tracking_disabled: {
            out.print(F("disabled"));
        } break;
        case electrode_config_baseline_tracking_init_wfirst_5hbits: {
            out.print(F("enabled - on init load 5 high bits from first reading"));
        } break;
        case electrode_config_baseline_tracking_init_wfirst_10bits: {
            out.print(F("enabled - on init load 10bits from first reading"));
        } break;
    }
}

void slight_MPR121::electrode_config_baseline_tracking_print(Print &out) {
    electrode_config_baseline_tracking_print(
        out,
        electrode_config_baseline_tracking_get()
    );
}

slight_MPR121::electrode_config_baseline_tracking_t slight_MPR121::electrode_config_baseline_tracking_convert(
    uint8_t value
) {
    electrode_config_baseline_tracking_t result = electrode_config_baseline_tracking_no_change;

    switch (value) {
        case 0: {
            result = electrode_config_baseline_tracking_no_change;
        } break;
        case 1: {
            result = electrode_config_baseline_tracking_disabled;
        } break;
        case 2: {
            result = electrode_config_baseline_tracking_init_wfirst_5hbits;
        } break;
        case 3: {
            result = electrode_config_baseline_tracking_init_wfirst_10bits;
        } break;
        default: {
            result = electrode_config_baseline_tracking_init_wfirst_5hbits;
        } break;
    }

    return result;
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.11 Auto-Configuration Registers

// REG_AutoConfiguration_Control0
bool slight_MPR121::auto_config_enable_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_enable_mask,
        auto_config_enable_shift
    );
    bool reg_flag = false;
    if (reg_value) {
        reg_flag = true;
    }
    return reg_flag;
}

void slight_MPR121::auto_config_enable_set(bool value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_enable_mask,
        auto_config_enable_shift,
        value
    );
}

bool slight_MPR121::auto_reconfig_enable_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control0,
        auto_reconfig_enable_mask,
        auto_reconfig_enable_shift
    );
    bool reg_flag = false;
    if (reg_value) {
        reg_flag = true;
    }
    return reg_flag;
}

void slight_MPR121::auto_reconfig_enable_set(bool value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control0,
        auto_reconfig_enable_mask,
        auto_reconfig_enable_shift,
        value
    );
}

// bva baseline value adjust
slight_MPR121::auto_config_bva_t slight_MPR121::auto_config_bva_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_bva_mask,
        auto_config_bva_shift
    );
    return (auto_config_bva_t)reg_value;
}

void slight_MPR121::auto_config_bva_set(
    slight_MPR121::auto_config_bva_t value
) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_bva_mask,
        auto_config_bva_shift,
        (uint8_t)value
    );
}

void slight_MPR121::auto_config_bva_set(uint8_t value) {
    auto_config_bva_set(
        auto_config_bva_convert(value)
    );
}


void slight_MPR121::auto_config_bva_print(
    Print &out,
    slight_MPR121::auto_config_bva_t value
) {
    // out.print(F("baseline "));
    switch (value) {
        case baseline_not_changed: {
            out.print(F("not changed"));
        } break;
        case baseline_cleared: {
            out.print(F("cleared"));
        } break;
        case baseline_autoconfig_lower3bits_cleared: {
            out.print(F("autoconfig lower 3bits cleared"));
        } break;
        case baseline_autoconfig: {
            out.print(F("autoconfig"));
        } break;
    }
}

void slight_MPR121::auto_config_bva_print(Print &out) {
    auto_config_bva_print(
        out,
        auto_config_bva_get()
    );
}

slight_MPR121::auto_config_bva_t slight_MPR121::auto_config_bva_convert(
    uint8_t value
) {
    auto_config_bva_t result = baseline_not_changed;
    switch (value) {
        case 0: {
            result = baseline_not_changed;
        } break;
        case 1: {
            result = baseline_cleared;
        } break;
        case 2: {
            result = baseline_autoconfig_lower3bits_cleared;
        } break;
        case 3: {
            result = baseline_autoconfig;
        } break;
        default: {
            result = baseline_not_changed;
        } break;
    }
    return result;
}


// retry
slight_MPR121::auto_config_retry_t slight_MPR121::auto_config_retry_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_retry_mask,
        auto_config_retry_shift
    );
    return (auto_config_retry_t)reg_value;
}

void slight_MPR121::auto_config_retry_set(auto_config_retry_t value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_retry_mask,
        auto_config_retry_shift,
        (uint8_t)value
    );
}

void slight_MPR121::auto_config_retry_set(uint8_t value) {
    auto_config_retry_set(
        auto_config_retry_convert(value)
    );
}

void slight_MPR121::auto_config_retry_print(Print &out) {
    auto_config_retry_print(out, auto_config_retry_get());
}

void slight_MPR121::auto_config_retry_print(
    Print &out,
    slight_MPR121::auto_config_retry_t value
) {
    switch (value) {
        case retry_0: {
            out.print(F("0"));
        } break;
        case retry_2: {
            out.print(F("2"));
        } break;
        case retry_4: {
            out.print(F("4"));
        } break;
        case retry_8: {
            out.print(F("8"));
        } break;
    }

    // out.print(F("gain "));
    // out.print(value >> 6, DEC);
}

slight_MPR121::auto_config_retry_t slight_MPR121::auto_config_retry_convert(
    uint8_t value
) {
    auto_config_retry_t result = retry_0;
    if (value < 2) {
        result = retry_0;
    } else {
        if (value < 4) {
            result = retry_2;
        } else {
            if (value < 8) {
                result = retry_4;
            } else {
                result = retry_8;
            }
        }
    }
    return result;
}

// first filter iteration
uint8_t slight_MPR121::auto_config_first_filter_iteration_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_first_filter_iteration_mask,
        auto_config_first_filter_iteration_shift
    );
    return reg_value;
}

void slight_MPR121::auto_config_first_filter_iteration_set(uint8_t value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control0,
        auto_config_first_filter_iteration_mask,
        auto_config_first_filter_iteration_shift,
        value
    );
}

// REG_AutoConfiguration_Control1
bool slight_MPR121::auto_config_fail_interrupt_enable_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control1,
        auto_config_fail_interrupt_enable_mask,
        auto_config_fail_interrupt_enable_shift
    );
    bool reg_flag = false;
    if (reg_value) {
        reg_flag = true;
    }
    return reg_flag;
}

void slight_MPR121::auto_config_fail_interrupt_enable_set(bool value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control1,
        auto_config_fail_interrupt_enable_mask,
        auto_config_fail_interrupt_enable_shift,
        value
    );
}

bool slight_MPR121::auto_reconfig_fail_interrupt_enable_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control1,
        auto_reconfig_fail_interrupt_enable_mask,
        auto_reconfig_fail_interrupt_enable_shift
    );
    bool reg_flag = false;
    if (reg_value) {
        reg_flag = true;
    }
    return reg_flag;
}

void slight_MPR121::auto_reconfig_fail_interrupt_enable_set(bool value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control1,
        auto_reconfig_fail_interrupt_enable_mask,
        auto_reconfig_fail_interrupt_enable_shift,
        value
    );
}

bool slight_MPR121::auto_config_out_of_range_interrupt_enable_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control1,
        auto_config_out_of_range_interrupt_enable_mask,
        auto_config_out_of_range_interrupt_enable_shift
    );
    bool reg_flag = false;
    if (reg_value) {
        reg_flag = true;
    }
    return reg_flag;
}

void slight_MPR121::auto_config_out_of_range_interrupt_enable_set(bool value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control1,
        auto_config_out_of_range_interrupt_enable_mask,
        auto_config_out_of_range_interrupt_enable_shift,
        value
    );
}

bool slight_MPR121::auto_config_skip_charge_time_search_get() {
    // read register
    uint8_t reg_value = read_register_part(
        REG_AutoConfiguration_Control1,
        auto_config_skip_charge_time_search_mask,
        auto_config_skip_charge_time_search_shift
    );
    bool reg_flag = false;
    if (reg_value) {
        reg_flag = true;
    }
    return reg_flag;
}

void slight_MPR121::auto_config_skip_charge_time_search_set(bool value) {
    // write register
    write_register_part(
        REG_AutoConfiguration_Control1,
        auto_config_skip_charge_time_search_mask,
        auto_config_skip_charge_time_search_shift,
        value
    );
}

// REG_AutoConfiguration_Up_Side_Limit
uint8_t slight_MPR121::auto_config_up_side_limit_get_raw()  {
    // read register
    uint8_t reg_value = read_register(
        REG_AutoConfiguration_Up_Side_Limit
    );
    return reg_value;
}
uint16_t slight_MPR121::auto_config_up_side_limit_get()  {
    uint8_t reg_value = auto_config_up_side_limit_get_raw();
    // convert to 10bit value
    // so it is compatible with the Filterd_Data values
    uint16_t value = reg_value << 2;
    return value;
}

void slight_MPR121::auto_config_up_side_limit_set_raw(uint8_t value) {
    // write register
    write_register(
        REG_AutoConfiguration_Up_Side_Limit,
        value
    );
}
void slight_MPR121::auto_config_up_side_limit_set(uint16_t value)  {
    // convert to internal 8bit MSB value
    uint8_t value_raw = (uint8_t)(value >> 2);
    auto_config_up_side_limit_set_raw(value_raw);
}

// REG_AutoConfiguration_Low_Side_Limit
uint8_t slight_MPR121::auto_config_low_side_limit_get_raw()  {
    // read register
    uint8_t reg_value = read_register(
        REG_AutoConfiguration_Low_Side_Limit
    );
    return reg_value;
}
uint16_t slight_MPR121::auto_config_low_side_limit_get()  {
    uint8_t reg_value = auto_config_low_side_limit_get_raw();
    // convert to 10bit value
    // so it is compatible with the Filterd_Data values
    uint16_t value = reg_value << 2;
    return value;
}

void slight_MPR121::auto_config_low_side_limit_set_raw(uint8_t value) {
    // write register
    write_register(
        REG_AutoConfiguration_Low_Side_Limit,
        value
    );
}
void slight_MPR121::auto_config_low_side_limit_set(uint16_t value)  {
    // convert to internal 8bit MSB value
    uint8_t value_raw = (uint8_t)(value >> 2);
    auto_config_low_side_limit_set_raw(value_raw);
}

// REG_AutoConfiguration_Target_Level
uint8_t slight_MPR121::auto_config_target_level_get_raw()  {
    // read register
    uint8_t reg_value = read_register(
        REG_AutoConfiguration_Target_Level
    );
    return reg_value;
}
uint16_t slight_MPR121::auto_config_target_level_get()  {
    uint8_t reg_value = auto_config_target_level_get_raw();
    // convert to 10bit value
    // so it is compatible with the Filterd_Data values
    uint16_t value = reg_value << 2;
    return value;
}

void slight_MPR121::auto_config_target_level_set_raw(uint8_t value) {
    // write register
    write_register(
        REG_AutoConfiguration_Target_Level,
        value
    );
}
void slight_MPR121::auto_config_target_level_set(uint16_t value)  {
    // convert to internal 8bit MSB value
    uint8_t value_raw = (uint8_t)(value >> 2);
    auto_config_target_level_set_raw(value_raw);
}

// Auto Configuration load full automatics
void slight_MPR121::auto_config_calculate_values_and_set(uint8_t value_vcc)  {
    // calculate Lower-Side Limit, Upper-Side Limit and Target Level
    // following AN3889 - MPR121 Capacitance Sensing Settings
    // http://cache.nxp.com/files/sensors/doc/app_note/AN3889.pdf

    // these values are all internal MSB 8bit of a 10Bit value.
    // so if you want to compare theme you have to shift theme << 2.

    // initialize with default values for 1,8V system as AN3889:
    uint8_t value_usl  = 156;
    uint8_t value_tl  = 140;
    uint8_t value_lsl  = 101;

    // vcc_value defaults to 33 = 3.3V

    // value_vcc bound
    if (value_vcc > 50) {
        value_vcc = 50;
    }
    if (value_vcc < 18) {
        value_vcc = 18;
    }


    // calculate Upper-Side Limit:
    // USL = ((VCC - 0.7) / VCC) * 256
    // usl step1: VCC - 7
    uint8_t usl_step1 = value_vcc - 7;
    // multiply with 1000 to get enough precision.
    uint16_t usl_step2 = usl_step1 * 1000;
    // usl_step3: usl_step2 / VCC
    uint16_t usl_step3 = usl_step2 / value_vcc;
    // usl_step4: usl_step3 * 256
    uint32_t usl_step4 = usl_step3 * 256;
    // usl_step5: usl_step2 / 1000
    value_usl = usl_step4 / 1000;

    // calculate Lower-Side Limit:
    // LSL = USL * 0,65
    // lsl_step1: multiply USL * 65
    uint16_t lsl_step1 = value_usl * 65;
    // lsl_step2: lsl_step1 / 100
    value_lsl = lsl_step1 / 100;

    // calculate Target Level:
    // TL = USL * 0,9
    // tl_step1: multiply USL * 90
    uint16_t tl_step1 = value_usl * 90;
    // tl_step2: tl_step1 / 100
    value_tl = tl_step1 / 100;

    // set values:

    auto_config_up_side_limit_set_raw(value_usl);
    auto_config_low_side_limit_set_raw(value_lsl);
    auto_config_target_level_set_raw(value_tl);
}

// void slight_MPR121::auto_config_load_recommend_config()  {
//     auto_config_load_recommend_config(Serial);
// }

// void slight_MPR121::auto_config_load_recommend_config(Print &out)  {
void slight_MPR121::auto_config_load_recommend_config(Print *out)  {
    // load all config registers to recommend values.
    // following AN3889 - MPR121 Capacitance Sensing Settings
    // http://cache.nxp.com/files/sensors/doc/app_note/AN3889.pdf

    if (out) {
        (*out).println(F("auto config - load recommend configuration:"));
    }

    // setup global configurations
    if (out) {
        (*out).println(F("\t setup global configuration registers"));
    }
    // Charge Current defaults to 16uA
    global_config_charge_discharge_current_set(16);
    // first filter iterations default is 6 samples
    first_filter_iterations_t ffi_value = slight_MPR121::ffi_6;
    global_config_first_filter_iterations_set(ffi_value);

    // electrode sample interval default is 16ms
    global_config_electrode_sample_interval_set(slight_MPR121::esi_16ms);
    // second filter iterations default is 6 samples
    global_config_second_filter_iterations_set(slight_MPR121::sfi_6);
    // charge discharge time default is 0.5us
    global_config_charge_discharge_time_set(slight_MPR121::cdt_05us);

    // auto configuration registers
    if (out) {
        (*out).println(F("\t setup auto configuration registers"));
    }
    // both first_filter_iterations values should match.
    auto_config_first_filter_iteration_set(ffi_value);
    auto_config_retry_set(slight_MPR121::retry_0);
    auto_config_bva_set(
        slight_MPR121::baseline_autoconfig_lower3bits_cleared
    );
    auto_config_enable_set(true);
    auto_reconfig_enable_set(true);

    // setup electrode configurations
    if (out) {
        (*out).println(F("\t setup electrode configuration registers:"));
    }
    electrode_config_electrode_enable_set(
        slight_MPR121::electrode_config_electrode_enable_ELE0_ELE11
    );
    electrode_config_proximity_enable_set(
        slight_MPR121::electrode_config_proximity_enable_off
    );
    electrode_config_baseline_tracking_set(
        slight_MPR121::electrode_config_baseline_tracking_init_wfirst_5hbits
    );
}




// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.13 Soft Rest Register
void slight_MPR121::soft_reset() {
    write_register(REG_SoftReset, 0x63);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.14 GPIO Registers

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// x.xx PWM Duty Control Registers
uint8_t slight_MPR121::gpio_pwm_get(uint8_t gpio) {
    // read register
    uint8_t reg = REG_GPIO_PWM_4_5;
    uint8_t reg_value = read_register(reg);
    uint8_t reg_mask = 0b00001111;
    // isolate bits
    reg_value = reg_value & reg_mask;
    // if second set shift to correct bits.
    if (reg_mask == 0b11110000) {
        reg_value = reg_value >> 4;
    }
    return reg_value;
}

void slight_MPR121::gpio_pwm_set(uint8_t gpio, uint8_t value) {
    // read register
    uint8_t reg = REG_GPIO_PWM_4_5;
    uint8_t reg_value = read_register(reg);
    uint8_t reg_mask = 0b00001111;
    // clear bits
    reg_value = reg_value & (~reg_mask);
    // set bits
    reg_value = reg_value | value;
    // write register
    write_register(reg, reg_value);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// END OF REGISTER FUNCTIONS
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Helpers / Allow Writes
void slight_MPR121::switch_mode_temporarily_to_allow_write() {
    // Serial.println(F("switch_mode_temporarily_to_allow_write"));
    switch_mode_backup_electrode_config = read_register(
        REG_Electrode_Configuration
    );
    write_register(
        REG_Electrode_Configuration,
        electrode_config_StopMode
    );
}

void slight_MPR121::switch_mode_restore() {
    // Serial.println(F("switch_mode_restore"));
    write_register(
        REG_Electrode_Configuration,
        switch_mode_backup_electrode_config
    );
}

uint8_t slight_MPR121::electrode_bounded(uint8_t electrode) {
    if (electrode > 12) {
        electrode = 12;
    }
    return electrode;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// advanced read write operations
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// alterantive to __builtin_popcount:
// found at https://en.wikipedia.org/wiki/Hamming_weight
// int popcount_4(uint64_t x) {
//     int count;
//     for (count=0; x; count++)
//         x &= x-1;
//     return count;
// }

uint8_t slight_MPR121::ones_in_mask_get(uint8_t mask) {
    uint8_t ones_in_mask = __builtin_popcount(mask);
    return ones_in_mask;
}

uint8_t slight_MPR121::value_max_get(uint8_t mask) {
    uint8_t ones_in_mask = ones_in_mask_get(mask);
    uint8_t value_max = 0b11111111 >> (8 - ones_in_mask);
    return value_max;
}

uint8_t slight_MPR121::value_max_get(uint8_t mask, uint8_t shift) {
    uint8_t value_max = mask >> shift;
    return value_max;
}

uint16_t slight_MPR121::value_max_get(uint16_t mask, uint16_t shift) {
    uint16_t value_max = mask >> shift;
    return value_max;
}

uint8_t slight_MPR121::value_limit(
    uint8_t mask,
    uint8_t shift,
    uint8_t value
) {
    uint8_t value_max = value_max_get(mask, shift);
    if (value > value_max) {
        value = value_max;
    }
    return value;
}



// public
uint8_t slight_MPR121::read_register_part(
    register_name_t reg_name,
    uint8_t reg_mask,
    uint8_t reg_shift
) {
    return read_register_part(
        (uint8_t)reg_name,
        reg_mask,
        reg_shift
    );
}

void slight_MPR121::write_register_part(
    register_name_t reg_name,
    uint8_t reg_mask,
    uint8_t reg_shift,
    uint8_t value,
    boolean switch_to_stopmode
) {
    write_register_part(
        (uint8_t)reg_name,
        reg_mask,
        reg_shift,
        value,
        switch_to_stopmode
    );
}

// private
uint8_t slight_MPR121::read_register_part(
    uint8_t reg_name,
    uint8_t reg_mask,
    uint8_t reg_shift
) {
    // read register
    uint8_t reg_value = read_register(reg_name);
    // isolate bits
    reg_value = reg_value & reg_mask;
    // shift to correct bits.
    reg_value = reg_value >> reg_shift;
    return reg_value;
}


void slight_MPR121::write_register_part(
    uint8_t reg_name,
    uint8_t reg_mask,
    uint8_t reg_shift,
    uint8_t value,
    boolean switch_to_stopmode
) {
    value = value_limit(reg_mask, reg_shift, value);
    // read register
    uint8_t reg_value = read_register(reg_name);
    // clear bits
    reg_value = reg_value & (~reg_mask);
    // set bits
    reg_value = reg_value | (value << reg_shift);
    // write register
    if (switch_to_stopmode) {
        switch_mode_temporarily_to_allow_write();
    }
    write_register(reg_name, reg_value);
    if (switch_to_stopmode) {
        switch_mode_restore();
    }
}







//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// basic read write operations
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t slight_MPR121::read_register(uint8_t reg_name) {
    uint8_t result_value = 0;
    if (ready) {
        twi_state = TWI_STATE_undefined;
        // set register
        Wire.beginTransmission(twi_address);
        Wire.write(reg_name);
        twi_state = (twi_state_t)Wire.endTransmission();
        if (twi_state == TWI_STATE_success) {
            // read data
            Wire.requestFrom(twi_address, (uint8_t)1);
            result_value = Wire.read();
        } else {
            twi_state_print(Serial, twi_state);
        }
    }
    return result_value;
}

uint8_t slight_MPR121::read_register(register_name_t reg_name) {
    return read_register((uint8_t)reg_name);
}

uint16_t slight_MPR121::read_register16bit(uint8_t reg_name) {
    uint16_t result_value = 0;
    if (ready) {
        twi_state = TWI_STATE_undefined;
        // set register
        Wire.beginTransmission(twi_address);
        Wire.write(reg_name);
        twi_state = (twi_state_t)Wire.endTransmission();
        if (twi_state == TWI_STATE_success) {
            // read data
            Wire.requestFrom(twi_address, (uint8_t)2);
            result_value = Wire.read();
            uint16_t highbyte = ((uint16_t)Wire.read()) << 8;
            result_value |= highbyte;
        } else {
            twi_state_print(Serial, twi_state);
        }
    }
    return result_value;
}

uint16_t slight_MPR121::read_register16bit(register_name_t reg_name) {
    return read_register16bit((uint8_t)reg_name);
}


void slight_MPR121::write_register(uint8_t reg_name, uint8_t value) {
    if (ready) {
        twi_state = TWI_STATE_undefined;
        // set register
        Wire.beginTransmission(twi_address);
        Wire.write(reg_name);
        Wire.write(value);
        twi_state = (twi_state_t)Wire.endTransmission();
        if (twi_state == TWI_STATE_success) {
            // all fine.
        } else {
            twi_state_print(Serial, twi_state);
        }
    }
}

void slight_MPR121::write_register(
    register_name_t reg_name,
    uint8_t value
) {
    write_register((uint8_t)reg_name, value);
}



void slight_MPR121::twi_state_print(Print &out) {
    twi_state_print(out, twi_state);
}

void slight_MPR121::twi_state_print(Print &out, twi_state_t state) {
    switch (state) {
        case TWI_STATE_success: {
            out.print(F("success"));
        } break;
        case TWI_STATE_data_to_long: {
            out.print(F("data too long to fit in transmit buffer"));
        } break;
        case TWI_STATE_rec_NACK_on_address: {
            out.print(F("received NACK on transmit of address"));
        } break;
        case TWI_STATE_rec_NACK_on_data: {
            out.print(F("received NACK on transmit of data"));
        } break;
        case TWI_STATE_other_error: {
            out.print(F("other error"));
        } break;
        default: {
            out.print(F("??"));
        }
    }
}

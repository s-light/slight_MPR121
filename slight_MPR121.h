/******************************************************************************

    arduino library for MPR121 touch sensor.
    tested with adafruit MPR121 breakoutboard
        (https://www.adafruit.com/product/1982)

    basic library api is compatible to https://github.com/s-light/slight_CAP1188

    written by stefan krueger (s-light),
        git@s-light.eu, http://s-light.eu, https://github.com/s-light/

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



#ifndef slight_MPR121_H_
#define slight_MPR121_H_

/** Includes Core Arduino functionality **/
#if ARDUINO
    #if ARDUINO < 100
        #include <WProgram.h>
    #else
        #include <Arduino.h>
    #endif
#endif

#include <Arduino.h>


class slight_MPR121 {
public:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // constructor

    slight_MPR121(uint8_t twi_address);
    slight_MPR121(uint8_t twi_address, uint8_t interrupt_pin);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public types

    static const uint8_t NO_PIN = 255;

    enum twi_state_t {
        TWI_STATE_success = 0,
        TWI_STATE_data_to_long = 1,
        TWI_STATE_rec_NACK_on_address = 2,
        TWI_STATE_rec_NACK_on_data = 3,
        TWI_STATE_other_error = 4,
        TWI_STATE_undefined = 99,
    };

    // registers
    enum register_name_t {
        // 5.2 Touch Status Registers (0x00~0x01)
        // read only
        // Touch Status0
        // B0 Electrode0
        // ..
        // B7 Electrode7
        // Touch Status1
        // B0 Electrode8
        // ..
        // B3 Electrode11
        // B4 Electrode PROX
        // B7 OVCF Over Current Flag (read & write)
        REG_Touch_Status0 = 0x00,
        REG_Touch_Status1 = 0x01,
        // 5.12 Out-Of-Range Status Registers (0x02, 0x03)
        // read only
        // OutOfRange Status0
        // B0 Electrode0
        // ..
        // B7 Electrode7
        // OutOfRange Status0
        // B0 Electrode8
        // ..
        // B3 Electrode11
        // B4 Electrode PROX
        // B6 ARFF Auto-Reconfiguration Fail Flag
        // B7 ACFF Auto-Configuration Fail Flag
        REG_OutOfRange_Status0 = 0x02,
        REG_OutOfRange_Status1 = 0x03,
        // 5.3 Electrode Filtered Data Register 0x04~0x1D)
        // read only
        // 10bit value L=Bit0..Bit7 H=Bit8..Bit9
        REG_Electrode_0_Filterd_Data_L = 0x04,
        REG_Electrode_0_Filterd_Data_H = 0x05,
        REG_Electrode_1_Filterd_Data_L = 0x06,
        REG_Electrode_1_Filterd_Data_H = 0x07,
        REG_Electrode_2_Filterd_Data_L = 0x08,
        REG_Electrode_2_Filterd_Data_H = 0x09,
        REG_Electrode_3_Filterd_Data_L = 0x0A,
        REG_Electrode_3_Filterd_Data_H = 0x0B,
        REG_Electrode_4_Filterd_Data_L = 0x0C,
        REG_Electrode_4_Filterd_Data_H = 0x0D,
        REG_Electrode_5_Filterd_Data_L = 0x0E,
        REG_Electrode_5_Filterd_Data_H = 0x0F,
        REG_Electrode_6_Filterd_Data_L = 0x10,
        REG_Electrode_6_Filterd_Data_H = 0x11,
        REG_Electrode_7_Filterd_Data_L = 0x12,
        REG_Electrode_7_Filterd_Data_H = 0x13,
        REG_Electrode_8_Filterd_Data_L = 0x14,
        REG_Electrode_8_Filterd_Data_H = 0x15,
        REG_Electrode_9_Filterd_Data_L = 0x16,
        REG_Electrode_9_Filterd_Data_H = 0x17,
        REG_Electrode_10_Filterd_Data_L = 0x18,
        REG_Electrode_10_Filterd_Data_H = 0x19,
        REG_Electrode_11_Filterd_Data_L = 0x1A,
        REG_Electrode_11_Filterd_Data_H = 0x1B,
        REG_Electrode_PROX_Filterd_Data_L = 0x1C,
        REG_Electrode_PROX_Filterd_Data_H = 0x1D,
        // 5.4 Baseline Value Register (0x1E~0x2A)
        // read / write(in Stop Mode)
        // internally 10bit value. 8MSB readablle.
        // to comapre with Filterd_Data shift left 2 (<<2)
        REG_Electrode_0_Baseline_Value = 0x1E,
        REG_Electrode_1_Baseline_Value = 0x1F,
        REG_Electrode_2_Baseline_Value = 0x20,
        REG_Electrode_3_Baseline_Value = 0x21,
        REG_Electrode_4_Baseline_Value = 0x22,
        REG_Electrode_5_Baseline_Value = 0x23,
        REG_Electrode_6_Baseline_Value = 0x24,
        REG_Electrode_7_Baseline_Value = 0x25,
        REG_Electrode_8_Baseline_Value = 0x26,
        REG_Electrode_9_Baseline_Value = 0x27,
        REG_Electrode_10_Baseline_Value = 0x28,
        REG_Electrode_11_Baseline_Value = 0x29,
        REG_Electrode_PROX_Baseline_Value = 0x2A,
        // 5.5 Baseline Filtering Control Register (0x2B~0x40)
        // Read/Write
        // Rising
        // 6bit MHD= Maximum Half Delta
        REG_Baseline_Filtering_Maximum_Half_Delta_Rising = 0x2B,
        // 6bit NHD= Noise Half Delta
        REG_Baseline_Filtering_Noise_Half_Delta_Rising = 0x2C,
        // 8bit NCL= Noise Count Limit
        REG_Baseline_Filtering_Noise_Count_Limit_Rising = 0x2D,
        // 8bit FDL= Filter Delay Count Limit
        REG_Baseline_Filtering_Filter_Delay_Count_Limit_Rising = 0x2E,
        // Falling
        // 6bit MHD= Maximum Half Delta
        REG_Baseline_Filtering_Maximum_Half_Delta_Falling = 0x2F,
        // 6bit NHD= Noise Half Delta
        REG_Baseline_Filtering_Noise_Half_Delta_Falling = 0x30,
        // 8bit NCL= Noise Count Limit
        REG_Baseline_Filtering_Noise_Count_Limit_Falling = 0x31,
        // 8bit FDL= Filter Delay Count Limit
        REG_Baseline_Filtering_Filter_Delay_Count_Limit_Falling = 0x32,
        // Touched
        // 6bit NHD= Noise Half Delta
        REG_Baseline_Filtering_Noise_Half_Delta_Amount_Touched = 0x33,
        // 8bit NCL= Noise Count Limit
        REG_Baseline_Filtering_Noise_Count_Limit_Touched = 0x34,
        // 8bit FDL= Filter Delay Count Limit
        REG_Baseline_Filtering_Filter_Delay_Count_Limit_Touched = 0x35,
        // Electrode PROX
        // Rising
        // 6bit MHD= Maximum Half Delta
        REG_Baseline_Filtering_PROX_Maximum_Half_Delta_Rising = 0x36,
        // 6bit NHD= Noise Half Delta
        REG_Baseline_Filtering_PROX_Noise_Half_Delta_Rising = 0x37,
        // 8bit NCL= Noise Count Limit
        REG_Baseline_Filtering_PROX_Noise_Count_Limit_Rising = 0x38,
        // 8bit FDL= Filter Delay Count Limit
        REG_Baseline_Filtering_PROX_Filter_Delay_Count_Limit_Rising = 0x39,
        // Falling
        // 6bit MHD= Maximum Half Delta
        REG_Baseline_Filtering_PROX_Maximum_Half_Delta_Falling = 0x3A,
        // 6bit NHD= Noise Half Delta
        REG_Baseline_Filtering_PROX_Noise_Half_Delta_Falling = 0x3B,
        // 8bit NCL= Noise Count Limit
        REG_Baseline_Filtering_PROX_Noise_Count_Limit_Falling = 0x3C,
        // 8bit FDL= Filter Delay Count Limit
        REG_Baseline_Filtering_PROX_Filter_Delay_Count_Limit_Falling = 0x3D,
        // Touched
        // 6bit NHD= Noise Half Delta
        REG_Baseline_Filtering_PROX_Noise_Half_Delta_Amount_Touched = 0x3E,
        // 8bit NCL= Noise Count Limit
        REG_Baseline_Filtering_PROX_Noise_Count_Limit_Touched = 0x3F,
        // 8bit FDL= Filter Delay Count Limit
        REG_Baseline_Filtering_PROX_Filter_Delay_Count_Limit_Touched = 0x40,
        // 5.6 Touch / Release Threshold (0x41~0x5A)
        // Read/Write
        // more info on this values in Application Note
        // AN3892 - MPR121 Jitter and False Touch Detection
        // http://cache.nxp.com/files/sensors/doc/app_note/AN3892.pdf
        REG_Electrode_0_Touch_Threshold = 0x41,
        REG_Electrode_0_Release_Threshold = 0x42,
        REG_Electrode_1_Touch_Threshold = 0x43,
        REG_Electrode_1_Release_Threshold = 0x44,
        REG_Electrode_2_Touch_Threshold = 0x45,
        REG_Electrode_2_Release_Threshold = 0x46,
        REG_Electrode_3_Touch_Threshold = 0x47,
        REG_Electrode_3_Release_Threshold = 0x48,
        REG_Electrode_4_Touch_Threshold = 0x49,
        REG_Electrode_4_Release_Threshold = 0x4A,
        REG_Electrode_5_Touch_Threshold = 0x4B,
        REG_Electrode_5_Release_Threshold = 0x4C,
        REG_Electrode_6_Touch_Threshold = 0x4D,
        REG_Electrode_6_Release_Threshold = 0x4E,
        REG_Electrode_7_Touch_Threshold = 0x4F,
        REG_Electrode_7_Release_Threshold = 0x50,
        REG_Electrode_8_Touch_Threshold = 0x51,
        REG_Electrode_8_Release_Threshold = 0x52,
        REG_Electrode_9_Touch_Threshold = 0x53,
        REG_Electrode_9_Release_Threshold = 0x54,
        REG_Electrode_10_Touch_Threshold = 0x55,
        REG_Electrode_10_Release_Threshold = 0x56,
        REG_Electrode_11_Touch_Threshold = 0x57,
        REG_Electrode_11_Release_Threshold = 0x58,
        REG_Electrode_PROX_Touch_Threshold = 0x59,
        REG_Electrode_PROX_Release_Threshold = 0x5A,
        // 5.7 Debounce Register (0x5B)
        // Read/Write
        // B0..B2 Debounce number for touch 0..7
        // B4..B6 Debounce number for release 0..7
        REG_Debounce = 0x5B,
        // 5.8 Filter and Global CDC CDT Configuration (0x5C, 0x5D)
        // in AN3889 called 'Analog Front End - AFE CONFIGURATION REGISTER'
        // Read/Write
        // Config0:
        // B0..B5 CDC Charge Discharge Current
        // B6..B7 FFI First Filter Iterations
        // Config1:
        // B0..B2 ESI Electrode Sample Interval
        // B3..B4 SFI Second Filter Iterations
        // B5..B7 CDT Charge Discharge Time
        REG_Global_Config0 = 0x5C,
        REG_Global_Config1 = 0x5D,
        // 5.11 Electrode Configuration Register (ECR, 0x5E)
        // Read/Write
        // B0..B4 ELE_EN Electrode Enable
        // B5..B6 ELEPROX_EN Proximity Enable
        // B7..B8 CL Calibration Lock
        REG_Electrode_Configuration = 0x5E,
        // 5.9 Electrode Charge Current Register (0x5F~0x6B)
        // Read/Write
        // 6bit
        // B0..B5 CDCx Charge Current 0..63uA in 1uA steps; 0=Global value
        REG_Electrode_0_Current = 0x5F,
        REG_Electrode_1_Current = 0x60,
        REG_Electrode_2_Current = 0x61,
        REG_Electrode_3_Current = 0x62,
        REG_Electrode_4_Current = 0x63,
        REG_Electrode_5_Current = 0x64,
        REG_Electrode_6_Current = 0x65,
        REG_Electrode_7_Current = 0x66,
        REG_Electrode_8_Current = 0x67,
        REG_Electrode_9_Current = 0x68,
        REG_Electrode_10_Current = 0x69,
        REG_Electrode_11_Current = 0x6A,
        REG_Electrode_PROX_Current = 0x6B,
        // 5.10 Electrode Charge Time Register (0x6C~0x72)
        // Read/Write
        // 3bit
        // B0..B2 CDTx Charge Time 0..32uS in 1uS steps; 0=Global value
        // B4..B6 CDTx +1 Charge Time 0..32uS in 1uS steps; 0=Global value
        REG_Electrode_0_1_Time = 0x6C,
        REG_Electrode_2_3_Time = 0x6D,
        REG_Electrode_4_5_Time = 0x6E,
        REG_Electrode_6_7_Time = 0x6F,
        REG_Electrode_8_9_Time = 0x70,
        REG_Electrode_10_11_Time = 0x71,
        REG_Electrode_PROX_Time = 0x72,
        // 5.14 GPIO Registers (0x73~0x7A)
        // Details see AN3894
        // ECR has priority for configuration
        // EN
        //  0 GPIO disabled Port is high-Z
        //  1 GPIO enabled
        // DIR=0 (Input)
        // CTRL0:1  Function
        // 00       input port.
        // 10       input port with internal pulldown.
        // 11       input port with internal pullup.
        // 01       Not defined yet (as same as CTL = 00).
        // DIR=1 (Output)
        // CTRL0:1  Function
        // 00       CMOS output port.
        // 11       high side only open drain output port for LED driver.
        // 10       low side only open drain output port.
        // 01       Not defined yet (as same as CTL = 00).
        // Read/Write
        // B0 - Electrode4
        // ..
        // B7 - Electrode11
        REG_GPIO_Control0 = 0x73,
        REG_GPIO_Control1 = 0x74,
        REG_GPIO_Data = 0x75,
        REG_GPIO_Direction = 0x76,
        REG_GPIO_Enable = 0x77,
        REG_GPIO_Data_Set = 0x78,
        REG_GPIO_Data_Clear = 0x78,
        REG_GPIO_Data_Toggle = 0x7A,
        // 5.11 Auto-Configuration Registers (0x7B~0x7F)
        // see AN3889 for configuration values
        // Read/Write
        // Control0
        // B0 ACE Auto-Configuration Enable
        // B1 ARE Auto-Reconfiguration Enable
        // B2..B3 BVA fill with same as ECR CL
        // B4..B5 RETRY 00 No retry; 01 2*retry; 10 4*retry; 11 8*retry
        // B5..B7 FFI fill with same as FFI in 0x5C
        // Control1
        // B0 ACFIE Auto-Configuration Fail Interrupt Enable
        // B1 ARFIE Auto-Reconfiguration Fail Interrupt Enable
        // B2 OORIE Out Of range Interrupt Enable
        // B7 SCTS Skip Charge Time Search
        REG_AutoConfiguration_Control0 = 0x7B,
        REG_AutoConfiguration_Control1 = 0x7C,
        // USL= Upper-Side Limit
        // to comapre with Filterd_Data shift left 2 (<<2)
        REG_AutoConfiguration_Up_Side_Limit = 0x7D,
        // LSL= Lower-Side Limit
        // to comapre with Filterd_Data shift left 2 (<<2)
        REG_AutoConfiguration_Low_Side_Limit = 0x7E,
        // TL= Target Level
        // to comapre with Filterd_Data shift left 2 (<<2)
        REG_AutoConfiguration_Target_Level = 0x7F,
        // 5.13 Soft Rest Register (0x80)
        // write 0x63 for Soft Reset.
        REG_SoftReset = 0x80,
        // x.xx PWM Duty Control Registers
        // details AN3894
        // Read/Write
        // PWM Period: 8ms (1/256 of 32 KHz OSC)
        // Electrode Measurement intterrupts pwm.
        // PWM Value:
        // 0 = PWM is off, stable high when DAT register is “1”
        // 1..15 = PWM
        // B0..B3 - ElectrodeN
        // B4..B7 - ElectrodeN+1
        REG_GPIO_PWM_4_5 = 0x81,
        REG_GPIO_PWM_6_7 = 0x82,
        REG_GPIO_PWM_8_9 = 0x83,
        REG_GPIO_PWM_10_11 = 0x84,
    };

    // callback
    typedef void (* callback_t) (slight_MPR121 *instance);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // basic library api
    bool begin();
    void update();

    // sensor configurations
    void configuration_load_defaults();

    // poll-update scheduler / IRQ handling
    void update_interval_set_autofit();
    void update_interval_set(uint32_t interval);
    uint32_t update_interval_get();

    void touch_event_set_callback(callback_t);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // register helper functions

    // electrode helper
    // enum electrode_name_t {
    //     electrode_0 = B0000000000000000,
    // };
    // uint8_t electrode_restrictrange(uint8_t electrode);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // gloal helpers

    // Charge Discharge Time CDT
    // 0.5us - 32us
    // bits value   time
    // 000   0       invalid / global
    // 001   1       0.5us
    // 010   2       1us
    // 011   3       2us
    // 100   4       4us
    // 101   5       8us
    // 110   6       16us
    // 111   7       32us
    enum charge_discharge_time_t {
        cdt_global = B00000000,
        cdt_05us =   B00000001,
        cdt_1us =    B00000010,
        cdt_2us =    B00000011,
        cdt_4us =    B00000100,
        cdt_8us =    B00000101,
        cdt_16us =   B00000110,
        cdt_32us =   B00000111,
    };
    static charge_discharge_time_t charge_discharge_time_convert(uint8_t);
    static void charge_discharge_time_print(
        Print &out,
        charge_discharge_time_t value
    );

    // First Filter Iterations FFI
    // bits value   number of samples
    // 00   0       6
    // 01   1       10
    // 10   2       18
    // 11   3       34
    enum first_filter_iterations_t {
        ffi_6 =  B00000000,
        ffi_10 = B00000001,
        ffi_18 = B00000010,
        ffi_34 = B00000011,
    };
    static first_filter_iterations_t first_filter_iterations_convert(uint8_t);
    static void first_filter_iterations_print(
        Print &out,
        first_filter_iterations_t value
    );


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // individual registers:

    // 5.2 Touch Status Registers
    static const uint16_t touch_status_electrode_mask = 0b0001111111111111;
    static const uint16_t touch_status_overcurrent_flag_mask = 0b1000000000000000;

    uint16_t touch_status_read();
    uint16_t touch_status_get();
    bool touch_status_get(uint8_t electrode);
    bool touch_status_get_overcurrent_flag();

    // 5.12 Out-Of-Range Status Registers
    static const uint16_t outofrange_electrode_mask = 0b0001111111111111;
    static const uint16_t outofrange_ARFF_mask = 0b0100000000000000;
    static const uint16_t outofrange_ACFF_mask = 0b1000000000000000;

    uint16_t outofrange_status_get_raw();
    uint16_t outofrange_status_get();
    bool outofrange_status_get(uint8_t electrode);
    bool outofrange_status_get_autoreconfiguration_fail_flag();
    bool outofrange_status_get_autoconfiguration_fail_flag();

    // 5.3 Electrode Filtered Data Register
    uint16_t electrode_filterd_data_get(uint8_t electrode);

    // 5.4 Baseline Value Register
    uint8_t electrode_baseline_value_get_raw(uint8_t electrode);
    uint16_t electrode_baseline_value_get(uint8_t electrode);
    void electrode_baseline_value_set_raw(uint8_t electrode, uint8_t value);
    void electrode_baseline_value_set(uint8_t electrode, uint16_t value);

    // 5.5 Baseline Filtering Control Register

    // 5.6 Touch / Release Threshold
    uint8_t electrode_touch_threshold_get(uint8_t electrode);
    void electrode_touch_threshold_set(uint8_t electrode, uint8_t value);

    uint8_t electrode_release_threshold_get(uint8_t electrode);
    void electrode_release_threshold_set(uint8_t electrode, uint8_t value);

    // 5.7 Debounce Register
    static const uint8_t electrode_debounce_touch_mask = 0b00000111;
    static const uint8_t electrode_debounce_touch_shift = 0;
    static const uint8_t electrode_debounce_release_mask = 0b01110000;
    static const uint8_t electrode_debounce_release_shift = 4;

    uint8_t electrode_debounce_touch_get();
    void electrode_debounce_touch_set(uint8_t value);
    uint8_t electrode_debounce_release_get();
    void electrode_debounce_release_set(uint8_t value);

    // 5.8 Filter and Global CDC CDT Configuration
    // REG_Global_Config0
    static const uint8_t global_config_charge_discharge_current_mask = 0b00111111;
    static const uint8_t global_config_charge_discharge_current_shift = 0;
    static const uint8_t global_config_first_filter_iterations_mask = 0b11000000;
    static const uint8_t global_config_first_filter_iterations_shift = 6;

    // 0=disables charging, 1..63uA charge current
    uint8_t global_config_charge_discharge_current_get();
    void global_config_charge_discharge_current_set(uint8_t value);


    first_filter_iterations_t global_config_first_filter_iterations_get();
    void global_config_first_filter_iterations_set(first_filter_iterations_t value);
    void global_config_first_filter_iterations_set(uint8_t value);
    void global_config_first_filter_iterations_print(Print &out);

    // REG_Global_Config1
    static const uint8_t global_config_electrode_sample_interval_mask = 0b00000111;
    static const uint8_t global_config_electrode_sample_interval_shift = 0;
    static const uint8_t global_config_second_filter_iterations_mask = 0b00011000;
    static const uint8_t global_config_second_filter_iterations_shift = 3;
    static const uint8_t global_config_charge_discharge_time_mask = 0b11100000;
    static const uint8_t global_config_charge_discharge_time_shift = 5;


    // Electrode Sample Interval ESI
    // 1ms - 128ms
    // bits value   time
    // 000   0       1ms
    // 001   1       2ms
    // 010   2       4ms
    // 011   3       8ms
    // 100   4       16ms
    // 101   5       32ms
    // 110   6       64ms
    // 111   7       128ms
    enum global_config_esi_t {
        esi_1ms =   B00000000,
        esi_2ms =   B00000001,
        esi_4ms =   B00000010,
        esi_8ms =   B00000011,
        esi_16ms =  B00000100,
        esi_32ms =  B00000101,
        esi_64ms =  B00000110,
        esi_128ms = B00000111,
    };
    global_config_esi_t global_config_electrode_sample_interval_get();
    void global_config_electrode_sample_interval_set(global_config_esi_t value);
    void global_config_electrode_sample_interval_set(uint8_t value);
    static global_config_esi_t global_config_electrode_sample_interval_convert(uint8_t);
    static void global_config_electrode_sample_interval_print(
        Print &out,
        global_config_esi_t value
    );
    void global_config_electrode_sample_interval_print(Print &out);

    // Second Filter Iterations SFI
    // bits value   number of samples
    // 00   0       4
    // 01   1       6
    // 10   2       10
    // 11   3       18
    enum global_config_sfi_t {
        sfi_4 =  B00000000,
        sfi_6 =  B00000001,
        sfi_10 = B00000010,
        sfi_18 = B00000011,
    };
    global_config_sfi_t global_config_second_filter_iterations_get();
    void global_config_second_filter_iterations_set(global_config_sfi_t value);
    void global_config_second_filter_iterations_set(uint8_t value);
    static global_config_sfi_t global_config_second_filter_iterations_convert(uint8_t);
    static void global_config_second_filter_iterations_print(
        Print &out,
        global_config_sfi_t value
    );
    void global_config_second_filter_iterations_print(Print &out);

    charge_discharge_time_t global_config_charge_discharge_time_get();
    void global_config_charge_discharge_time_set(charge_discharge_time_t value);
    void global_config_charge_discharge_time_set(uint8_t value);
    void global_config_charge_discharge_time_print(Print &out);

    // 5.9 Electrode Charge Current Register
    static const uint8_t electrode_charge_current_mask = 0b00111111;
    static const uint8_t electrode_charge_current_shift = 0;

    uint8_t electrode_charge_current_get(uint8_t electrode);
    void electrode_charge_current_set(uint8_t electrode, uint8_t value);

    // 5.10 Electrode Charge Time Register
    static const uint8_t electrode_charge_time_even_mask = 0b00001111;
    static const uint8_t electrode_charge_time_even_shift = 0;
    static const uint8_t electrode_charge_time_odd_mask = 0b11110000;
    static const uint8_t electrode_charge_time_odd_shift = 4;

    uint8_t electrode_charge_time_get(uint8_t electrode);
    void electrode_charge_time_set(uint8_t electrode, uint8_t value);

    // 5.11 Electrode Configuration Register - ECR
    static const uint8_t electrode_config_StopMode = 0b00000000;

    static const uint8_t electrode_config_electrode_enable_mask = 0b00001111;
    static const uint8_t electrode_config_electrode_enable_shift = 0;
    static const uint8_t electrode_config_proximity_enable_mask = 0b00110000;
    static const uint8_t electrode_config_proximity_enable_shift = 4;
    static const uint8_t electrode_config_baseline_tracking_mask = 0b11000000;
    static const uint8_t electrode_config_baseline_tracking_shift = 6;

    // whole register
    void electrode_config_set(uint8_t value);
    uint8_t electrode_config_get();

    // Electrode Enable – ELE_EN
    // 0000 – StopMode all disabled (default)
    // 0001 – RunMode Electrode0 enabled
    // 0010 – RunMode Electrode0 - Electrode1 enabled
    // 0011 – RunMode Electrode0 - Electrode2 enabled
    // 0100 – RunMode Electrode0 - Electrode3 enabled
    // 0101 – RunMode Electrode0 - Electrode4 enabled
    // 0110 – RunMode Electrode0 - Electrode5 enabled
    // 0111 – RunMode Electrode0 - Electrode6 enabled
    // 1000 – RunMode Electrode0 - Electrode7 enabled
    // 1001 – RunMode Electrode0 - Electrode8 enabled
    // 1010 – RunMode Electrode0 - Electrode9 enabled
    // 1011 – RunMode Electrode0 - Electrode10 enabled
    // 11xx – RunMode Electrode0 - Electrode11 enabled
    enum electrode_config_electrode_enable_t {
        electrode_config_electrode_enable_off = 0b00000000,
        electrode_config_electrode_enable_ELE0 = 0b00000001,
        electrode_config_electrode_enable_ELE0_ELE1 = 0b00000010,
        electrode_config_electrode_enable_ELE0_ELE2 = 0b00000011,
        electrode_config_electrode_enable_ELE0_ELE3 = 0b00000100,
        electrode_config_electrode_enable_ELE0_ELE4 = 0b00000101,
        electrode_config_electrode_enable_ELE0_ELE5 = 0b00000110,
        electrode_config_electrode_enable_ELE0_ELE6 = 0b00000111,
        electrode_config_electrode_enable_ELE0_ELE7 = 0b00001000,
        electrode_config_electrode_enable_ELE0_ELE8 = 0b00001001,
        electrode_config_electrode_enable_ELE0_ELE9 = 0b00001010,
        electrode_config_electrode_enable_ELE0_ELE10 = 0b00001011,
        electrode_config_electrode_enable_ELE0_ELE11 = 0b00001100,
    };
    // uint8_t electrode_config_electrode_enable_get_raw();
    electrode_config_electrode_enable_t electrode_config_electrode_enable_get();
    void electrode_config_electrode_enable_set(electrode_config_electrode_enable_t value);
    void electrode_config_electrode_enable_set(uint8_t value);
    static electrode_config_electrode_enable_t electrode_config_electrode_enable_convert(uint8_t);
    static void electrode_config_electrode_enable_print(
        Print &out,
        electrode_config_electrode_enable_t value
    );
    void electrode_config_electrode_enable_print(Print &out);

    // Proximity Enable – ELEPROX_EN
    // 0000 – StopMode Proximity disabled (default)
    // 0010 – RunMode Electrode0 - Electrode1 for Proximity enabled
    // 0011 – RunMode Electrode0 - Electrode3 for Proximity enabled
    // 0100 – RunMode Electrode0 - Electrode11 for Proximity enabled
    enum electrode_config_proximity_enable_t {
        electrode_config_proximity_enable_off = 0b00000000,
        electrode_config_proximity_enable_ELE0_ELE1 = 0b00000001,
        electrode_config_proximity_enable_ELE0_ELE3 = 0b00000010,
        electrode_config_proximity_enable_ELE0_ELE11 = 0b00000011,
    };
    // uint8_t electrode_config_proximity_enable_get_raw();
    electrode_config_proximity_enable_t electrode_config_proximity_enable_get();
    void electrode_config_proximity_enable_set(electrode_config_proximity_enable_t value);
    void electrode_config_proximity_enable_set(uint8_t value);
    static electrode_config_proximity_enable_t electrode_config_proximity_enable_convert(uint8_t);
    static void electrode_config_proximity_enable_print(
        Print &out,
        electrode_config_proximity_enable_t value
    );
    void electrode_config_proximity_enable_print(Print &out);

    // Calibration Lock - LC
    // also called Baseline Tracking
    // 00 – baseline tracking enabled - value not changed on init (default)
    // 01 – baseline tracking disabled
    // 10 – baseline tracking enabled -
    // 11 – baseline tracking enabled -
    enum electrode_config_baseline_tracking_t {
        electrode_config_baseline_tracking_no_change = 0b00000000,
        electrode_config_baseline_tracking_disabled = 0b00000001,
        electrode_config_baseline_tracking_init_wfirst_5hbits = 0b00000010,
        electrode_config_baseline_tracking_init_wfirst_10bits = 0b00000011,
    };
    // uint8_t electrode_config_baseline_tracking_get_raw();
    electrode_config_baseline_tracking_t electrode_config_baseline_tracking_get();
    void electrode_config_baseline_tracking_set(electrode_config_baseline_tracking_t value);
    void electrode_config_baseline_tracking_set(uint8_t value);
    static electrode_config_baseline_tracking_t electrode_config_baseline_tracking_convert(uint8_t);
    static void electrode_config_baseline_tracking_print(
        Print &out,
        electrode_config_baseline_tracking_t value
    );
    void electrode_config_baseline_tracking_print(Print &out);



    // 5.11 Auto-Configuration Registers

    // REG_AutoConfiguration_Control0
    static const uint8_t auto_config_enable_mask = 0b00000001;
    static const uint8_t auto_config_enable_shift = 0;
    static const uint8_t auto_reconfig_enable_mask = 0b00000010;
    static const uint8_t auto_reconfig_enable_shift = 1;
    static const uint8_t auto_config_bva_mask = 0b00001100;
    static const uint8_t auto_config_bva_shift = 2;
    static const uint8_t auto_config_retry_mask = 0b00110000;
    static const uint8_t auto_config_retry_shift = 4;
    static const uint8_t auto_config_first_filter_iteration_mask = 0b11000000;
    static const uint8_t auto_config_first_filter_iteration_shift = 6;

    bool auto_config_enable_get();
    void auto_config_enable_set(bool value);

    bool auto_reconfig_enable_get();
    void auto_reconfig_enable_set(bool value);


    // Baseline Value Adjust (BVA) –
    // The baseline value adjust determines the initial value of
    // the baseline registers after auto-configuration completes.
    // 00 – Baseline is not changed
    // 01 – Baseline is cleared
    // 10 – Baseline is set to the AUTO-CONFIG baseline with the lower 3 bits cleared
    // 11 – Baseline is set to the AUTO-CONFIG baseline
    enum auto_config_bva_t {
        baseline_not_changed = B00000000,
        baseline_cleared = B00000001,
        baseline_autoconfig_lower3bits_cleared = B00000010,
        baseline_autoconfig = B00000011,
    };
    // uint8_t auto_config_bva_get_raw();
    auto_config_bva_t auto_config_bva_get();
    void auto_config_bva_set(auto_config_bva_t value);
    void auto_config_bva_set(uint8_t value);
    static auto_config_bva_t auto_config_bva_convert(uint8_t);
    static void auto_config_bva_print(
        Print &out,
        auto_config_bva_t value
    );
    void auto_config_bva_print(Print &out);

    enum auto_config_retry_t {
        retry_0 = B00000000,  // default
        retry_2 = B00000001,
        retry_4 = B00000010,
        retry_8 = B00000011,
    };
    auto_config_retry_t auto_config_retry_get();
    // uint8_t auto_config_retry_get();
    void auto_config_retry_set(auto_config_retry_t value);
    void auto_config_retry_set(uint8_t value);
    static auto_config_retry_t auto_config_retry_convert(uint8_t value);
    static void auto_config_retry_print(Print& out, auto_config_retry_t value);
    void auto_config_retry_print(Print& out);

    uint8_t auto_config_first_filter_iteration_get();
    void auto_config_first_filter_iteration_set(uint8_t value);

    // REG_AutoConfiguration_Control1
    static const uint8_t auto_config_fail_interrupt_enable_mask = 0b00000001;
    static const uint8_t auto_config_fail_interrupt_enable_shift = 0;
    static const uint8_t auto_reconfig_fail_interrupt_enable_mask = 0b00000010;
    static const uint8_t auto_reconfig_fail_interrupt_enable_shift = 1;
    static const uint8_t auto_config_out_of_range_interrupt_enable_mask = 0b00000100;
    static const uint8_t auto_config_out_of_range_interrupt_enable_shift = 2;
    static const uint8_t auto_config_skip_charge_time_search_mask = 0b10000000;
    static const uint8_t auto_config_skip_charge_time_search_shift = 7;

    bool auto_config_fail_interrupt_enable_get();
    void auto_config_fail_interrupt_enable_set(bool value);

    bool auto_reconfig_fail_interrupt_enable_get();
    void auto_reconfig_fail_interrupt_enable_set(bool value);

    bool auto_config_out_of_range_interrupt_enable_get();
    void auto_config_out_of_range_interrupt_enable_set(bool value);

    bool auto_config_skip_charge_time_search_get();
    void auto_config_skip_charge_time_search_set(bool value);

    // REG_AutoConfiguration_Up_Side_Limit
    uint8_t auto_config_up_side_limit_get_raw();
    uint16_t auto_config_up_side_limit_get();
    void auto_config_up_side_limit_set_raw(uint8_t value);
    void auto_config_up_side_limit_set(uint16_t value);

    // REG_AutoConfiguration_Low_Side_Limit
    uint8_t auto_config_low_side_limit_get_raw();
    uint16_t auto_config_low_side_limit_get();
    void auto_config_low_side_limit_set_raw(uint8_t value);
    void auto_config_low_side_limit_set(uint16_t value);

    // REG_AutoConfiguration_Target_Level
    uint8_t auto_config_target_level_get_raw();
    uint16_t auto_config_target_level_get();
    void auto_config_target_level_set_raw(uint8_t value);
    void auto_config_target_level_set(uint16_t value);

    // AutoConfiguration automatic calculation
    void auto_config_calculate_values_and_set(uint8_t value_vcc = 33);
    // void auto_config_load_recommend_config();
    // void auto_config_load_recommend_config(Print &out);
    void auto_config_load_recommend_config(Print *out = NULL);

    // 5.13 Soft Rest Register
    void soft_reset();

    // 5.14 GPIO Registers

    // x.xx PWM Duty Control Registers
    void gpio_pwm_set(uint8_t gpio, uint8_t value);
    uint8_t gpio_pwm_get(uint8_t gpio);



    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // advanced read write operations
    uint8_t read_register_part(
        register_name_t reg_name,
        uint8_t reg_mask,
        uint8_t reg_shift
    );

    void write_register_part(
        register_name_t reg_name,
        uint8_t reg_mask,
        uint8_t reg_shift,
        uint8_t value,
        boolean switch_to_stopmode = true
    );

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // basic read write operations
    void write_register(register_name_t reg_name, uint8_t value);
    uint8_t read_register(register_name_t reg_name);
    uint16_t read_register16bit(register_name_t reg_name);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // twi state helper
    twi_state_t twi_state_get();
    void twi_state_print(Print &out);
    static void twi_state_print(Print &out, twi_state_t state);

private:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // private functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void touch_status_update();

    void touch_event_callback();

    // write helper
    void switch_mode_temporarily_to_allow_write();
    void switch_mode_restore();
    uint8_t switch_mode_backup_electrode_config;

    // electrode helper
    uint8_t electrode_bounded(uint8_t electrode);

    // advanced read write operations / helper
    uint8_t ones_in_mask_get(uint8_t mask);
    uint8_t value_max_get(uint8_t mask);
    uint8_t value_max_get(uint8_t mask, uint8_t shift);
    uint16_t value_max_get(uint16_t mask, uint16_t shift);
    uint8_t value_limit(uint8_t mask, uint8_t shift, uint8_t);

    uint8_t read_register_part(
        uint8_t reg_name,
        uint8_t reg_mask,
        uint8_t reg_shift
    );

    void write_register_part(
        uint8_t reg_name,
        uint8_t reg_mask,
        uint8_t reg_shift,
        uint8_t value,
        boolean switch_to_stopmode = true
    );

    // basic read write operations
    void write_register(uint8_t reg_name, uint8_t value);
    uint8_t read_register(uint8_t reg_name);
    uint16_t read_register16bit(uint8_t reg_name);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    bool ready;
    twi_state_t twi_state;

    const uint8_t twi_address;
    const uint8_t interrupt_pin;

    uint8_t touch_status;
    uint8_t touch_status_old;
    bool touch_status_overcurrent_flag;

    uint32_t timestamp_lastread;
    uint32_t update_interval;

    callback_t callback_touch_event;

};  // class slight_MPR121

#endif  // slight_MPR121_H_

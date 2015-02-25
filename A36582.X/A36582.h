#ifndef __A36582_H
#define __A36582_H

#include "ETM_ANALOG.h"

/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer2 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer3 - Used/Configured by ETM CAN - Used for detecting error on can bus

  I2C    - Used/Configured by EEPROM Module

  Timer4 - Used for looking at time between pulses
  Timer5 - Used for 10msTicToc

  ADC Module - See Below For Specifics

  INT1   - Pulse Trigger - used to trigger ADC conversion and pulse sequence
  INT3   - Lowest level current latch.  Used to look for current in pulse tank (without a trigger) to detect faulty trigger


  PIN_OUT_TP_E  - A 2us pulse is set to this test point when an arc is detected

*/








// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero

// ----------------- DIGITAL INPUT PINS --------------- //
/*

  RA12 (INT1) - Fiber Trigger IN
  RA14 (INT3) - PULSE OVER CURRENT LATCH 4


  RD8  - PULSE OVER CURRENT LATCH 1
  RD9  - PULSE OVER CURRENT LATCH 2
  RD10 - PULSE OVER CURRENT LATCH 3
  RD11 - PULSE OVER CURRENT LATCH 4

  Pins that are overidden by a hardware module and should be left as inputs during port configuration
  RA9  - ADC VREF-
  RA10 - ADC VREF+

  RB0  - PROGRAM/DEBUG
  RB1  - PROGRAM/DEBUG
  RB2  - Analog Input - PIC ADC MAGNETRON IMON
  RB13 - Analog Input - ADC 5V VMON

  RF0 CAN 1
  RF1 CAN 1
  
  RG2 I2C
  RG3 I2C
  RG6 SPI 1
  RG7 SPI 1
  RG8 SPI 1
  RG14 - Reset Detect
  
  
  // ALL OTHER PINS SHOULD BE CONFIGURED AS OUTPUTS

*/

//   ------------------  Digital Output Pins ---------------
/*


  RC2 - ADC CONVERT
  RC4 - ADC CHIP SELECT
  RD0 - PULSE LATCH RESET (This is also Output Compare 1 - If we want to use that module to generate reset signal)

  RA7 - LED Operational
  RB8 - Test Point E
  RB9 - Test Point F
  RF4 - Test Point A
  RF5 - Test Point B
  RG0 - Test Point C
  RG1 - Test Point D
  RG12 - LED A RED
  RG13 - LED B GREEN
  
*/


#define A36582_TRISA_VALUE 0b0101011000000000 
#define A36582_TRISB_VALUE 0b0010000000000111
#define A36582_TRISC_VALUE 0b0000000000000000
#define A36582_TRISD_VALUE 0b0000111100000000
#define A36582_TRISF_VALUE 0b0000000000000011
#define A36582_TRISG_VALUE 0b0100000111001100

// ---------- DIGITAL INPUT PINS ------------ //
#define PIN_PULSE_OVER_CURRENT_LATCH_1        _RD8
#define PIN_PULSE_OVER_CURRENT_LATCH_2        _RD9
#define PIN_PULSE_OVER_CURRENT_LATCH_3        _RD10
#define PIN_PULSE_OVER_CURRENT_LATCH_4        _RD11

#define ILL_LATCH_SET                         1

// ---------- DIGITAL OUTPUT PINS ------------ //
#define PIN_RESET_DETECT                      _RG14

#define PIN_ADC_CONVERT                       _LATC2
#define OLL_ADC_START_CONVERSION              1

#define PIN_ADC_CHIP_SELECT                   _LATC4
#define OLL_ADC_SELECT_CHIP                   0

#define PIN_PULSE_LATCH_RESET                 _LATD0
#define OLL_RESET_LATCHES                     0

#define PIN_LED_OPERATIONAL_GREEN             _LATA7
#define PIN_LED_A_RED                         _LATG12
#define PIN_LED_B_GREEN                       _LATG13  // This is is configured by the CAN module to flash on CAN Bus activity
#define OLL_LED_ON                            0

#define PIN_OUT_TP_E                          _LATB8
#define PIN_OUT_TP_F                          _LATB9
#define PIN_OUT_TP_A                          _LATF4
#define PIN_OUT_TP_B                          _LATF5
#define PIN_OUT_TP_C                          _LATG1
#define PIN_OUT_TP_D                          _LATG0

// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN3 - Magnetron Imon
   AN13 - 5V Mon
   
   AT startup, sample 5V input
   After Startup - Sample Magnetron Current with each trigger on INT1
*/


// Generic Module Settings
#define ADPCFG_SETTING          (ENABLE_AN2_ANA & ENABLE_AN13_ANA)

// Settings for reading voltage at startup
#define ADCON1_SETTING_STARTUP  (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING_STARTUP  (ADC_VREF_EXT_EXT & ADC_SCAN_OFF & ADC_SAMPLES_PER_INT_16 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING_STARTUP  (ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_10Tcy)
#define ADCHS_SETTING_STARTUP   (ADC_CH0_POS_SAMPLEA_AN13 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN13 & ADC_CH0_NEG_SAMPLEB_VREFN)


// Settings for reading Magnetron Current
#define ADCON1_SETTING_OPERATE  (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_MANUAL & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING_OPERATE  (ADC_VREF_EXT_EXT & ADC_SCAN_OFF & ADC_SAMPLES_PER_INT_1 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING_OPERATE  (ADC_SAMPLE_TIME_0 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_4Tcy)
#define ADCHS_SETTING_OPERATE   (ADC_CH0_POS_SAMPLEA_AN2 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN2 & ADC_CH0_NEG_SAMPLEB_VREFN)


/* 
   TMR4 Configuration
   Timer4 is used to see if the time between triggers is too short
   With 10Mhz Clock, x8 multiplier will yield max period of 52.4mS, 800nS per tick
*/

#define MINIMUM_PULSE_PERIOD_US   2200

#define T4CON_VALUE                    (T4_ON & T4_IDLE_CON & T4_GATE_OFF & T4_PS_1_8 & T4_SOURCE_INT & T4_32BIT_MODE_OFF)
#define MINIMUM_PULSE_PERIOD_T4        2750 //(FCY_CLK_MHZ*MINIMUM_PULSE_PERIOD_US/8)

#define TIMER_4_TIME_2_MILLISECONDS    2500

/* 
   TMR5 Configuration
   Timer5 - Used for 10msTicToc
   Period should be set to 10mS
*/

#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR5_PERIOD_US/8)



// State Definitions
#define STATE_STARTUP                10
#define STATE_FLASH_LED              20
#define STATE_OPERATE                50
#define STATE_FAULT                  60


#define _STATUS_HIGH_ENERGY                             _STATUS_0
#define _STATUS_ARC_DETECTED                            _STATUS_1

#define _FAULT_ARC_SLOW                                 _FAULT_0
#define _FAULT_ARC_FAST                                 _FAULT_1
#define _FAULT_ARC_CONTINUOUS                           _FAULT_2
#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_3
#define _FAULT_FALSE_TRIGGER                            _FAULT_4

typedef struct {
  AnalogInput imag_internal_adc;               // 10mA per LSB
  AnalogInput imag_external_adc;               // 10mA per LSB
  AnalogInput analog_input_5v_mon;             // 1mV per LSB

  unsigned int filt_int_adc_high;
  unsigned int filt_int_adc_low;

  unsigned int filt_ext_adc_high;
  unsigned int filt_ext_adc_low;

  unsigned int control_state;

  //unsigned int led_divider;
  
  unsigned int arc_this_hv_on;
  unsigned long pulse_this_hv_on;

  unsigned long      arc_total;     // When pulsing these need to get written to EEPROM periodically 
  unsigned long long pulse_total;   // Need to switch to FRAM part so that we can write once per second continuously
  unsigned int       count_crc;
  unsigned int       next_register;
  
  unsigned int fast_arc_counter;
  unsigned int slow_arc_counter;
  unsigned int consecutive_arc_counter;

  unsigned int pulse_counter_fast;
  unsigned int pulse_counter_slow;

  unsigned int sample_index;
  unsigned int sample_energy_mode;
  unsigned int sample_complete;


  unsigned int minimum_pulse_period_fault_count;
  unsigned int pulse_with_no_trigger_counter;
  
  unsigned int led_flash_counter;
  unsigned int millisecond_counter;

  unsigned int false_trigger_decrement_counter;
  unsigned int false_trigger_counter;

  unsigned int over_current_arc_count;
  unsigned int under_current_arc_count;

} MagnetronCurrentMonitorGlobalData;

extern MagnetronCurrentMonitorGlobalData global_data_A36582;

#define PULSE_COUNT_REGISTER_A            0x20
#define PULSE_COUNT_REGISTER_B            0x21


#endif

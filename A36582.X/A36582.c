// This is firmware for the Magnetron Current Monitor Board

#include <libpic30.h>
#include <adc12.h>
#include <p30f6014a.h>
#include <timer.h>
#include "A36582.h"
#include "FIRMWARE_VERSION.h"
#include "ETM_EEPROM.h"
#include "A36582_SETTINGS.h"
#include "ETM_CAN_PUBLIC.h"
#include "ETM_SPI.h"
#include "ETM_RC_FILTER.h"


_FOSC(ECIO & CSW_FSCM_OFF); 
//_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_64 & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


unsigned int dan_test_int;
unsigned int dan_test_ext;
unsigned int dan_test_spc;

void InitializeA36582(void);
void DoStateMachine(void);
void DoA36582(void);
void DoStartupLEDs(void);
void DoPostPulseProcess(void);
void ResetPulseLatches(void);

MagnetronCurrentMonitorGlobalData global_data_A36582;

ETMEEProm U17_M24LC64F;


int main(void) {
  global_data_A36582.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}


void DoStateMachine(void) {
  switch (global_data_A36582.control_state) {
    
  case STATE_STARTUP:
    InitializeA36582();
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    global_data_A36582.control_state = STATE_FLASH_LED;
    break;


  case STATE_FLASH_LED:
    _CONTROL_NOT_READY = 1;
    ResetPulseLatches();
    while (global_data_A36582.control_state == STATE_FLASH_LED) {
      DoA36582();
      DoStartupLEDs();
      
      if (global_data_A36582.led_flash_counter >= LED_STARTUP_FLASH_TIME) {
	global_data_A36582.control_state = STATE_OPERATE;
      }
    }
    break;
    
  case STATE_OPERATE:
    global_data_A36582.fast_arc_counter = 0;
    global_data_A36582.slow_arc_counter = 0;
    global_data_A36582.consecutive_arc_counter = 0;
    global_data_A36582.pulse_counter_fast = 0;
    global_data_A36582.pulse_counter_slow = 0;
    _FAULT_REGISTER = 0;
    _CONTROL_NOT_READY = 0;
    while (global_data_A36582.control_state == STATE_OPERATE) {
      DoA36582();
      if (global_data_A36582.sample_complete) {
	global_data_A36582.sample_complete = 0;
	DoPostPulseProcess();
      }

      if (_FAULT_REGISTER) {
	global_data_A36582.control_state = STATE_FAULT;
      }
    }
    break;
    
    
  case STATE_FAULT:
    _CONTROL_NOT_READY = 1;
    while (global_data_A36582.control_state == STATE_FAULT) {
      DoA36582();
      if (global_data_A36582.sample_complete) {
	global_data_A36582.sample_complete = 0;
	DoPostPulseProcess();
      }

      if (_SYNC_CONTROL_RESET_ENABLE) {
	global_data_A36582.control_state = STATE_OPERATE;
      }
    }
    break;
    

  default:
    global_data_A36582.control_state = STATE_FAULT;
    break;
  }
}


void DoA36582(void) {
  ETMCanDoCan();
  
  if (_CONTROL_CAN_COM_LOSS) {
    _FAULT_CAN_COMMUNICATION_LATCHED = 1;
  }

  // If the system is faulted or inhibited set the red LED
  if (_CONTROL_NOT_READY) {
    PIN_LED_A_RED = OLL_LED_ON;
  } else {
    PIN_LED_A_RED = !OLL_LED_ON;
  }
  
  if (_T5IF) {
    _T5IF = 0;
    // 10ms has passed
    if (global_data_A36582.control_state == STATE_FLASH_LED) {
      global_data_A36582.led_flash_counter++;
    }
    
    
    
    local_debug_data.debug_0 = global_data_A36582.fast_arc_counter;
    local_debug_data.debug_1 = global_data_A36582.slow_arc_counter;
    local_debug_data.debug_2 = global_data_A36582.consecutive_arc_counter;
    local_debug_data.debug_4 = global_data_A36582.filt_int_adc_low;
    local_debug_data.debug_5 = global_data_A36582.filt_ext_adc_low;
    local_debug_data.debug_6 = global_data_A36582.filt_int_adc_high;
    local_debug_data.debug_7 = global_data_A36582.filt_ext_adc_high;
    local_debug_data.debug_8 = global_data_A36582.imag_external_adc.reading_scaled_and_calibrated;
    local_debug_data.debug_9 = global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated;
  }
}


void DoStartupLEDs(void) {
  switch ((global_data_A36582.led_flash_counter >> 4) & 0b11) 
    {
      
    case 0:
      PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;
      
    case 1:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;
      
    case 2:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;
      
    case 3:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = OLL_LED_ON;
      break;
      
    }
}


void InitializeA36582(void) {
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  
  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;


  // Configure Trigger Interrupt
  _INT1IP = 7; // This must be the highest priority interrupt
  _INT1EP = 1; // Negative Transition
  _INT1IE = 1;
  
  // Configure the "False Trigger" Interrupt
  _INT3IP = 6; // This must be the highest priority interrupt
  _INT3EP = 0; // Positive Transition

  // By Default, the can module will set it's interrupt Priority to 4
  
  
  // Initialize all I/O Registers
  TRISA = A36582_TRISA_VALUE;
  TRISB = A36582_TRISB_VALUE;
  TRISC = A36582_TRISC_VALUE;
  TRISD = A36582_TRISD_VALUE;
  TRISF = A36582_TRISF_VALUE;
  TRISG = A36582_TRISG_VALUE;




  // Initialize TMR4
  TMR4  = 0;
  _T4IF = 0;
  T4CON = T4CON_VALUE;

  
  // Initialize TMR5
  PR5   = PR5_VALUE_10_MILLISECONDS;
  TMR5  = 0;
  _T5IF = 0;
  T5CON = T5CON_VALUE;


  
  // Initialize the External EEprom
  ETMEEPromConfigureDevice(&U17_M24LC64F, EEPROM_I2C_ADDRESS_0, I2C_PORT, EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD);


  // Initialize the Can module
  ETMCanInitialize();


  // Initialize the Analog input data structures
  // DPARKER set the scale factors
  ETMAnalogInitializeInput(&global_data_A36582.imag_internal_adc, MACRO_DEC_TO_SCALE_FACTOR_16(.25075), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36582.imag_external_adc, MACRO_DEC_TO_SCALE_FACTOR_16(.25075), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION, 
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36582.analog_input_5v_mon, MACRO_DEC_TO_SCALE_FACTOR_16(.12500), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   PWR_5V_OVER_FLT, PWR_5V_UNDER_FLT, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);


  // Configure SPI port, used by External ADC
  ConfigureSPI(ETM_SPI_PORT_2, ETM_DEFAULT_SPI_CON_VALUE, ETM_DEFAULT_SPI_CON2_VALUE, ETM_DEFAULT_SPI_STAT_VALUE, SPI_CLK_2_MBIT, FCY_CLK);
 
 
  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O

  ADCON1 = ADCON1_SETTING_STARTUP;     // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING_STARTUP;     // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING_STARTUP;     // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING_STARTUP;      // Configure the high speed ADC module based on H file parameters
  //ADCSSL = ADCSSL_SETTING_STARTUP;
  
  _ADIF = 0;
  _ADON = 1;

  while (_ADIF == 0); // Wait for 16 ADC conversions to complete;
  _ADON = 0;
  global_data_A36582.analog_input_5v_mon.filtered_adc_reading  = ADCBUF0 + ADCBUF1 + ADCBUF2 +ADCBUF3 + ADCBUF4 + ADCBUF5 + ADCBUF6 + ADCBUF7;
  global_data_A36582.analog_input_5v_mon.filtered_adc_reading += ADCBUF8 + ADCBUF9 + ADCBUFA +ADCBUFB + ADCBUFC + ADCBUFD + ADCBUFE + ADCBUFF;

  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.analog_input_5v_mon);

  if (ETMAnalogCheckOverAbsolute(&global_data_A36582.analog_input_5v_mon)) {
    _CONTROL_SELF_CHECK_ERROR = 1;
    ETMCanSetBit(&local_debug_data.self_test_result_register, SELF_TEST_5V_OV);
  }
  
  if (ETMAnalogCheckUnderAbsolute(&global_data_A36582.analog_input_5v_mon)) {
    _CONTROL_SELF_CHECK_ERROR = 1;
    ETMCanSetBit(&local_debug_data.self_test_result_register, SELF_TEST_5V_UV);
  }
  
  ADCON1 = ADCON1_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING_OPERATE;      // Configure the high speed ADC module based on H file parameters
  //ADCSSL = ADCSSL_SETTING_STARTUP;
  
  _ADIF = 0;
  _ADON = 1;
  _SAMP = 1;

  // Run a dummy conversion
  __delay32(400);
  _SAMP = 0;

}


void DoPostPulseProcess(void) {
    // Process the pulse data
  
  // Wait 40us for the conversions to complete (and the noise from the arc to dissipate)
  __delay32(400);
  
  // Read the analog current level from external ADC
  PIN_ADC_CHIP_SELECT = OLL_ADC_SELECT_CHIP;
  global_data_A36582.imag_external_adc.filtered_adc_reading = SendAndReceiveSPI(0, ETM_SPI_PORT_2);
  dan_test_ext = global_data_A36582.imag_external_adc.filtered_adc_reading;
  PIN_ADC_CHIP_SELECT = !OLL_ADC_SELECT_CHIP;
  PIN_ADC_CONVERT = !OLL_ADC_START_CONVERSION;
  PIN_OUT_TP_F = 0;
  
  // Read the analog current level from internal ADC
  global_data_A36582.imag_internal_adc.filtered_adc_reading = (ADCBUF0 << 4);
  dan_test_int = global_data_A36582.imag_internal_adc.filtered_adc_reading;

  // Scale the readings
  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.imag_internal_adc);
  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.imag_external_adc);
  

  // Look for an arc
  _STATUS_ARC_DETECTED = 0;

  if ((PIN_PULSE_OVER_CURRENT_LATCH_1 == ILL_LATCH_SET) || (PIN_PULSE_OVER_CURRENT_LATCH_4 != ILL_LATCH_SET)) {
    // The current after the trigger was too high or too low
    _STATUS_ARC_DETECTED = 1;
    if (PIN_OUT_TP_C) {
      PIN_OUT_TP_C = 0;
    } else {
      PIN_OUT_TP_C = 1;
    }
  }

  // DPARKER Consider checking the analog current reading to also look for arc

  if (_STATUS_ARC_DETECTED) {
    // Trigger Test Point E for 2uS after an arc is detected
    PIN_OUT_TP_E = 1;
    __delay32(20);
    PIN_OUT_TP_E = 0;
    
    global_data_A36582.arc_total++;
    global_data_A36582.arc_this_hv_on++;
    global_data_A36582.fast_arc_counter++;
    global_data_A36582.slow_arc_counter++;
    global_data_A36582.consecutive_arc_counter++;
  } else {
    if (global_data_A36582.consecutive_arc_counter) { 
      global_data_A36582.consecutive_arc_counter--;
    }
  }

  global_data_A36582.pulse_total++;
  global_data_A36582.pulse_this_hv_on++;
	
  // Decrement fast_arc_counter if needed
  global_data_A36582.pulse_counter_fast++;
  if (global_data_A36582.pulse_counter_fast > ARC_COUNTER_FAST_DECREMENT_INTERVAL) {
    global_data_A36582.pulse_counter_fast = 0;
    if (global_data_A36582.fast_arc_counter) {
      global_data_A36582.fast_arc_counter--;
    }
  }
	
  // Decrement slow_arc_counter if needed
  global_data_A36582.pulse_counter_slow++;
  if (global_data_A36582.pulse_counter_slow > ARC_COUNTER_SLOW_DECREMENT_INTERVAL) {
    global_data_A36582.pulse_counter_slow = 0;
    if (global_data_A36582.slow_arc_counter) {
      global_data_A36582.slow_arc_counter--;
    }
  }

  // Look for ARC faults
  if (global_data_A36582.slow_arc_counter >= ARC_COUNTER_SLOW_MAX_ARCS) {
    _FAULT_ARC_SLOW = 1;
  }

  if (global_data_A36582.fast_arc_counter >= ARC_COUNTER_FAST_MAX_ARCS) {
    _FAULT_ARC_FAST = 1;
  }
	
  if (global_data_A36582.consecutive_arc_counter >= ARC_COUNTER_CONSECUTIVE_MAX) {
    _FAULT_ARC_CONTINUOUS = 1;
  }
	
  // Filter the ADC current readings
  if (!_STATUS_ARC_DETECTED) {
    if (_STATUS_HIGH_ENERGY) {
      global_data_A36582.filt_int_adc_high = RCFilterNTau(global_data_A36582.filt_int_adc_high,
							  global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated,
							  RC_FILTER_64_TAU);
      
      global_data_A36582.filt_ext_adc_high = RCFilterNTau(global_data_A36582.filt_ext_adc_high,
							  global_data_A36582.imag_external_adc.reading_scaled_and_calibrated,
							  RC_FILTER_64_TAU);
    } else {
      global_data_A36582.filt_int_adc_low = RCFilterNTau(global_data_A36582.filt_int_adc_low,
							 global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated,
							 RC_FILTER_64_TAU);
      
      global_data_A36582.filt_ext_adc_low = RCFilterNTau(global_data_A36582.filt_ext_adc_low,
							 global_data_A36582.imag_external_adc.reading_scaled_and_calibrated,
							 RC_FILTER_64_TAU);
    }
  }

  


  // Reset the Latches
  ResetPulseLatches();
	
  ETMCanLogCustomPacketC();  // This is the data log packet that contains the data for the previous pulse
  
}

void ResetPulseLatches(void) {
  PIN_PULSE_LATCH_RESET = OLL_RESET_LATCHES;
  __delay32(20);
  PIN_PULSE_LATCH_RESET = !OLL_RESET_LATCHES;
}


//void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
  /*
    A sample trigger has been received
  */ 
  
  // Trigger the internal ADC to start conversion
  _SAMP = 0;  // There Appears to be a delay of ~3 ADC Clocks between this and the sample being held (and the conversion starting)
              // I think the ADC clock is running if the ADC is on, therefor we have a sampleing error of up to +/- 1/2 ADC Clock.  Ugh!!!
  PIN_OUT_TP_F = 1;
  // DPARKER delay until we are in the middle of the current pulse to sample
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();

  // Trigger the external ADC to start conversion
  PIN_ADC_CONVERT = OLL_ADC_START_CONVERSION;


  global_data_A36582.sample_energy_mode = _STATUS_HIGH_ENERGY;
  global_data_A36582.sample_index = etm_can_next_pulse_count;
  global_data_A36582.sample_complete = 1;

  // Check that there was enough time between pulses
  //
  if ((TMR4 <= MINIMUM_PULSE_PERIOD_T4) && (_T4IF == 0)) {
    global_data_A36582.minimum_pulse_period_fault_count++;
  }

  _T4IF = 0;
  TMR4 = 0;
  _INT1IF = 0;
  PIN_OUT_TP_D = 0;
}
  

/*
  Figure out if there was a pulse without a trigger
  The trigger pulse should come before the current pulse
  _T4IF will be cleared and TMR4 will be set to zero in the trigger interrupt
  This will detect detect a pulse without a trigger being present by checking those two 
*/
void __attribute__((interrupt, no_auto_psv)) _INT3Interrupt(void) {
  // There was trigger on INT3
  if (_T4IF || TMR4 > 20) {
    global_data_A36582.pulse_with_no_trigger_counter++;
  }   
}



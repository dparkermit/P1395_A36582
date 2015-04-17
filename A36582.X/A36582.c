// This is firmware for the Magnetron Current Monitor Board

#include <libpic30.h>
#include <adc12.h>
#include <xc.h>
#include <timer.h>
#include "A36582.h"
#include "FIRMWARE_VERSION.h"
#include "ETM_EEPROM.h"
#include "A36582_SETTINGS.h"
#include "ETM_SPI.h"
#include "ETM_RC_FILTER.h"
#include "P1395_MODULE_CONFIG.h"
#include "P1395_CAN_SLAVE.h"

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
void SavePulseCountersToEEProm(void);
unsigned int MakeCountCRC(unsigned int* data_ptr);

MagnetronCurrentMonitorGlobalData global_data_A36582;


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
    global_data_A36582.false_trigger_counter = 0;
    global_data_A36582.under_current_arc_count = 0;
    global_data_A36582.over_current_arc_count = 0;
    _FAULT_REGISTER = 0;
    _CONTROL_NOT_READY = 0;
    while (global_data_A36582.control_state == STATE_OPERATE) {
      DoA36582();
      if (global_data_A36582.sample_complete) {
	DoPostPulseProcess();
	global_data_A36582.sample_complete = 0;
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
  ETMCanSlaveDoCan();
  
  if (_SYNC_CONTROL_CLEAR_DEBUG_DATA) {
    global_data_A36582.arc_this_hv_on = 0;
    global_data_A36582.pulse_this_hv_on = 0;
  }

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
    
    // Run at 1 second interval
    global_data_A36582.millisecond_counter += 10;
    if (global_data_A36582.millisecond_counter >= 1000) {
      global_data_A36582.millisecond_counter = 0;
      SavePulseCountersToEEProm();
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
    local_debug_data.debug_A = global_data_A36582.pulse_with_no_trigger_counter;
    local_debug_data.debug_B = global_data_A36582.minimum_pulse_period_fault_count;
    local_debug_data.debug_C = global_data_A36582.false_trigger_counter;
    local_debug_data.debug_D = global_data_A36582.over_current_arc_count;
    local_debug_data.debug_E = global_data_A36582.under_current_arc_count;
    

    // Update tthe false trigger counter
    global_data_A36582.false_trigger_decrement_counter++;
    if (global_data_A36582.false_trigger_decrement_counter >= FALSE_TRIGGER_DECREMENT_10_MS_UNITS) {
      global_data_A36582.false_trigger_decrement_counter = 0;
      if (global_data_A36582.false_trigger_counter) {
	global_data_A36582.false_trigger_counter--;
      }
    }
    if (global_data_A36582.false_trigger_counter >= FALSE_TRIGGERS_FAULT_LEVEL) {
      _FAULT_FALSE_TRIGGER = 1;
    }
  }
  

  // DPARKER - THIS DID NOT WORK - IT RESET THE LATCHES WHEN THEY SHOULD NOT BE AND CAUSED FALSE ARCS TO BE DETECTED
  // However we may need some way to reset the pulse latches if they are set for a long time
  // If this happens, INT3 will not trigger and we will loose the ability to detect pulses without a trigger
  // Perhaps another counter that if there has been no trigger for the previous second, then if the latches are set they are cleared.
  
  // Alternatively we could check the state of the latches inside the interrupt.  That way the checks can't be broken by this call
  
  /*
  // Reset the pulse latches if they are set and it has been more than 2ms since the last pulse
  if ((TMR4 > TIMER_4_TIME_2_MILLISECONDS) && (PIN_PULSE_OVER_CURRENT_LATCH_4 == ILL_LATCH_SET)) {

    ResetPulseLatches();
    // DPARKER. why doesn't this clear the fault latches before a fault latch check.  You would think that the TMR4 check would prevent this.
  // Perhaps adding a long delay and then reckecking TMR4 before the ResetPulseLatches would fix the problem?
  }
  */
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
  unsigned int pulse_data[7];

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
  //_INT1EP = 1; // Negative Transition
  _INT1IE = 1;
  
  // Configure the "False Trigger" Interrupt
  _INT3IP = 6; // This must be the highest priority interrupt
  _INT3EP = 0; // Positive Transition
  _INT3IE = 1;

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
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);


  // Initialize the Can module
  ETMCanSlaveInitialize();


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
    //ETMCanSlaveSetBit(&local_debug_data.self_test_result_register, SELF_TEST_5V_OV);
    // DPARKER use the self test bits
  }
  
  if (ETMAnalogCheckUnderAbsolute(&global_data_A36582.analog_input_5v_mon)) {
    _CONTROL_SELF_CHECK_ERROR = 1;
    //ETMCanSlaveSetBit(&local_debug_data.self_test_result_register, SELF_TEST_5V_UV);
    // DPARKER use the self test bits
  }
  
  ADCON1 = ADCON1_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING_OPERATE;      // Configure the high speed ADC module based on H file parameters
  //ADCSSL = ADCSSL_SETTING_STARTUP;
  
  _ADIF = 0;
  _ADON = 1;
  _SAMP = 1;


  // Read Data from EEPROM
  ETMEEPromReadPage(PULSE_COUNT_REGISTER_A, 7, &pulse_data[0]);
  
  // If the data checks out, update with data
  if (pulse_data[6] == MakeCountCRC(&pulse_data[0])) {
    global_data_A36582.arc_total = *(unsigned long*)&pulse_data[0];
    global_data_A36582.pulse_total = *(unsigned long long*)&pulse_data[2];
    if ((global_data_A36582.arc_total == 0xFFFFFFFF) || global_data_A36582.pulse_total == 0xFFFFFFFFFFFFFFFF) {
      global_data_A36582.arc_total = 0;
      global_data_A36582.pulse_total = 0;
    }
  }
  // DPARKER, check the B register
  
  
  // Run a dummy conversion
  _SAMP = 0;

}

unsigned int MakeCountCRC(unsigned int* data_ptr) {
  unsigned int crc;
  crc = *data_ptr;
  data_ptr++;
  crc += *data_ptr;
  data_ptr++;
  crc += *data_ptr;
  data_ptr++;
  crc += *data_ptr;
  data_ptr++;
  crc += *data_ptr;
  data_ptr++;
  crc += *data_ptr;

  return crc;
}

void SavePulseCountersToEEProm(void) {
  unsigned int test_data[7];

  global_data_A36582.count_crc = MakeCountCRC((unsigned int*)&global_data_A36582.arc_total);
  
  if(global_data_A36582.next_register) {
    // Write to "Page A" 

    // Write Data to EEPROM
    ETMEEPromWritePage(PULSE_COUNT_REGISTER_A, 7, (unsigned int*)&global_data_A36582.arc_total);

    // Read Data from EEPROM
    ETMEEPromReadPage(PULSE_COUNT_REGISTER_A, 7, &test_data[0]);
    
    // If the data checks out, update next register
    if (test_data[6] == MakeCountCRC(&test_data[0])) {
      global_data_A36582.next_register = 0;
    }
 
  } else  {
    // Write to "Page B"

    // Write Data to EEPROM
    ETMEEPromWritePage(PULSE_COUNT_REGISTER_B, 7, (unsigned int*)&global_data_A36582.arc_total);
    
    // Read Data from EEPROM
    ETMEEPromReadPage(PULSE_COUNT_REGISTER_B, 7, &test_data[0]);
    
    // If the data checks out, update next register
    if (test_data[6] == MakeCountCRC(&test_data[0])) {
      global_data_A36582.next_register = 1;
    }    
    
  }
    
  


}


void DoPostPulseProcess(void) {
    // Process the pulse data
  
  // Wait 40us for the conversions to complete (and the noise from the arc to dissipate)
  //__delay32(400);
  
  
  // Read the analog current level from internal ADC
  // DPARKRER this should be ~zero with the new timing strategy
  global_data_A36582.imag_internal_adc.filtered_adc_reading = (ADCBUF0 << 4);
  dan_test_int = global_data_A36582.imag_internal_adc.filtered_adc_reading;
  //_LATF6 = 0;

  // Scale the readings
  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.imag_internal_adc);
  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.imag_external_adc);
  

  // Look for an arc
  _STATUS_ARC_DETECTED = 0;

  if ((PIN_PULSE_OVER_CURRENT_LATCH_1 == ILL_LATCH_SET) || (PIN_PULSE_OVER_CURRENT_LATCH_4 != ILL_LATCH_SET)) {
    // The current after the trigger was too high or too low
    PIN_OUT_TP_C = 1;
    _STATUS_ARC_DETECTED = 1;
   
    if (PIN_PULSE_OVER_CURRENT_LATCH_1 == ILL_LATCH_SET) {
      global_data_A36582.over_current_arc_count++;
    }
    if (PIN_PULSE_OVER_CURRENT_LATCH_4 != ILL_LATCH_SET) {
      global_data_A36582.under_current_arc_count++;
    }

    __delay32(500);  // 50us Trigger pulse that we had an arc
    PIN_OUT_TP_C = 0;
    
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
  _STATUS_HIGH_ENERGY = global_data_A36582.sample_energy_mode;
  if (!_STATUS_ARC_DETECTED) { 
    if (global_data_A36582.sample_energy_mode) {
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
  
  if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
    ETMCanSlaveLogCustomPacketC();  // This is the data log packet that contains the data for the previous pulse
  }
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
  /*
  Nop(); //100ns
  Nop(); //200ns
  Nop(); //300ns
  Nop(); //400ns
  Nop(); //500ns
  Nop(); //600ns
  Nop(); //700ns
  Nop(); //800ns
  Nop(); //900ns
  Nop(); //1000ns
  */
  // Trigger the internal ADC to start conversion
  _SAMP = 0;  // There Appears to be a delay of ~3 ADC Clocks between this and the sample being held (and the conversion starting)
              // I think the ADC clock is running if the ADC is on, therefor we have a sampleing error of up to +/- 1/2 ADC Clock.  Ugh!!!
  //_LATF6 = 1;
  // DPARKER delay until we are in the middle of the current pulse to sample

  global_data_A36582.sample_energy_mode = etm_can_next_pulse_level;
  global_data_A36582.sample_index = etm_can_next_pulse_count;
  global_data_A36582.sample_complete = 1;

  // Check that there was enough time between pulses
  //
  if ((TMR4 <= MINIMUM_PULSE_PERIOD_T4) && (_T4IF == 0)) {
    global_data_A36582.minimum_pulse_period_fault_count++;
  }
  _T4IF = 0;
  TMR4 = 0;
  

  // Wait for the pulse energy to dissipate
  __delay32(150);

  // Read the data from port
  PIN_OUT_TP_F = 1;
  PIN_ADC_CHIP_SELECT = OLL_ADC_SELECT_CHIP;
  global_data_A36582.imag_external_adc.filtered_adc_reading = SendAndReceiveSPI(0, ETM_SPI_PORT_2);
  dan_test_ext = global_data_A36582.imag_external_adc.filtered_adc_reading;
  PIN_ADC_CHIP_SELECT = !OLL_ADC_SELECT_CHIP;
  PIN_OUT_TP_F = 0;

  _INT1IF = 0;
}
  

/*
  Figure out if there was a pulse without a trigger
  The trigger pulse is a sample pulse so it comes in the middle of current pulse.
  We should wait for 10us After this is entered.  If there has not been a trigger pulse durring that time 
  then it was a false tirgger
*/
void __attribute__((interrupt, no_auto_psv)) _INT3Interrupt(void) {
  // There was trigger on INT3
  _INT3IF = 0;
  __delay32(100); // wait for 10us

  if (!global_data_A36582.sample_complete) {
    // There was a current pulse without a sample trigger within the next 10us
    global_data_A36582.pulse_with_no_trigger_counter++;
    global_data_A36582.false_trigger_counter++;
    ResetPulseLatches();
  }   
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}



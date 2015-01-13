#include "A36582.h"
#include "FIRMWARE_VERSION.h"
#include "ETM_EEPROM.h"

// This is firmware for the HV Lambda Board

_FOSC(ECIO & CSW_FSCM_OFF); 
//_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_64 & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


void InitializeA36582(void);
void DoStateMachine(void);
void DoA36582(void);
void DoStartupLEDs(void);
void DoPostPulseProcess(void);

MagnetronCurrentMonitorGlobalData global_data_A36582;

ETMEEProm U17_M24LC64F;


int main(void) {
  global_data_A36582.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}


#define LED_STARTUP_FLASH_TIME    400 // 4 Seconds

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
    while (global_data_A36582.control_state == STATE_FLASH_LED) {
      DoA36582();
      DoStartupLEDs();
      
      if (global_data_A36582.led_flash_counter >= LED_STARTUP_FLASH_TIME) {
	global_data_A36582.control_state = STATE_OPERATE;
      }
    }
    break;
    
  case STATE_OPERATE:
    _FAULT_REGISTER = 0;
    _CONTROL_NOT_READY = 0;
    while (global_data_A36582.control_state == STATE_OPERATE) {
      DoA36582();
      if (global_data_A36582.sample_complete) {
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
  
  if (global_data_A36582.control_state == STATE_FLASH_LED) {
    global_data_A36582.led_flash_counter++;
  }

  if (_CONTROL_CAN_COM_LOSS) {
    _FAULT_CAN_COMMUNICATION_LATCHED = 1;
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
  _INT1EP = 0; // Positive Transition
  
  // Configure the "False Trigger" Interrupt
  _INT1IP = 6; // This must be the highest priority interrupt
  _INT1EP = 0; // Positive Transition

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



#define PWR_5V_OVER_FLT        5200                   // 5.2 V
#define PWR_5V_UNDER_FLT       4800                   // 4.8 V


  // Initialize the Analog input data structures
  // DPARKER set the scale factors
  ETMAnalogInitializeInput(&global_data_A36582.analog_input_magnetron_current_internal_adc, MACRO_DEC_TO_SCALE_FACTOR_16(.25075), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36582.analog_input_magnetron_current_external_adc, MACRO_DEC_TO_SCALE_FACTOR_16(.25075), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION, 
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36582.analog_input_5v_mon, MACRO_DEC_TO_SCALE_FACTOR_16(.12500), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   PWR_5V_OVER_FLT, PWR_5V_UNDER_FLT, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);


#define SPI_CLK_1_MBIT          1000000
#define SPI_CLK_2_MBIT          2000000

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
}


void DoPostPulseProcess(void) {
    // Process the pulse data
  
  // Wait 40us for the conversions to complete (and the noise from the arc to dissipate)
  __delay32(400);
  
  // Read the analog current level from external ADC
  PIN_ADC_CHIP_SELECT = OLL_ADC_SELECT_CHIP;
  global_data_A36582.analog_input_magnetron_current_external_adc.filtered_adc_reading = SendAndReceiveSPI(0, ETM_SPI_PORT_2);
  PIN_ADC_CHIP_SELECT = !OLL_ADC_SELECT_CHIP;
  PIN_ADC_CONVERT = !OLL_ADC_START_CONVERSION;
  
  // Read the analog current level from internal ADC
  global_data_A36582.analog_input_magnetron_current_internal_adc.filtered_adc_reading = ADCBUF0;  // DPARKER this will probably keep incrementing.  Need to reset the module so that it always stores in ADCBUF0
  // Note sure how to do this at this time

  // Look for an arc
  _STATUS_ARC_DETECTED = 0;

  if ((PIN_PULSE_OVER_CURRENT_LATCH_1 == ILL_LATCH_SET) || (PIN_PULSE_OVER_CURRENT_LATCH_4 != ILL_LATCH_SET)) {
    // The current after the trigger was too high or too low
    _STATUS_ARC_DETECTED = 1;
  }

  // DPARKER Consider checking the analog current reading to also look for arc

  if (_STATUS_ARC_DETECTED) {
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
  if (global_data_A36582.pulse_counter_fast > ARC_COUNTER_SLOW_DECREMENT_INTERVAL) {
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
  if (_STATUS_HIGH_ENERGY) {
    // DPARKER do these checks 
  } else {

  }
	
  ETMCanLogCustomPacketC();  // This is the data log packet that contains the data for the previous pulse
  
}


void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
  /*
    A sample trigger has been received
  */ 
  
  // DPARKER delay until we are in the middle of the current pulse to sample
  Nop();
  Nop();
  Nop();
  Nop();

  // Trigger the internal ADC to start conversion
  _SAMP = 0;
  
  // Trigger the external ADC to start conversion
  PIN_ADC_CONVERT = OLL_ADC_START_CONVERSION;

  global_data_A36582.sample_energy_mode = _STATUS_HIGH_ENERGY;
  global_data_A36582.sample_index = etm_can_next_pulse_count;
  global_data_A36582.sample_complete = 1;

  // Check that there was enough time between pulses
  if ((TMR4 < MINIMUM_PULSE_PERIOD_T4) && !_T4IF) {
    global_data_A36582.minimum_pulse_period_fault_count++;
  }
  _T4IF = 0;
  TMR4 = 0;

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



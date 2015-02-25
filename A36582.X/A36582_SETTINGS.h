#ifndef __A36582_SETTINGS
#define __A36582_SETTINGS

#define PWR_5V_OVER_FLT        5200                   // 5.2 V
#define PWR_5V_UNDER_FLT       4800                   // 4.8 V

#define LED_STARTUP_FLASH_TIME    400                 // 4 Seconds

// The fast arc counter will shutdown with more than 50 arcs in 800 pulses
#define ARC_COUNTER_FAST_PERIOD                   800
#define ARC_COUNTER_FAST_MAX_ARCS                 50
#define ARC_COUNTER_FAST_DECREMENT_INTERVAL       (ARC_COUNTER_FAST_PERIOD / ARC_COUNTER_FAST_MAX_ARCS)


// The slow arc counter will shutdown with more than 100 arcs in 24000 pulses
#define ARC_COUNTER_SLOW_PERIOD                   24000
#define ARC_COUNTER_SLOW_MAX_ARCS                 100 
#define ARC_COUNTER_SLOW_DECREMENT_INTERVAL       (ARC_COUNTER_SLOW_PERIOD / ARC_COUNTER_SLOW_MAX_ARCS)



// The consecutive arc counter will shut down with more than 15 consecutive arcs
#define ARC_COUNTER_CONSECUTIVE_MAX               15

// False Trigger Configuration
#define FALSE_TRIGGER_DECREMENT_10_MS_UNITS       100 // The false trigger counter is decremented once per second
#define FALSE_TRIGGERS_FAULT_LEVEL                10  // The counter must reach 10 to trigger a fault.



#endif

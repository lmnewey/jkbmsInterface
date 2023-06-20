// Extracted from vairous examples and the JKBMS library in ESPHome
// I just wanted a standalone library to work on a victron integration so used 
// two of the esp home examples and one from STM32. You will have to google
// esphome JKBMS as I dont recall the source of the other examples.


#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include <stdint.h>

#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
// available variables are:
// Not quit all of them are implemented but most are
//   float power_tube_temperature_sensor_ ;
//  float temperature_sensor_1_sensor_ ;
//   float temperature_sensor_2_sensor_ ;
//   float total_voltage ;
//    float current_sensor_ ;
//    float current ;
//   float power ;
//   uint8_t raw_battery_remaining_capacity ;
//   float capacity_remaining_sensor_ ;
//   float temperature_sensors_sensor_ ;
//   float charging_cycles_sensor_ ;
//   float total_charging_cycle_capacity_sensor_ ;
//   float battery_strings_sensor_ ;
//   uint16_t raw_errors_bitmask ;
//   float errors_bitmask_sensor_ ;
//   std::string errors_text_sensor_ ;
//   float total_voltage_overvoltage_protection_sensor_ ;
//   float total_voltage_undervoltage_protection_sensor_ ;
//   float cell_voltage_overvoltage_protection_sensor_ ;
//   float cell_voltage_overvoltage_recovery_sensor_;
//   float cell_voltage_overvoltage_delay_sensor_ ;
//   float cell_voltage_undervoltage_protection_sensor_ ;
//   float cell_voltage_undervoltage_recovery_sensor_ ;
//   float cell_voltage_undervoltage_delay_sensor_ ;
//   float cell_pressure_difference_protection_sensor_ ;
//   float discharging_overcurrent_protection_sensor_ ;
//   float discharging_overcurrent_delay_sensor_ ;
//   float charging_overcurrent_protection_sensor_ ;
//   float charging_overcurrent_delay_sensor_ ;
//   float balance_starting_voltage_sensor_ ;
//   float balance_opening_pressure_difference_sensor_ ;
//   bool balancing_switch_binary_sensor_ ;
//   float power_tube_temperature_protection_sensor_ ;
//   float power_tube_temperature_recovery_sensor_ ;
//   float temperature_sensor_temperature_protection_sensor_ ;
//   float temperature_sensor_temperature_recovery_sensor_ ;
//   float temperature_sensor_temperature_difference_protection_sensor_ ;
//   float charging_high_temperature_protection_sensor_ ;
//   float discharging_high_temperature_protection_sensor_ ;
//   float charging_low_temperature_protection_sensor_ ;
//   float charging_low_temperature_recovery_sensor_ ;
//   float discharging_low_temperature_protection_sensor_ ;
//   float discharging_low_temperature_recovery_sensor_ ;
//   uint32_t raw_total_battery_capacity_setting ;
//   float total_battery_capacity_setting_sensor_ ;
//   float capacity_remaining_derived_sensor_ ;
//   bool charging_switch_binary_sensor_ ;
//   bool discharging_switch_binary_sensor_ ;
//   float current_calibration_sensor_ ;
//   float device_address_sensor_ ;
//   uint8_t raw_battery_type ;
//   std::string battery_type_text_sensor_ ;    
//   float sleep_wait_time_sensor_ ;
//   float alarm_low_volume_sensor_ ;
//   std::string password_text_sensor_ ;
//   bool dedicated_charger_switch_binary_sensor_ ;
//   std::string device_type_text_sensor_ ;
//   float total_runtime_sensor_ ;
//   std::string software_version_text_sensor_ ;
//   float actual_battery_capacity_sensor_ ;
//   std::string manufacturer_text_sensor_ ;
//   float protocol_version_sensor_ ;

// Include the JKBMS library
#include <jkbmsInterface.h>
	
// create an object of the JKBMS class
JKBMS jkbms ;

// Using this for CPP, not sure what the appropriate way to do this is
int main() {
    // tell it to start its polling task - this is a non-blocking call, you may want (or i might), make the polling interval adjustable
    jkbms.start();
	while(1){    
        // Send the request for data
 	    jkbms.Request_JK_Battery_485_Status_Frame();

        // wait a time for the data to be received
 	    vTaskDelay(pdMS_TO_TICKS(10000)); // it will be received well before this delay is up, but this is a good value to use for testing

        // print some of the values to the what evers 
        printf("Remaining Capacity %.2f\n",jkbms.capacity_remaining_derived_sensor_); // This returns a percentage of AH remaining 
    }  

  return 0;
}


// im new to C / C++ interoperability so im not sure why this needs to be here, YOLO
extern "C" void app_main(void)
{
main();
}

	

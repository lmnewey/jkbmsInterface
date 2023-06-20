



/**
 * JKBMS.h
 * 
 * JKBMS interface, a library for the ESP32 platform
 * Heavily plaguarized from the ESP home library and some 
 * from an STM32 repo. 
 * 
 * Pins used GPIO 16 for TX, 17 for RX and a GND
 * the plug on the JKBMS is the RS485 plug
 * it is a 1.25 pitch JST connector
 * White wire to TX, Red wire to RX, Black wire to GND. Pins are 1 - GND, 2 - TX, 3 - rX, VCC (this states bat v which is potentially 50+v so be careful[not tested])
 * 
 * @author Creator lmnewey 
 * @version 0.0.0
 * @license MIT
 */

 #ifndef JKBMS_h
 #define JKBMS_h



// #ifdef __cplusplus
// extern "C" {
//#endif

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include <stdint.h>
#include <vector>
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <iostream>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>




class JKBMS {
public:
//bool hasJKData();
void start();
void Request_JK_Battery_485_Status_Frame();

float power_tube_temperature_sensor_ ;
 float temperature_sensor_1_sensor_ ;
  float temperature_sensor_2_sensor_ ;
  float total_voltage ;
   float current_sensor_ ;
   float current ;
  float power ;
  uint8_t raw_battery_remaining_capacity ;
  float capacity_remaining_sensor_ ;
  float temperature_sensors_sensor_ ;
  float charging_cycles_sensor_ ;
  float total_charging_cycle_capacity_sensor_ ;
  float battery_strings_sensor_ ;
  uint16_t raw_errors_bitmask ;
  float errors_bitmask_sensor_ ;
  std::string errors_text_sensor_ ;
  float total_voltage_overvoltage_protection_sensor_ ;
  float total_voltage_undervoltage_protection_sensor_ ;
  float cell_voltage_overvoltage_protection_sensor_ ;
  float cell_voltage_overvoltage_recovery_sensor_;
  float cell_voltage_overvoltage_delay_sensor_ ;
  float cell_voltage_undervoltage_protection_sensor_ ;
  float cell_voltage_undervoltage_recovery_sensor_ ;
  float cell_voltage_undervoltage_delay_sensor_ ;
  float cell_pressure_difference_protection_sensor_ ;
  float discharging_overcurrent_protection_sensor_ ;
  float discharging_overcurrent_delay_sensor_ ;
  float charging_overcurrent_protection_sensor_ ;
  float charging_overcurrent_delay_sensor_ ;
  float balance_starting_voltage_sensor_ ;
  float balance_opening_pressure_difference_sensor_ ;
  bool balancing_switch_binary_sensor_ ;
  float power_tube_temperature_protection_sensor_ ;
  float power_tube_temperature_recovery_sensor_ ;
  float temperature_sensor_temperature_protection_sensor_ ;
  float temperature_sensor_temperature_recovery_sensor_ ;
  float temperature_sensor_temperature_difference_protection_sensor_ ;
  float charging_high_temperature_protection_sensor_ ;
  float discharging_high_temperature_protection_sensor_ ;
  float charging_low_temperature_protection_sensor_ ;
  float charging_low_temperature_recovery_sensor_ ;
  float discharging_low_temperature_protection_sensor_ ;
  float discharging_low_temperature_recovery_sensor_ ;
  uint32_t raw_total_battery_capacity_setting ;
  float total_battery_capacity_setting_sensor_ ;
  float capacity_remaining_derived_sensor_ ;
  bool charging_switch_binary_sensor_ ;
  bool discharging_switch_binary_sensor_ ;
  float current_calibration_sensor_ ;
  float device_address_sensor_ ;
  uint8_t raw_battery_type ;
  std::string battery_type_text_sensor_ ;    
  float sleep_wait_time_sensor_ ;
  float alarm_low_volume_sensor_ ;
  std::string password_text_sensor_ ;
  bool dedicated_charger_switch_binary_sensor_ ;
  std::string device_type_text_sensor_ ;
  float total_runtime_sensor_ ;
  std::string software_version_text_sensor_ ;
  float actual_battery_capacity_sensor_ ;
  std::string manufacturer_text_sensor_ ;
  float protocol_version_sensor_ ;
  int years;
  int days;
  int hours;


private:
int portNum;
int rx_buffer_size ;
uint16_t getCurrent(const uint16_t value);
float get_current_(const uint16_t value, const uint8_t protocol_version);
//void proceses_jk_modbus_Buffer();
int8_t getTemperature(const int16_t value);
float get_temperature_(const uint16_t value);
std::string mode_bits_to_string_(const uint16_t mask);
// uint16_t chksum(const uint8_t data[], const uint16_t len);
// uint16_t chksum( const uint16_t len, int iter);
std::string error_bits_to_string_(const uint16_t mask);
std::string format_total_runtime_(const uint32_t value);
bool parse_jk_modbus_byte_(int size, int iter);
void on_jk_modbus_data(const std::vector<uint8_t> data);
void on_jk_modbus_data(const uint8_t &function, const std::vector<uint8_t> &data);
void on_status_data_(const std::vector<uint8_t> &data);
};
// #ifdef __cplusplus
// }
// #endif

#endif /* WIFI_MANAGER_H_INCLUDED */



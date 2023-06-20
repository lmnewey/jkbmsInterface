#include <stdio.h>
#include "jkbmsInterface.h"


#include <stdio.h>
#include <string.h>
// Extracted from vairous examples and the JKBMS library in ESPHome
// I just wanted a standalone library to work on a victron integration so used 
// two of the esp home examples and one from STM32. You will have to google
// esphome JKBMS as I dont recall the source of the other examples.


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


#include <wifi_manager.h>

#include <JKBMS.h>
 

 // set this to be the same as the application
#define UART_PORT_NUM UART_NUM_0
#define UART_NUM UART_NUM_0

#define RX_BUFFER_SIZE 1024
int portNum = UART_PORT_NUM;
int rx_buffer_size = RX_BUFFER_SIZE;
QueueHandle_t uart_queue;
#define TX_PIN 16
#define RX_PIN 17
#define BAUD_RATE 115200
const int buffsize = 400;



// #define rx_bufferSIZE 1024

std::vector<uint8_t> rx_buffer;
static const char* TAG = "JKBMS ModBus/TTL Interface";


//Plaguraization starts here, this came largely from ESPHOME's implementation

uint16_t offset ;
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

static const uint8_t FUNCTION_READ_ALL = 0x06;
static const uint8_t ADDRESS_READ_ALL = 0x00;
static const uint8_t WRITE_REGISTER = 0x02;

static const uint8_t ERRORS_SIZE = 14;
static const char *const ERRORS[ERRORS_SIZE] = {
    "Low capacity",                              // Byte 0.0, warning
    "Power tube overtemperature",                // Byte 0.1, alarm
    "Charging overvoltage",                      // Byte 0.2, alarm
    "Discharging undervoltage",                  // Byte 0.3, alarm
    "Battery over temperature",                  // Byte 0.4, alarm
    "Charging overcurrent",                      // Byte 0.5, alarm
    "Discharging overcurrent",                   // Byte 0.6, alarm
    "Cell pressure difference",                  // Byte 0.7, alarm
    "Overtemperature alarm in the battery box",  // Byte 1.0, alarm
    "Battery low temperature",                   // Byte 1.1, alarm
    "Cell overvoltage",                          // Byte 1.2, alarm
    "Cell undervoltage",                         // Byte 1.3, alarm
    "309_A protection",                          // Byte 1.4, alarm
    "309_A protection",                          // Byte 1.5, alarm
};


static const uint8_t BATTERY_TYPES_SIZE = 3;
static const char *const BATTERY_TYPES[BATTERY_TYPES_SIZE] = {
    "Lithium Iron Phosphate",  // 0x00
    "Ternary Lithium",         // 0x01
    "Lithium Titanate",        // 0x02
};

  float get_temperature(const uint16_t value) {
    if (value > 100)
      return (float) (100 - (int16_t) value);

    return (float) value;
  };

  

  float get_current(const uint16_t value, const uint8_t protocol_version) {
    float current = 0.0f;
    if (protocol_version == 0x01) {
      if ((value & 0x8000) == 0x8000) {
        current = (float) (value & 0x7FFF);
      } else {
        current = (float) (value & 0x7FFF) * -1;
      }
    }

    return current;
  };



// this is not implemented yet, its apart of the copy past from ESPHOME
  std::string JKBMS::format_total_runtime_(const uint32_t value) {
    int seconds = (int) value;
    years = seconds / (24 * 3600 * 365);
    seconds = seconds % (24 * 3600 * 365);
    days = seconds / (24 * 3600);
    seconds = seconds % (24 * 3600);
    hours = seconds / 3600;
    return "Not Implemented";
    // return (years ? to_string(years) + "y " : "") + (days ? to_string(days) + "d " : "") +
    //        (hours ? to_string(hours) + "h" : "");
  }

std::string error_bits_to_string(const uint16_t mask) {
  bool first = true;
  std::string errors_list = "";

  if (mask) {
    for (int i = 0; i < ERRORS_SIZE; i++) {
      if (mask & (1 << i)) {
        if (first) {
          first = false;
        } else {
          errors_list.append(";");
        }
        errors_list.append(ERRORS[i]);
      }
    }
  }
   return errors_list;
}


// typedef enum  {
// 	v0 = 0,
// 	v1 = 1
// } bms_type;

struct jk_bms_cell_voltage {
	uint8_t cell_number;
	uint16_t cell_voltage;
};

struct jk_bms_limits {
	uint16_t battery_charge_voltage;
	uint16_t battery_charge_current_limit;
	uint16_t battery_discharge_current_limit;
	uint16_t battery_discharge_voltage;
};

static const uint8_t OPERATION_MODES_SIZE = 4;
static const char *const OPERATION_MODES[OPERATION_MODES_SIZE] = {
    "Charging enabled",     // 0x00
    "Discharging enabled",  // 0x01
    "Balancer enabled",     // 0x02
    "Battery dropped",      // 0x03
};


struct jk_bms_alarms {
	uint16_t alarm_data;
	bool low_capacity;
	bool power_tube_overtemperature;
	bool charging_overvoltage;
	bool discharging_undervoltage;
	bool battery_over_temperature;
	bool charging_overcurrent;
	bool discharging_overcurrent;
	bool cell_pressure_difference;
	bool overtemperature_alarm_battery_box;
	bool battery_low_temperature;
	bool cell_overvoltage;
	bool cell_undervoltage;
	bool a_protection_309_1;
	bool a_protection_309_2;
};

struct jk_bms_battery_status {
	int8_t power_tube_temperature;
	int8_t sensor_temperature_1;
	int8_t sensor_temperature_2;
	int8_t temperature_sensor_count;
	uint16_t battery_voltage;
	int16_t battery_current;
	uint8_t battery_soc;
	uint16_t battery_cycles;
	uint32_t battery_cycle_capacity;

	uint8_t current_low_byte;
	uint8_t current_hi_byte;
};

struct jk_bms_battery_info {
	uint8_t cells_number;
	struct jk_bms_cell_voltage cells_voltage[24];
	struct jk_bms_battery_status battery_status;
	struct jk_bms_alarms battery_alarms;
	struct jk_bms_limits battery_limits;
};

extern struct jk_bms_battery_info jk_bms_battery_info;
struct jk_bms_battery_info jk_bms_battery_info;



uint16_t chksum(const uint8_t data[], const uint16_t len) {
  uint16_t checksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    checksum = checksum + data[i];
  }
  return checksum;
}

uint16_t chksum( const uint16_t len, int iter) {
  uint16_t checksum = 0;
  for (uint16_t i = iter; i < len; i++) {
    checksum = checksum + rx_buffer[i];
  }
  return checksum;
}


std::string JKBMS::mode_bits_to_string_(const uint16_t mask) {
  bool first = true;
  std::string modes_list = "";

  if (mask) {
    for (int i = 0; i < OPERATION_MODES_SIZE; i++) {
      if (mask & (1 << i)) {
        if (first) {
          first = false;
        } else {
          modes_list.append(";");
        }
        modes_list.append(OPERATION_MODES[i]);
      }
    }
  }

  return modes_list;
}



void JKBMS::Request_JK_Battery_485_Status_Frame(){
  //bms_type type = v0;
  uint8_t frame[21];
 frame[0] = 0x4E;      // start sequence
  frame[1] = 0x57;      // start sequence
  frame[2] = 0x00;      // data length lb
  frame[3] = 0x13;      // data length hb
  frame[4] = 0x00;      // bms terminal number
  frame[5] = 0x00;      // bms terminal number
  frame[6] = 0x00;      // bms terminal number
  frame[7] = 0x00;      // bms terminal number
  frame[8] = 0x06;  // command word: 0x01 (activation), 0x02 (write), 0x03 (read), 0x05 (password), 0x06 (read all)
  frame[9] = 0x03;      // frame source: 0x00 (bms), 0x01 (bluetooth), 0x02 (gps), 0x03 (computer)
  frame[10] = 0x00;     // frame type: 0x00 (read data), 0x01 (reply frame), 0x02 (BMS active upload)
  frame[11] = 0x00;  // register: 0x00 (read all registers), 0x8E...0xBF (holding registers)
  frame[12] = 0x00;     // record number
  frame[13] = 0x00;     // record number
  frame[14] = 0x00;     // record number
  frame[15] = 0x00;     // record number
  frame[16] = 0x68;     // end sequence
  uint16_t crc = chksum(frame, 17);
  frame[17] = 0x00;  // crc unused
  frame[18] = 0x00;  // crc unused
  frame[19] = crc >> 8;//0x01;//crc >> 8;
  frame[20] = crc >> 0;//0x29;//crc >> 0;
  uart_write_bytes(portNum, frame, 21);
 printf("Request Sent via port: %i\n", portNum);
}

void on_status_data_(const std::vector<uint8_t> &data) {
  auto jk_get_16bit = [&](size_t i) -> uint16_t { return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0); };
  auto jk_get_32bit = [&](size_t i) -> uint32_t {return (uint32_t(jk_get_16bit(i + 0)) << 16) | (uint32_t(jk_get_16bit(i + 2)) << 0);
  };
}


void on_jk_modbus_data(const uint8_t &function, const std::vector<uint8_t> &data) {
  if (function == FUNCTION_READ_ALL) {
    on_status_data_(data);
    return;
  }

  ESP_LOGW(TAG, "Invalid size (%zu) for JK BMS frame!", data.size());
}


void on_jk_modbus_data(const std::vector<uint8_t> data)
{
  auto jk_get_16bit = [&](size_t i) -> uint16_t { return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0); };
  auto jk_get_32bit = [&](size_t i) -> uint32_t {return (uint32_t(jk_get_16bit(i + 0)) << 16) | (uint32_t(jk_get_16bit(i + 2)) << 0);};

  uint8_t cells = data[1] / 3;
  //float cellsv[cells] = {};
  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  float average_cell_voltage = 0.0f;
  uint8_t min_voltage_cell = 0;
  uint8_t max_voltage_cell = 0;
  for (uint8_t i = 0; i < cells; i++) {
    
    float cell_voltage = jk_get_16bit(i * 3 + 3) * 0.001f;
    average_cell_voltage = average_cell_voltage + cell_voltage;
    if (cell_voltage < min_cell_voltage) {
      min_cell_voltage = cell_voltage;
      min_voltage_cell = i + 1;
    }
    if (cell_voltage > max_cell_voltage) {
      max_cell_voltage = cell_voltage;
      max_voltage_cell = i + 1;
    }
     //cellsv[i] = cell_voltage;
     printf("Cell %i, ", i);
     printf("V:%.6f \n",  cell_voltage);
  }
  offset = data[1] + 3;
  average_cell_voltage = average_cell_voltage / cells;
  power_tube_temperature_sensor_ = get_temperature(jk_get_16bit(offset + 3 * 0));
  temperature_sensor_1_sensor_ = get_temperature(jk_get_16bit(offset + 3 * 1));
  temperature_sensor_2_sensor_ =  get_temperature(jk_get_16bit(offset + 3 * 2));
  total_voltage = jk_get_16bit(offset + 3 * 3) * 0.01f;
  //printf("Pack Voltage: %.3f \n", total_voltage);
   current_sensor_ = get_current(jk_get_16bit(offset + 3 * 4), 0x01) * 0.01f;
   current = get_current(jk_get_16bit(offset + 3 * 4), data[offset + 84 + 3 * 45]) * 0.01f;
    power = total_voltage * current;
  //printf("Current Power: %.2f \n", power);
  raw_battery_remaining_capacity = data[offset + 3 * 5];
  capacity_remaining_sensor_ = (float) raw_battery_remaining_capacity;
  //printf("AH Remaining: %i \n", raw_battery_remaining_capacity);
  temperature_sensors_sensor_ = (float) data[offset + 2 + 3 * 5];
  charging_cycles_sensor_ = jk_get_16bit(offset + 4 + 3 * 5);
  total_charging_cycle_capacity_sensor_ = jk_get_32bit(offset + 4 + 3 * 6);
  battery_strings_sensor_ = jk_get_16bit(offset + 6 + 3 * 7);
  raw_errors_bitmask = jk_get_16bit(offset + 6 + 3 * 8);
  errors_bitmask_sensor_ = (float) raw_errors_bitmask;
  errors_text_sensor_ = error_bits_to_string(raw_errors_bitmask);
  total_voltage_overvoltage_protection_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 10) * 0.01f;
  total_voltage_undervoltage_protection_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 11) * 0.01f;
  cell_voltage_overvoltage_protection_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 12) * 0.001f;
  cell_voltage_overvoltage_recovery_sensor_= (float) jk_get_16bit(offset + 6 + 3 * 13) * 0.001f;
  cell_voltage_overvoltage_delay_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 14);
  cell_voltage_undervoltage_protection_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 15) * 0.001f;
  cell_voltage_undervoltage_recovery_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 16) * 0.001f;
  cell_voltage_undervoltage_delay_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 17);
  cell_pressure_difference_protection_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 18) * 0.001f;
  discharging_overcurrent_protection_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 19);
  discharging_overcurrent_delay_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 20);
  charging_overcurrent_protection_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 21);
  charging_overcurrent_delay_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 22);
  balance_starting_voltage_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 23) * 0.001f;
  balance_opening_pressure_difference_sensor_ = (float) jk_get_16bit(offset + 6 + 3 * 24) * 0.001f;
  balancing_switch_binary_sensor_ = (bool) data[offset + 6 + 3 * 25];
  power_tube_temperature_protection_sensor_ = (float) jk_get_16bit(offset + 8 + 3 * 25);
  power_tube_temperature_recovery_sensor_ = (float) jk_get_16bit(offset + 8 + 3 * 26);
  temperature_sensor_temperature_protection_sensor_ = (float) jk_get_16bit(offset + 8 + 3 * 27);
  temperature_sensor_temperature_recovery_sensor_ = (float) jk_get_16bit(offset + 8 + 3 * 28);
  temperature_sensor_temperature_difference_protection_sensor_ = (float) jk_get_16bit(offset + 8 + 3 * 29);
  charging_high_temperature_protection_sensor_ = (float) jk_get_16bit(offset + 8 + 3 * 30);
  discharging_high_temperature_protection_sensor_ = (float) jk_get_16bit(offset + 8 + 3 * 31);
  charging_low_temperature_protection_sensor_ = (float) (int16_t) jk_get_16bit(offset + 8 + 3 * 32);
  charging_low_temperature_recovery_sensor_ =(float) (int16_t) jk_get_16bit(offset + 8 + 3 * 33);
  discharging_low_temperature_protection_sensor_ =(float) (int16_t) jk_get_16bit(offset + 8 + 3 * 34);
  discharging_low_temperature_recovery_sensor_ =(float) (int16_t) jk_get_16bit(offset + 8 + 3 * 35);
  raw_total_battery_capacity_setting = jk_get_32bit(offset + 10 + 3 * 36);
  total_battery_capacity_setting_sensor_ = (float) raw_total_battery_capacity_setting;
  capacity_remaining_derived_sensor_ =  (float) (raw_total_battery_capacity_setting * (raw_battery_remaining_capacity * 0.01f));
  charging_switch_binary_sensor_ = (bool) data[offset + 15 + 3 * 36];
  discharging_switch_binary_sensor_ = (bool) data[offset + 17 + 3 * 36];
  current_calibration_sensor_ = (float) jk_get_16bit(offset + 19 + 3 * 36) * 0.001f;
  device_address_sensor_ = (float) data[offset + 19 + 3 * 37];
  
  raw_battery_type = data[offset + 21 + 3 * 37];
  if (raw_battery_type < BATTERY_TYPES_SIZE) {
    battery_type_text_sensor_ = BATTERY_TYPES[raw_battery_type];
  } else {
    battery_type_text_sensor_ = "Unknown";
  }

  sleep_wait_time_sensor_ = (float) jk_get_16bit(offset + 23 + 3 * 37);
  alarm_low_volume_sensor_ = (float) data[offset + 23 + 3 * 38];
password_text_sensor_ =
                       std::string(data.begin() + offset + 25 + 3 * 38, data.begin() + offset + 35 + 3 * 38);
dedicated_charger_switch_binary_sensor_ = (bool) data[offset + 36 + 3 * 38];
device_type_text_sensor_ = std::string(data.begin() + offset + 38 + 3 * 38, data.begin() + offset + 46 + 3 * 38);
total_runtime_sensor_ = (float) jk_get_32bit(offset + 46 + 3 * 40) * 0.0166666666667;
//  std::string total_runtime_formatted_text_sensor_ =  format_total_runtime_(jk_get_32bit(offset + 46 + 3 * 40) * 60);

 software_version_text_sensor_ =
                       std::string(data.begin() + offset + 51 + 3 * 40, data.begin() + offset + 51 + 3 * 45);
actual_battery_capacity_sensor_ = (float) jk_get_32bit(offset + 54 + 3 * 45);
 manufacturer_text_sensor_ = std::string(data.begin() + offset + 59 + 3 * 45, data.begin() + offset + 83 + 3 * 45);

 protocol_version_sensor_ = (float) data[offset + 84 + 3 * 45];

}

bool parse_jk_modbus_byte_(int size, int iter) {
  size_t at = rx_buffer.size();
  

  // Byte 2: Size (low byte)
  if (at == iter+2)
    return true;

  // Byte 3: Size (high byte)
  if (at == iter+3)
    return true;

  uint16_t data_len = (uint16_t(rx_buffer[2]) << 8 | (uint16_t(rx_buffer[2 + 1]) << 0));

  // data_len: CRC_LO (over all bytes)
  if (at <= data_len)
    return true;

  uint8_t function = rx_buffer[iter+8];

  // data_len+1: CRC_HI (over all bytes)
  uint16_t computed_crc = chksum(data_len, iter); // Commented out after i changed the function call
  uint16_t remote_crc = uint16_t(rx_buffer[data_len]) << 8 | (uint16_t(rx_buffer[data_len + 1]) << 0);
  if (computed_crc != remote_crc) {
    ESP_LOGW(TAG, "JkModbus CRC Check failed! %02X!=%02X", computed_crc, remote_crc);
    return false;
  }
  else{
      ESP_LOGW(TAG, "JkModbus CRC Check Success! %02X!=%02X", computed_crc, remote_crc);

  }

   std::vector<uint8_t> data(rx_buffer.begin() + (11+iter), rx_buffer.begin() + data_len - (3+iter));
   //std::vector<uint8_t> data(this->rx_buffer_.begin() + 11, this->rx_buffer_.begin() + data_len - 3);
    on_jk_modbus_data(data);

   return false;
}


bool JKBMS::parse_jk_modbus_byte_(int size, int iter) {
  size_t at = rx_buffer.size();
  

  // Byte 2: Size (low byte)
  if (at == iter+2)
    return true;

  // Byte 3: Size (high byte)
  if (at == iter+3)
    return true;

  uint16_t data_len = (uint16_t(rx_buffer[2]) << 8 | (uint16_t(rx_buffer[2 + 1]) << 0));

  
  if (at <= data_len)
    return true;

  
  uint16_t computed_crc = chksum(data_len, iter); // Commented out after i changed the function call
  uint16_t remote_crc = uint16_t(rx_buffer[data_len]) << 8 | (uint16_t(rx_buffer[data_len + 1]) << 0);
  if (computed_crc != remote_crc) {
    ESP_LOGW(TAG, "JkModbus CRC Check failed! %02X!=%02X", computed_crc, remote_crc);
    return false;
  }
  else{
      ESP_LOGW(TAG, "JkModbus CRC Check Success! %02X!=%02X", computed_crc, remote_crc);
  }
   std::vector<uint8_t> data(rx_buffer.begin() + (11+iter), rx_buffer.begin() + data_len - (3+iter));
    on_jk_modbus_data(data);
   return false;
}


void proceses_jk_modbus_Buffer()
{
	int rsize = rx_buffer.size();
  printf("Buffer size at receive time:%i \n", rsize );
  
	for(int i = 0; i< rsize; i++)
			if(i <= rsize-1){
			if( rx_buffer[i] != 0x4E || rx_buffer[i+1] != 0x57) {	
			// invalid header call if you feel you need it
			}
			else
			{
				
				parse_jk_modbus_byte_(rsize, i);
			}

    }
  
}
	
void uartRxHandler(void *arg) {
    uart_event_t event;
    size_t bufferedSize;

    // Get the UART RX event and the buffered data size
    if (uart_driver_install(portNum, rx_buffer_size, rx_buffer_size, 10, &uart_queue, 0) == ESP_OK) {
        while (1) {
          
            if (xQueueReceive(uart_queue, (void *)&event, (portTickType)portMAX_DELAY)) {	
                 if (event.type == UART_DATA) {
                  
                    uart_get_buffered_data_len(portNum, &bufferedSize);
                    std::vector<uint8_t> tempBuffer(bufferedSize);

                    // Read data from UART FIFO
                    uart_read_bytes(portNum, tempBuffer.data(), bufferedSize, portMAX_DELAY);

                    // Append data to the global buffer
                    rx_buffer.insert(rx_buffer.end(), tempBuffer.begin(), tempBuffer.end());
                    
                }                
            }
        }
    }
}				
				
void JKBMS::start(){
  uart_config_t uartConfig;
    uartConfig.baud_rate = 115200;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(portNum, &uartConfig);
    uart_set_pin(portNum, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Create a task to handle UART events
    printf("Create Serial Task\n");
    xTaskCreate(uartRxHandler, "uart_rx_handler", 4096, NULL, 10, NULL);


vTaskDelay(pdMS_TO_TICKS(250));


 while(1){    
  Request_JK_Battery_485_Status_Frame();
  vTaskDelay(pdMS_TO_TICKS(250)); 
   if(rx_buffer.size() > 50)
   	{    
		proceses_jk_modbus_Buffer();		
		rx_buffer.clear();
	}

  } 
}





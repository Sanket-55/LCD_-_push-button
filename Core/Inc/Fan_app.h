#include <stdint.h>
#define MAX_DISPLAY_SIZE 16
#define NUMBER_OF_SENSOR 3
#define NUMBER_OF_SETTING 12
#define MAX_PASSWORD 100
#define MIN_PASSWORD 0
#define PASSWORD_INDEX 11
#define MAX_TEMP_THRESHOLD 100
#define MIN_TEMP_THRESHOLD 0
#define MAX_DUTY_CYCLE 100
#define MIN_DUTY_CYCLE 0
#define MAX_FAN_FAIL_DC 100
#define MIN_FAN_FAIL_DC 0
void display_sensor_config(uint8_t position);
void display_sensor(uint8_t position);






typedef struct
		{
			char display_Setting[MAX_DISPLAY_SIZE];
			uint8_t is_bool;
			uint8_t setting_value;
			uint8_t max_value;
			uint8_t min_value;
		}Setting_List;


typedef struct
		{
			char display_Sensor[MAX_DISPLAY_SIZE];
			uint8_t Sensor_Value;
		}Sensor_List;

#include "Fan_app.h"
#include "button.h"
#include "LCD.h"

Sensor_List DisplaySensorList[NUMBER_OF_SENSOR] = {
		{"TEMP_SR = ",0},
		{"AMB_sr1 = ",0},
		{"AMB_sr2 = ",0}
};
Setting_List DisplaySettingList[NUMBER_OF_SETTING] = {
		 {"ENTER_PASSWORD =", 0, 0,MAX_PASSWORD, MIN_PASSWORD},
		 {"TEMP_THR = ", 0, 0, MAX_TEMP_THRESHOLD, MIN_TEMP_THRESHOLD},
		 {"MX_DC = ", 0, 0, MAX_DUTY_CYCLE, MIN_DUTY_CYCLE},
		 {"FAN_FAIL_DC = ", 0, 50,MAX_FAN_FAIL_DC, MIN_FAN_FAIL_DC},
		 {"SMOKE_ALRM = ", 1, 1},
		 {"DOOR_ALRM = ", 1, 1},
		 {"HIGHTEMP_ALRM =", 1, 0},
		 {"FANFAIL_ALRM =", 1, 0},
		 {"IN_TEMP_FAIL =", 1, 0},
		 {"AMB1_FAIL_ALRM =", 1, 0},
		 {"AMB2_FAIL_ALRM =", 1, 0},
		 {"PASS_CHANGE =", 0, 50,MAX_PASSWORD, MIN_PASSWORD}


};

uint8_t Temp_Value;
uint8_t sensor_position;
uint8_t setting_position;
uint8_t inner_depth;


void main_display(uint8_t button)
{
    switch (button)
    {
        case UP_BUTTON:
        {
            if (inner_depth == 3)
            {
            	if(DisplaySettingList[setting_position].is_bool)
            	{
            		DisplaySettingList[setting_position].setting_value = (!DisplaySettingList[setting_position].setting_value);
            	}
            	else
            	{
            		(DisplaySettingList[setting_position].setting_value == DisplaySettingList[setting_position].max_value)
            		? (DisplaySettingList[setting_position].setting_value = DisplaySettingList[setting_position].min_value)
            		: (DisplaySettingList[setting_position].setting_value++);
            	}
            	display_sensor_config(setting_position);
            }
            else if (inner_depth == 2)
            {
            	(setting_position == (NUMBER_OF_SETTING - 1)) ? (setting_position = 1): (setting_position++);
            	display_sensor_config(setting_position);
            }
            else if (inner_depth == 1)
            {
            	(DisplaySettingList[setting_position].setting_value == MAX_PASSWORD )? (DisplaySettingList[setting_position].setting_value = MIN_PASSWORD ):(DisplaySettingList[setting_position].setting_value ++);
            	display_sensor_config(setting_position);
            }
            else
            {
            	sensor_position++;
            	sensor_position = (sensor_position % NUMBER_OF_SENSOR);
                display_sensor(sensor_position);
            }
            break;
        }
        case DOWN_BUTTON:
        {
            if (inner_depth == 3)
            {
            	if(DisplaySettingList[setting_position].is_bool)
            	{
            		DisplaySettingList[setting_position].setting_value = (!DisplaySettingList[setting_position].setting_value);
            	}
            	else
            	{
            		(DisplaySettingList[setting_position].setting_value == DisplaySettingList[setting_position].min_value)
            		? (DisplaySettingList[setting_position].setting_value = DisplaySettingList[setting_position].max_value)
            		: (DisplaySettingList[setting_position].setting_value--);
            	}
            	display_sensor_config(setting_position);
            }
            else if (inner_depth == 2)
            {
            	(setting_position == 1) ? (NUMBER_OF_SETTING - 1): (setting_position--);
            	display_sensor_config(setting_position);
            }
            else if (inner_depth == 1)
            {
            	(DisplaySettingList[setting_position].setting_value == MIN_PASSWORD )? (DisplaySettingList[setting_position].setting_value = MAX_PASSWORD ):(DisplaySettingList[setting_position].setting_value --);
            	display_sensor_config(setting_position);
            }
            else {
            	sensor_position--;
            	sensor_position = (sensor_position + NUMBER_OF_SENSOR) % NUMBER_OF_SENSOR; // Handle wrap-around correctly
            	display_sensor_config(sensor_position);
            }
            break;
        }
        case ENTER_BUTTON:
        {
            if (inner_depth == 3)
            {
            	inner_depth --;
            	display_sensor_config(setting_position);
            }
            else if (inner_depth == 2)
            {
            	inner_depth++;
            	Temp_Value = DisplaySettingList[setting_position].setting_value;
            	display_sensor_config(setting_position);
            }
            else if (inner_depth == 1) {
            	if(DisplaySettingList[setting_position].setting_value == DisplaySettingList[PASSWORD_INDEX].setting_value)
            	{
            		DisplaySettingList[setting_position].setting_value =0;
            		setting_position++;
            		inner_depth++;
            		display_sensor_config(setting_position);
            	}
            	else
            	{
            		display_sensor_config(setting_position);
            	}
            }
            else
            {
            	setting_position=0;
            	inner_depth++;
            	display_sensor_config(setting_position);
            }


            break;
        }
        case BACK_BUTTON:
        {
            if (inner_depth == 3)
            {
            	DisplaySettingList[setting_position].setting_value = Temp_Value;
            	inner_depth --;
            	display_sensor_config(setting_position);
            }
            else if (inner_depth == 2)
            {
            	inner_depth = 0;
            	setting_position=0;
            	display_sensor(sensor_position);

                        }
            else if (inner_depth == 1)
            {
            	inner_depth--;
            	display_sensor(sensor_position);

            }
            else
            {
                display_sensor(sensor_position);
            }
            break;
        }
    }
}

void display_sensor_config(uint8_t position) {
	lcd_clear();
	lcd_send_string(DisplaySettingList[position].display_Setting);
	lcd_put_cur(1, 0);

	    /* Second line print sensor value */
	lcd_send_data(DisplaySettingList[position].setting_value);
}

void display_sensor(uint8_t position) {
    /* First line print sensor name */
    lcd_clear();
    lcd_send_string(DisplaySensorList[position].display_Sensor);
    lcd_put_cur(1, 0);

    /* Second line print sensor value */
    lcd_send_data(DisplaySensorList[position].Sensor_Value);
}

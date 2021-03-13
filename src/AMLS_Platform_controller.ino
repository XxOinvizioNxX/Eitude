/*
 * Copyright 2021 The AMLS Platform controller Open Source Project
 * This software is part of Autonomous Multirotor Landing System (AMLS) Project
 *
 * Licensed under the GNU Affero General Public License, Version 3.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.gnu.org/licenses/agpl-3.0.en.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* 
 * Libraries
*/
#include <Wire.h>
#include <LiquidCrystal.h>

/*
 * Pins
*/
#define LIGHT_S_1_PIN			A0
#define LIGHT_S_2_PIN			A1
#define LDC_RS_PIN				2
#define LDC_E_PIN				3
#define LCD_D4_PIN				4
#define LCD_D5_PIN				5
#define LCD_D6_PIN				6
#define LCD_D7_PIN				7
#define LIGHTS_PIN				9

/*
 * Setup
*/
#define MPU6050_ADDRESS			0x68
#define CALIBRATION_N			1000
#define ACC_FILTER_KOEFF		0.970
#define BAUD_RATE				57600
#define BUFFER_SIZE				512
#define LUX_FILTER_KOEFF		0.950
#define LCD_PRINT_TIME			500
//#define MAX_LUX				100000

/*
 * System variables
*/

// LCD
LiquidCrystal lcd(LDC_RS_PIN, LDC_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
uint64_t lcd_timer;

// Serial
char buffer[BUFFER_SIZE];
int16_t sofar;
int16_t command;

// Light sensor
uint16_t raw_illumination;
float filtered_value;

// MPU6050
volatile int16_t temperature;
volatile int16_t acc_x, acc_y, acc_z;
volatile int16_t gyro_pitch, gyro_roll, gyro_yaw;
uint8_t level_calibration_on;
int32_t gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal;
int32_t acc_x_cal_value, acc_y_cal_value;
int32_t acc_x_filtered, acc_y_filtered;
float speed, speed_accumulator;
int64_t speed_loop_timer;

/*
 * Initializes the system
*/
void setup()
{
	// Define pins mode
	pinMode(LIGHTS_PIN, OUTPUT);
	analogReference(EXTERNAL);

	// Open serial port
	Serial.begin(BAUD_RATE);
	delay(200);

	// Initialize I2C
	Wire.begin();

	// Initialize 16x2 display
	lcd.begin(16, 2);
	lcd.print(F(" <-  Eitude  -> "));
	lcd.setCursor(0, 1);
	lcd.print(F("Calibration ... "));

	// Setup MPU6050
	gyro_setup();

	// Done calibration
	lcd.clear();
	Serial.print(F(">"));
}

/*
 * Main loop
*/
void loop()
{
	// Calculate LUX
	illumination_handler();

	// Calculate speed
	speed_handler();

	// Check serial
	fill_buffer();

	// Print current state to LCD every LCD_PRINT_TIME
	if (millis() - lcd_timer >= LCD_PRINT_TIME) {
		print_lux_to_lcd();
		lcd_timer = millis();
	}
	
	// Minimum loop time
	delayMicroseconds(4000);
}

/*
 * Reads from serial to buffer until new line
*/
void fill_buffer() {
	while (Serial.available() > 0) {
		char c = Serial.read();
		if (sofar < BUFFER_SIZE - 1) buffer[sofar++] = c;
		if ((c == '\n') || (c == '\r')) {
			buffer[sofar] = 0;

			// Check if buffer is not empty
			if (sofar < 2) {
				sofar = 0;
				break;
			}

			// Show buffer
			print_buffer_to_lcd();

			// Process current command
			process_command();

			// Send ready sign
			Serial.print(F(">"));

			// Reset buffer
			sofar = 0;
		}
	}
}

/*
 * Processes the current command
*/
void process_command() {
	// M - commands
	command = parse_gcode('M', -1);
	switch (command)
	{
	case 3:
		// Enable tool (LED)
		digitalWrite(LIGHTS_PIN, 1);
		break;
	case 5:
		// Disable tool (LED)
		digitalWrite(LIGHTS_PIN, 0);
		break;
	default:
		break;
	}

	// L - commands
	command = parse_gcode('L', -1);
	switch (command)
	{
	case 0:
		// Illumination check
		Serial.print(F("S0 L"));
		Serial.println((uint16_t)filtered_value);
		break;
	case 1:
		// Speed check
		Serial.print(F("S0 L"));
		Serial.println(speed_accumulator, 2);
		break;
	default:
		break;
	}
}

/*
 * Reads current illumination from ADC and converts to Lux
*/
void illumination_handler() {
	raw_illumination = analogRead(LIGHT_S_1_PIN) + analogRead(LIGHT_S_2_PIN);
	raw_illumination /= 2;
	filtered_value = filtered_value * LUX_FILTER_KOEFF + raw_illumination * (1.0 - LUX_FILTER_KOEFF);
}

/*
 * Prints current illumination and lights state to the screen
*/
void print_lux_to_lcd() {
	lcd.setCursor(0, 1);

	// Illumination
	lcd.print(F("ADC:"));
	lcd.print((uint16_t)filtered_value);

	// Speed
	lcd.print(F(" SPD:"));
	lcd.print(speed_accumulator, 1);

	// Fill all the line
	lcd.print(F("                "));
}

/*
 * Prints buffer to the display
*/
void print_buffer_to_lcd() {
	lcd.setCursor(0, 0);

	// Timestamp
	lcd.print(millis() / 1000);
	lcd.print(F(": "));

	// Current GCode
	for (size_t i = 0; i < sofar - 1; i++)
		lcd.print(buffer[i]);

	// Fill all the line
	lcd.print(F("                "));
}

/*
 * Parses GCode buffer (returns the value of the specified code)
*/
float parse_gcode(char code, float val) {
	char* ptr = buffer;
	while ((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer + sofar) {
		if (*ptr == code) {
			return atof(ptr + 1);
		}
		ptr = strchr(ptr, ' ') + 1;
	}
	return val;
}

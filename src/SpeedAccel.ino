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

void speed_handler(void) {
	gyro_signalen();

	// Filter accelerations
	acc_x_filtered = (float)acc_x_filtered * ACC_FILTER_KOEFF + (float)acc_x * (1.0 - ACC_FILTER_KOEFF);
	acc_y_filtered = (float)acc_y_filtered * ACC_FILTER_KOEFF + (float)acc_y * (1.0 - ACC_FILTER_KOEFF);

	speed = acc_x_filtered;

	// Convert acceleration to G
	speed /= 4096.0;
	// Convert to m/s^2
	speed *= 9.81;	
	// Multiply by dt to get instant speed in m/ms
	speed *= (millis() - speed_loop_timer);

	// Reset timer
	speed_loop_timer = millis();

	// Convert to m/s
	speed /= 1000.0;						
	// Convert to km/h
	speed *= 3.6;						
	
	// Accumulate instatnt speed
	speed_accumulator += speed;

	if (!in_move_flag) {
		// If the platform is not moving, reset the speed
		speed_accumulator = speed_accumulator * SPEED_ZEROING_FACTOR;
	}
}

void check_vibrations(void) {
	gyro_signalen();
	// Calculate the total accelerometer vector.
	vibration_array[0] = (int32_t)acc_x * (int32_t)acc_x;
	vibration_array[0] += (int32_t)acc_y * (int32_t)acc_y;
	vibration_array[0] += (int32_t)acc_z * (int32_t)acc_z;
	vibration_array[0] = sqrt(vibration_array[0]);

	for (i = 16; i > 0; i--) {
		// Shift every variable one position up in the array.
		vibration_array[i] = vibration_array[i - 1];
		// Add the array value to the acc_av_vector variable.
		avarage_vibration_level += vibration_array[i];
	}
	// Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.
	avarage_vibration_level /= 17;

	if (vibration_counter < 20) {
		vibration_counter++;
		// Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable.
		vibration_total_result += abs(vibration_array[0] - avarage_vibration_level);
		
	}
	else {
		in_move_flag = vibration_total_result > VIBR_STOP_MOVE_THRESH;
		vibration_counter = 0;
		// Serial.println(vibration_total_result);
		vibration_total_result = 0;
	}
}

void gyro_setup(void) {
	Wire.beginTransmission(MPU6050_ADDRESS);
	// Set the PWR_MGMT_1 register (6B hex) bits as 00000000 to activate the gyro.
	Wire.write(0x6B);
	Wire.write(0x00);							
	Wire.endTransmission();

	Wire.beginTransmission(MPU6050_ADDRESS);
	// Set the GYRO_CONFIG register (1B hex) bits as 00001000 (500dps full scale)
	Wire.write(0x1B);
	Wire.write(0x08);							
	Wire.endTransmission();

	Wire.beginTransmission(MPU6050_ADDRESS);
	// Set the  ACCEL_CONFIG register (1A hex) bits as 00010000 (+/- 8g full scale range)
	Wire.write(0x1C);
	Wire.write(0x10);
	Wire.endTransmission();

	Wire.beginTransmission(MPU6050_ADDRESS);
	// Set the CONFIG register (1A hex) bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	Wire.write(0x1A);
	Wire.write(0x03);
	Wire.endTransmission();
}

void gyro_calibration(void) {
	// Disable subtracting calibration values
	level_calibration_on = 1;
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("  Calibration   "));
	lcd.setCursor(0, 1);

	acc_x_cal_value = 0;
	acc_y_cal_value = 0;
	gyro_roll_cal = 0;
	gyro_pitch_cal = 0;
	gyro_yaw_cal = 0;
	for (i = 0; i < CALIBRATION_N; i++) {
		gyro_signalen();
		acc_x_cal_value += acc_x;
		acc_y_cal_value += acc_y;
		gyro_roll_cal += gyro_roll;
		gyro_pitch_cal += gyro_pitch;
		gyro_yaw_cal += gyro_yaw;

		if (i % (CALIBRATION_N / 16) == 0)
			lcd.print('.');
		delayMicroseconds(4000);
	}
	acc_x_cal_value /= CALIBRATION_N;
	acc_y_cal_value /= CALIBRATION_N;
	gyro_roll_cal /= CALIBRATION_N;
	gyro_pitch_cal /= CALIBRATION_N;
	gyro_yaw_cal /= CALIBRATION_N;

	// Enable subtracting calibration values (calibration done)
	level_calibration_on = 0;
	lcd.clear();

	gyro_signalen();
	speed_loop_timer = millis();
}

void gyro_signalen(void) {
	Wire.beginTransmission(MPU6050_ADDRESS);
	// Start reading @ register 43h and auto increment with every read
	Wire.write(0x3B);
	Wire.endTransmission();

	// Request 14 bytes from the MPU 6050.
	Wire.requestFrom(MPU6050_ADDRESS, 14);

	// Add the low and high byte to the acc variables
	acc_x = Wire.read() << 8 | Wire.read();
	acc_y = Wire.read() << 8 | Wire.read();
	acc_z = Wire.read() << 8 | Wire.read();			

	// Add the low and high byte to the temperature variable
	temperature = Wire.read() << 8 | Wire.read();

	// Read high and low parts of the angular data
	gyro_roll = Wire.read() << 8 | Wire.read();
	gyro_pitch = Wire.read() << 8 | Wire.read();
	gyro_yaw = Wire.read() << 8 | Wire.read();

	// Invert the direction of the axes
	//gyro_roll *= -1;
	gyro_pitch *= -1;
	gyro_yaw *= -1;

	if (level_calibration_on == 0) {
		// Subtact the calibration values
		acc_x -= acc_x_cal_value;
		acc_y -= acc_y_cal_value;
		gyro_roll -= gyro_roll_cal;
		gyro_pitch -= gyro_pitch_cal;
		gyro_yaw -= gyro_yaw_cal;
	}
}

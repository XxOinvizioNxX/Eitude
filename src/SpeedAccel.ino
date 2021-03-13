void speed_handler(void) {
	gyro_signalen();
	//acc_xy_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y));

	acc_x_filtered = acc_x_filtered * ACC_FILTER_KOEFF + acc_x * (1.0 - ACC_FILTER_KOEFF);
	acc_y_filtered = acc_y_filtered * ACC_FILTER_KOEFF + acc_y * (1.0 - ACC_FILTER_KOEFF);

	speed = acc_x_filtered;
	speed /= 4096.0;							// Now acc is in G
	speed *= 9.81;								// Now acc is in m/s^2
	speed *= (millis() - speed_loop_timer);		// Multiply by dt to get instant speed in m/ms

	speed_loop_timer = millis();				// Reset timer

	speed /= 1000.0;							// Convert to m/s
	speed *= 3.6;								// Convert to km/h
	
	speed_accumulator += speed;
}

void gyro_setup(void) {
	Wire.beginTransmission(MPU6050_ADDRESS);	// Start communication with the MPU-6050.
	Wire.write(0x6B);							// We want to write to the PWR_MGMT_1 register (6B hex).
	Wire.write(0x00);							// Set the register bits as 00000000 to activate the gyro.
	Wire.endTransmission();						// End the transmission with the gyro.

	Wire.beginTransmission(MPU6050_ADDRESS);	// Start communication with the MPU-6050.
	Wire.write(0x1B);							// We want to write to the GYRO_CONFIG register (1B hex).
	Wire.write(0x08);							// Set the register bits as 00001000 (500dps full scale).
	Wire.endTransmission();						// End the transmission with the gyro.

	Wire.beginTransmission(MPU6050_ADDRESS);	// Start communication with the MPU-6050.
	Wire.write(0x1C);							// We want to write to the ACCEL_CONFIG register (1A hex).
	Wire.write(0x10);							// Set the register bits as 00010000 (+/- 8g full scale range).
	Wire.endTransmission();						// End the transmission with the gyro.

	Wire.beginTransmission(MPU6050_ADDRESS);	// Start communication with the MPU-6050.
	Wire.write(0x1A);							// We want to write to the CONFIG register (1A hex).
	Wire.write(0x03);							// Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
	Wire.endTransmission();						// End the transmission with the gyro.

	gyro_calibration();							// Calibrate MPU6050
}

void gyro_calibration(void) {
	level_calibration_on = 1;					// Disable subtracting calibration values
	acc_x_cal_value = 0;
	acc_y_cal_value = 0;
	gyro_roll_cal = 0;
	gyro_pitch_cal = 0;
	gyro_yaw_cal = 0;
	for (uint16_t i = 0; i < CALIBRATION_N; i++) {
		gyro_signalen();
		acc_x_cal_value += acc_x;
		acc_y_cal_value += acc_y;
		gyro_roll_cal += gyro_roll;
		gyro_pitch_cal += gyro_pitch;
		gyro_yaw_cal += gyro_yaw;
		delayMicroseconds(4000);
	}
	acc_x_cal_value /= CALIBRATION_N;
	acc_y_cal_value /= CALIBRATION_N;
	gyro_roll_cal /= CALIBRATION_N;
	gyro_pitch_cal /= CALIBRATION_N;
	gyro_yaw_cal /= CALIBRATION_N;

	level_calibration_on = 0;					// Enable subtracting calibration values (calibration done)

	gyro_signalen();
	speed_loop_timer = millis();
}

void gyro_signalen(void) {
	Wire.beginTransmission(MPU6050_ADDRESS);		// Start communication with the gyro.
	Wire.write(0x3B);								// Start reading @ register 43h and auto increment with every read.
	Wire.endTransmission();

	Wire.requestFrom(MPU6050_ADDRESS, 14);			// Request 14 bytes from the MPU 6050.
	acc_x = Wire.read() << 8 | Wire.read();			// Add the low and high byte to the acc_x variable.
	acc_y = Wire.read() << 8 | Wire.read();			// Add the low and high byte to the acc_y variable.
	acc_z = Wire.read() << 8 | Wire.read();			// Add the low and high byte to the acc_z variable.

	temperature = Wire.read() << 8 | Wire.read();	// Add the low and high byte to the temperature variable.
	gyro_roll = Wire.read() << 8 | Wire.read();		// Read high and low part of the angular data.
	gyro_pitch = Wire.read() << 8 | Wire.read();	// Read high and low part of the angular data.
	gyro_yaw = Wire.read() << 8 | Wire.read();		// Read high and low part of the angular data.
	//gyro_roll *= -1;								// Invert the direction of the axis.
	gyro_pitch *= -1;								// Invert the direction of the axis.
	gyro_yaw *= -1;                                 // Invert the direction of the axis.

	if (level_calibration_on == 0) {
		acc_x -= acc_x_cal_value;				// Subtact the manual accelerometer pitch calibration value.
		acc_y -= acc_y_cal_value;				// Subtact the manual accelerometer roll calibration value.
		gyro_roll -= gyro_roll_cal;					// Subtact the manual gyro roll calibration value.
		gyro_pitch -= gyro_pitch_cal;				// Subtact the manual gyro pitch calibration value.
		gyro_yaw -= gyro_yaw_cal;					// Subtact the manual gyro yaw calibration value.
	}
}

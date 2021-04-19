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

/// <summary>
/// Prints current state to the screen by the timer
/// </summary>
void lcd_print_state(void) {
	if (millis() - lcd_timer >= LCD_PRINT_TIME) {
		lcd.setCursor(0, 1);
		if (!error) {
			// Print state if no error
			// Illumination
			lcd.print(F("L:"));
			lcd.print((uint16_t)filtered_value);

			// Speed
			lcd.print(F(" S:"));
			lcd.print(speed_accumulator, 1);

			// Fill line
			lcd.print(F("                "));

			// Light ON flag
			lcd.setCursor(15, 0);
			if (digitalRead(LIGHTS_PIN))
				lcd.print(F("*"));
			else
				lcd.print(F("."));

			// Move flag
			lcd.setCursor(15, 1);
			if (in_move_flag)
				lcd.print(F("<"));
			else
				lcd.print(F("."));
		}
		else
			lcd_print_error();

		// Reset timer
		lcd_timer = millis();
	}
}

/// <summary>
/// Prints serial buffer to the display
/// </summary>
void lcd_print_buffer(void) {
	// Start at the 0 row 0 col
	lcd.setCursor(0, 0);

	// Clear line without last char
	for (count_var = 0; count_var < 15; count_var++)
		lcd.print(F(" "));
	lcd.setCursor(0, 0);

	// Timestamp
	lcd.print(millis() / 1000);
	lcd.print(F(": "));

	// Current GCode
	for (count_var = 0; count_var < sofar - 1; count_var++)
		lcd.print(buffer[count_var]);
}


/// <summary>
/// Prints current error state on the LCD
/// </summary>
void lcd_print_error(void) {
	if (error) {
		// Fill line
		lcd.print(F("                "));
		lcd.setCursor(0, 1);

		// Print error
		lcd.print(F("ERROR: "));
		lcd.print(error);
	}
}
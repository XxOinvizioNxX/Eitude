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
/// Fills buffer with the received data
/// </summary>
void serial_reader(void) {
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
			lcd_print_buffer();

			// Process current command
			gcode_handler();

			// Send ready sign
			serial_ready();

			// Reset buffer
			sofar = 0;
		}
	}
}

/// <summary>
/// Prints ready sign to the serial port
/// </summary>
void serial_ready(void) {
	Serial.print(F(">"));
}

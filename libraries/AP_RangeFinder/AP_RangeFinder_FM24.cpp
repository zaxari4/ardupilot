/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_RangeFinder_FM24.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

#define FM24_FRAME_HEADER 0xFF  // Header Byte for FM24 (0xFF)
#define FM24_FRAME_LENGTH 8     // FM24 simple frame length

#define FM24_BAUD 57600
#define FM24_BUFSIZE_RX 128
#define FM24_BUFSIZE_TX 128

extern const AP_HAL::HAL &hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_FM24::AP_RangeFinder_FM24(RangeFinder::RangeFinder_State &_state,
                                         AP_RangeFinder_Params &_params,
                                         uint8_t serial_instance) : AP_RangeFinder_Backend(_state, _params) {
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(FM24_BAUD, FM24_BUFSIZE_RX, FM24_BUFSIZE_TX);
    }
}

/*
   detect if a FM24 rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_FM24::detect(uint8_t serial_instance) {
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_FM24::get_reading(uint16_t &reading_cm) {
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;

    int16_t nbytes = uart->available();
    //printf("Read %d bytes: ", nbytes);

    while (nbytes-- > 0) {
        int16_t r = uart->read();
        //printf("%d,", r);
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0xFF, add to buffer
        if (_linebuf_len == 0) {
            if (c == FM24_FRAME_HEADER) {
                _linebuf[_linebuf_len++] = c;
            }
        } else if (_linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0xFF, add it to buffer
            // if not clear the buffer
            if (c == FM24_FRAME_HEADER) {
                _linebuf[_linebuf_len++] = c;
            } else {
                _linebuf_len = 0;
            }
        } else if (_linebuf_len == 2) {
            // if buffer has 2 elements and this byte is 0xFF, add it to buffer
            // if not clear the buffer
            if (c == FM24_FRAME_HEADER) {
                _linebuf[_linebuf_len++] = c;
            } else {
                _linebuf_len = 0;
            }
        } else {
            _linebuf[_linebuf_len++] = c;

            // if buffer now has 8 items and its last three bytes are 0 try to decode it
            if (_linebuf_len == FM24_FRAME_LENGTH && _linebuf[5] == 0 && _linebuf[6] == 0 && _linebuf[7] == 0) {
                uint16_t dist = ((uint16_t)_linebuf[3] << 8) | _linebuf[4];
                sum_cm += dist;
                count++;
                _linebuf_len = 0;  // clear the buffer
            }
        }
    }

    //printf("\n");

    if (count > 0) {
        // return average distance of readings
        reading_cm = sum_cm / count;
        return true;
    }

    // no readings so return false
    return false;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_FM24::update(void) {
    if (get_reading(state.distance_cm)) {
        state.last_reading_ms = AP_HAL::millis();
        // update range_valid state based on distance measured
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

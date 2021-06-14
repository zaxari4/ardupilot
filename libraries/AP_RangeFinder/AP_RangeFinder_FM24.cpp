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

#define FM24_FRAME_HEADER 0xFF  // Header Byte for FM24 (0xFF)
#define FM24_FRAME_LENGTH 8     // FM24 simple frame length

bool AP_RangeFinder_FM24::get_reading(uint16_t &reading_cm) {

    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;

    int16_t nbytes = uart->available();

    while (nbytes-- > 0) {

        int16_t r = uart->read();
        if (r < 0) continue;
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

    if (count > 0) {
        // return average distance of readings
        reading_cm = sum_cm / count;
        return true;
    }

    // no readings so return false
    return false;
}

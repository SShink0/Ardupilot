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


#include <AP_HAL/AP_HAL.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#ifdef HAL_WITH_ESC_TELEM

#include "AP_BattMonitor_ESC.h"

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_ESC::init(void)
{
}

void AP_BattMonitor_ESC::read(void)
{
    AP_ESC_Telem& telem = AP::esc_telem();

    uint8_t num_escs = 0;
    float voltage_sum = 0;
    float current_sum = 0;
    float consumed_sum = 0;
    float temperature_sum = 0;
    uint32_t highest_ms = 0;

    for (uint8_t i=0; i<ESC_TELEM_MAX_ESCS; i++) {
        int16_t  temperature_deg;
        uint16_t voltage_cv;
        uint16_t current_ca;
        uint16_t consumption_mah;

        if (telem.get_consumption_mah(i, consumption_mah)) {
            // accumulate consumed_sum regardless of age, to cope with ESC
            // dropping out
            consumed_sum += consumption_mah;
        }

        if (telem.get_voltage_cv(i, voltage_cv)) {
            voltage_sum += voltage_cv;
        }

        if (telem.get_current_ca(i, current_ca)) {
            current_ca += current_ca;
        }

        if (telem.get_temperature(i, temperature_deg)) {
            temperature_sum += temperature_deg;
        }

        num_escs++;

        if (telem.get_last_telem_data_ms(i) > highest_ms) {
            highest_ms = telem.get_last_telem_data_ms(i);
        }
    }

    if (num_escs > 0) {
        _state.voltage = (voltage_sum / num_escs) * 0.01;
        _state.temperature = temperature_sum / num_escs;
        _state.healthy = true;
    } else {
        _state.voltage = 0;
        _state.temperature = 0;
        _state.healthy = false;
    }
    _state.current_amps = current_sum * 0.01;
    _state.consumed_mah = consumed_sum;
    _state.last_time_micros = highest_ms * 1000;
    _state.temperature_time = highest_ms;

    if (current_sum > 0) {
        // if we have ever got a current value then we know we have a
        // current sensor
        have_current = true;
    }
}

#endif // HAL_WITH_ESC_TELEM

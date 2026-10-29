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
#include "AP_RangeFinder_SITL.h"

#if AP_RANGEFINDER_SIM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

static constexpr uint32_t RANGEFINDER_SIM_TIMEOUT_MS = 500;

/*
  constructor - registers instance at top RangeFinder driver
 */
AP_RangeFinder_SITL::AP_RangeFinder_SITL(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, uint8_t instance) :
    AP_RangeFinder_Backend(_state, _params),
    _instance(instance)
{}

/*
  update distance_cm
 */
void AP_RangeFinder_SITL::update(void)
{
    const float dist = AP::sitl()->get_rangefinder(_instance);

    bool seen, valid;
    uint32_t last_update_ms;
    if (AP::sitl()->get_rangefinder_update(_instance, seen, valid, last_update_ms)) {
        if (!seen || !valid || isnan(dist) || isinf(dist) ||
            AP_HAL::millis() - last_update_ms > RANGEFINDER_SIM_TIMEOUT_MS) {
            _have_sample = false;
            set_status(RangeFinder::Status::NoData);
            return;
        }
        if (_have_sample && last_update_ms == _last_update_ms) {
            return;
        }
        _have_sample = true;
        _last_update_ms = last_update_ms;
        state.distance_m = dist;
        state.last_reading_ms = last_update_ms;
        update_status();
        return;
    }

    // nan distance means nothing is connected
    if (isnan(dist) || isinf(dist)) {
        state.status = RangeFinder::Status::NoData;
        return;
    }

    state.distance_m = MAX(0, dist);
    state.last_reading_ms = AP_HAL::millis();

    // update range_valid state based on distance measured
    update_status();
}

#endif  // AP_RANGEFINDER_SIM_ENABLED

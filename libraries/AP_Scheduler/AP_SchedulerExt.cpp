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

/*
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 */
#include "AP_SchedulerExt.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include <stdio.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define SCHEDULER_DEFAULT_LOOP_RATE 400
#else
#define SCHEDULER_DEFAULT_LOOP_RATE  50
#endif

const AP_Param::GroupInfo AP_SchedulerExt::var_info[] = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
    // @Values: 0:Disabled,2:ShowSlips,3:ShowOverruns
    // @User: Advanced
    AP_GROUPINFO("DEBUG",    0, AP_SchedulerExt, _debug, 0),

    // @Param: LOOP_RATE
    // @DisplayName: Scheduling main loop rate
    // @Description: This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart. Values over 400 are considered highly experimental.
    // @Values: 50:50Hz,100:100Hz,200:200Hz,250:250Hz,300:300Hz,400:400Hz
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("LOOP_RATE",  1, AP_SchedulerExt, _loop_rate_hz, SCHEDULER_DEFAULT_LOOP_RATE),

    AP_GROUPEND
};

// constructor
AP_SchedulerExt::AP_SchedulerExt(scheduler_fastloop_fn_t fastloop_fn)
{
    if (_singleton_ext) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many schedulers");
#endif
        return;
    }
    _singleton_ext = this;

    AP_Param::setup_object_defaults(this, var_info);

    init(_loop_rate_hz, fastloop_fn);
}

/*
 * Get the AP_Scheduler singleton
 */
AP_Scheduler *AP_SchedulerExt::_singleton_ext;
AP_Scheduler *AP_SchedulerExt::get_singleton_ext()
{
    return _singleton_ext;
}

namespace AP {

AP_Scheduler &scheduler_ext()
{
    return *AP_SchedulerExt::get_singleton_ext();
}

};

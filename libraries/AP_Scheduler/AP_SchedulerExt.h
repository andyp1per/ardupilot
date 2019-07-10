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
#pragma once

#include "AP_Scheduler.h"       

class AP_SchedulerExt : public AP_Scheduler
{
public:
    AP_SchedulerExt(scheduler_fastloop_fn_t fastloop_fn = nullptr);

    /* Do not allow copies */
    AP_SchedulerExt(const AP_Scheduler &other) = delete;
    AP_SchedulerExt &operator=(const AP_SchedulerExt&) = delete;

    static AP_Scheduler *get_singleton_ext();
    static AP_Scheduler *_singleton_ext;

    static const struct AP_Param::GroupInfo var_info[];

};

namespace AP {
    AP_Scheduler &scheduler_ext();
};

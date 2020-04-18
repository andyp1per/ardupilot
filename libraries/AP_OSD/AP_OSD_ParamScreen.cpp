/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * AP_OSD partially based on betaflight and inav osd.c implemention.
 * clarity.mcm font is taken from inav configurator.
 * Many thanks to their authors.
 */
/*
  parameter settings for one screen
 */
#include "AP_OSD.h"
#include "AP_OSD_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>

#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_OSD_ParamScreen::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable screen
    // @Description: Enable this screen
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_OSD_ParamScreen, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PARAM1_EN
    // @DisplayName: PARAM1_EN
    // @Description: Enables display of parameter 1
    // @Values: 0:Disabled,1:Enabled

    // @Param: PARAM1_X
    // @DisplayName: PARAM1_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: PARAM1_Y
    // @DisplayName: PARAM1_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(param1, "PARAM1", 2, AP_OSD_ParamScreen, AP_OSD_Setting),
    
    AP_GROUPEND
};

void AP_OSD_ParamScreen::draw_parameter(AP_Param param, uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%3d%c", 0, SYM_DEGR);
}

#define DRAW_SETTING(n) if (n.enabled) draw_ ## n(n.xpos, n.ypos)
#define DRAW_PARAM(n) if (n.enabled) draw_parameter(n.parameter, n.xpos, n.ypos)


void AP_OSD_ParamScreen::draw(void)
{
    if (!enabled || !backend) {
        return;
    }
    DRAW_PARAM(param1);
}


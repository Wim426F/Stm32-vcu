/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Wim Boone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <superman.h>

void Superman::SetCanInterface(CanHardware* c)
{
    can = c;
}

void Superman::Task100Ms()
{
    if (!Param::GetInt(Param::T15Stat)) return;
    
    int control = Param::GetInt(Param::Control);

    //Preheat request active while RTC preheat window (Pre_Hrs:Pre_Min, Pre_Dur minutes) is running
    bool preheat = false;
    if (control == 2)
    {
        int16_t windowStart = Param::GetInt(Param::Pre_Hrs) * 60 + Param::GetInt(Param::Pre_Min);
        int16_t duration = Param::GetInt(Param::Pre_Dur);
        int16_t now = Param::GetInt(Param::Hour) * 60 + Param::GetInt(Param::Min);
        int16_t elapsed = (now - windowStart + 1440) % 1440; // minutes since window start, wraps at midnight
        preheat = (duration > 0 && elapsed < duration);
    }

    uint8_t bytes[6];
    bytes[0] = 0; // cool_cabin unused
    bytes[1] = (control == 1) ? 1 : 0; // heat_cabinl
    bytes[2] = (control == 1) ? 1 : 0; // heat_cabinr
    bytes[3] = preheat ? 1 : 0; // preheat_req
    bytes[4] = (uint8_t)(Param::GetFloat(Param::BMS_Tavg) + 40); // battery temp
    bytes[5] = (uint8_t)(Param::GetFloat(Param::tmpm) + 40); // powertrain temp

    can->Send(0x6E0, (uint32_t*)bytes, 6);
}

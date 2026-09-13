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

#ifndef SUPERMAN_H
#define SUPERMAN_H

#include <heater.h>
#include "params.h"

//Transmits cabin heat, preheat and temperature data to the Superman climate module on 0x6E0
class Superman : public Heater
{
public:
    void SetCanInterface(CanHardware* c);
    void Task100Ms();
    void SetTargetTemperature(float temp) { (void)temp; }
    void SetPower(uint16_t, bool) {};
    void DecodeCAN(int, uint32_t*) {};
};

#endif // SUPERMAN_H

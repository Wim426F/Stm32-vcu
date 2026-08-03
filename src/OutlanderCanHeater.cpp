/*
 * This file is part of the Zombieverter VCU project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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

#include <OutlanderCanHeater.h>
#include "OutlanderHeartBeat.h"

void OutlanderCanHeater::SetPower(uint16_t power, bool HeatReq)
{
    shouldHeat = HeatReq;
    power = power;//mask warning
}

void OutlanderCanHeater::SetCanInterface(CanHardware* c)
{
    OutlanderHeartBeat::SetCanInterface(c);//set Outlander Heartbeat on same CAN

    can = c;
    can->RegisterUserMessage(0x398);
}

void OutlanderCanHeater::Task100Ms()
{
    if (shouldHeat)
    {
        uint8_t bytes[8];

        bytes[0] = 0x03;
        bytes[1] = 0x50;
        bytes[2] = 0x00;
        bytes[3] = 0x4D;
        bytes[4] = 0x00;
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;

        // Heater can only do 2 power settings
        currentTemperature = Param::GetInt(Param::tmpheater);
        uint16_t requestedPower = 0;
        if (currentTemperature < desiredTemperature - 5) {
            requestedPower = 3000;
        } else if (currentTemperature < desiredTemperature) {
            requestedPower = 1500;
        }

        // Limit requested power to HeatPwr, stepping down to the next available level
        uint16_t powerLimit = Param::GetInt(Param::HeatPwr); // We use Heatpwr as a ceiling, not a setpoint
        if (requestedPower > powerLimit) {
            requestedPower = powerLimit >= 1500 ? 1500 : 0;
        }

        if (requestedPower == 3000) {
            bytes[2] = 0xA2;
        } else if (requestedPower == 1500) {
            bytes[2] = 0x32;
        }
        Param::SetInt(Param::powerheater, requestedPower);


        can->Send(0x188, (uint32_t*)bytes, 8);
    }
}

void OutlanderCanHeater::SetTargetTemperature(float temp)
{
    desiredTemperature = temp;
}

void OutlanderCanHeater::DecodeCAN(int id, uint32_t data[2])
{
    switch (id)
    {
    case 0x398:
        OutlanderCanHeater::handle398(data);
        break;
    }
}

void OutlanderCanHeater::handle398(uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    // A raw byte of 0 is a "no data" placeholder, not a real -40C reading.
    // Ignore those and keep the last valid value to avoid false heater-on cycling.
    if (bytes[3] != 0 || bytes[4] != 0)
    {
        int temp1 = bytes[3] - 40;
        int temp2 = bytes[4] - 40;
        Param::SetInt(Param::tmpheater, temp2 > temp1 ? temp2 : temp1);
    }

    if (bytes[6] == 0x09)
    {
        Param::SetInt(Param::udcheater, 0);
    }
    else
    {
        Param::SetInt(Param::udcheater, Param::GetInt(Param::udc));
    }

}

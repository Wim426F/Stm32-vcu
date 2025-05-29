/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2022  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
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

#include "EvControlsT2C.h"
#include "my_math.h"
#include "params.h"

enum ShiftCommand {
    DRIVE_SHIFT_COMMAND = 0x0D,
    NEUTRAL_SHIFT_COMMAND = 0x0E,
    REVERSE_SHIFT_COMMAND = 0x0F
};

EvControlsT2C::EvControlsT2C()
{
}

void EvControlsT2C::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x108);
    can->RegisterUserMessage(0x126);
    can->RegisterUserMessage(0x315);
}

void EvControlsT2C::DecodeCAN(int id, uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;
    switch (id)
    {
    case 0x108: // ID264DIR_torque
        // DIR_axleSpeed: 40|16@1- (0.1,0) [-2750|2750] "RPM"
        speed = ((int16_t)((bytes[5] << 8) | bytes[4])) * 0.1f;
        break;
    case 0x126: // ID126RearHVStatus
        // RearHighVoltage126: 0|10@1+ (0.5,0) [0|500] "V"
        voltage = ((bytes[0] | (bytes[1] & 0x03) << 8)) * 0.5f;
        break;
    case 0x315: // ID315RearInverterTemps
        // DIR_motorTemp: 16|8@1+ (1,-40) [-40|215] "C"
        // DIR_IGBTJunctTemp: 24|8@1+ (1,-40) [-40|215] "C"
        motor_temp = (int8_t)bytes[2] + 40; // Signed 8-bit, offset -40
        inv_temp = (int8_t)bytes[3] + 40;   // Signed 8-bit, offset -40
        break;
    }
}

void EvControlsT2C::SetTorque(float torquePercent)
{
}

void EvControlsT2C::Task10Ms()
{
}

void EvControlsT2C::Task100Ms()
{
    static int counter = 0;
    if (counter >= 5) { // 5 * 100ms = run actually at 500ms loop
        counter = 0;

        int opmode = Param::GetInt(Param::opmode);
        if (opmode == MOD_RUN) {
            // Power and Regen Control (ID 0x696)
            int max_power = Param::GetInt(Param::idcmax) * Param::GetInt(Param::udc) / 1000; // in kW
            Param::SetFloat(Param::maxPower, max_power);
            int max_regen = Param::GetInt(Param::regenmax); // % of max current

            uint16_t discharge_kW_x100 = (uint16_t)(max_power * 100); // Convert to kW * 100
            uint16_t regen_kW_x100 = (uint16_t)(max_regen * max_power); // Convert % to kW * 100
            uint8_t bytes[8] = {
                (uint8_t)(discharge_kW_x100 >> 8), (uint8_t)discharge_kW_x100,
                (uint8_t)(regen_kW_x100 >> 8), (uint8_t)regen_kW_x100,
                0x00, 0x00, 0x00, 0x00
            };
            can->Send(0x696, bytes, 8);
        }
    }
    counter++;
}

void EvControlsT2C::setGear()
{
    uint8_t shift_command = NEUTRAL_SHIFT_COMMAND;

    if (Param::GetInt(Param::dir) == GearDir::Forward)  
        shift_command = DRIVE_SHIFT_COMMAND;

    if (Param::GetInt(Param::dir) == GearDir::Neutral)  
        shift_command = NEUTRAL_SHIFT_COMMAND;

    if (Param::GetInt(Param::dir) == GearDir::Reverse)  
        shift_command = REVERSE_SHIFT_COMMAND;

    uint8_t bytes[8] = {shift_command, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00};
    can->Send(0x697, bytes, 8);
}
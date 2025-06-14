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
        speed = ((int16_t)((bytes[5] << 8) | bytes[4])) * 0.9f; // Rotor RPM (axle * 9)
        Param::SetInt(Param::speed, speed);
        // DIR_axleTrq: 24|16@1- (1,0) [-32768|32767] "Nm"
        torque = (int16_t)((bytes[3] << 8) | bytes[2]) * 1.0f;
        Param::SetFloat(Param::torque, torque);
        break;

    case 0x126: // ID126RearHVStatus
        // RearHighVoltage126: 0|10@1+ (0.5,0) [0|500] "V"
        voltage = ((bytes[0] | (bytes[1] & 0x03) << 8)) * 0.5f;
        Param::SetFloat(Param::INVudc, voltage);
        Param::SetFloat(Param::udc, voltage); // udc is bus voltage, udc2 is battery voltage
        // RearMotorCurrent126: 11|11@1+ (1,0) [0|2047] "A"
        idcMotor = ((bytes[1] >> 3) | (bytes[2] & 0x1F) << 5) * 1.0f;
        Param::SetFloat(Param::idcMotor, idcMotor);
        break;

    case 0x315: // ID315RearInverterTemps
        // DIR_motorTemp: 16|8@1+ (1,-40) [-40|215] "C"
        motor_temp = (int8_t)bytes[2] + 40; // Signed 8-bit, offset -40
        Param::SetInt(Param::tmpm, motor_temp);
        // DIR_IGBTJunctTemp: 24|8@1+ (1,-40) [-40|215] "C"
        inv_temp = (int8_t)bytes[3] + 40;   // Signed 8-bit, offset -40
        Param::SetInt(Param::tmphs, inv_temp);
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
            int max_power = Param::GetInt(Param::idcmax) * Param::GetInt(Param::udc) / 1000; // in kW, discharge limit
            Param::SetFloat(Param::maxPower, max_power);
            int max_regen_current = Param::GetInt(Param::regenmax) * Param::GetInt(Param::idcmax) / 100; // % of idcmax in amps
            int max_regen = max_regen_current * Param::GetInt(Param::udc) / 1000; // in kW
        
            uint16_t power_kW_x100 = (uint16_t)(max_power * 100); // power limit in kW, factor is 0.01
            uint16_t regen_kW_x100 = (uint16_t)(max_regen * 100); // regen limit in kW, factor is 0.01
            uint8_t bytes[8] = {
                (uint8_t)(power_kW_x100 >> 8), (uint8_t)(power_kW_x100 & 0xFF), // Big-endian discharge limit
                (uint8_t)(regen_kW_x100 >> 8), (uint8_t)(regen_kW_x100 & 0xFF),       // Big-endian regen limit
                0x00, 0x00, 0x00, 0x00
            };
            can->Send(0x696, bytes, 8);
        }
    }
    counter++;

    motorPower = (idcMotor * voltage) / 1000.0f; // kW
    Param::SetFloat(Param::motorPower, motorPower);
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
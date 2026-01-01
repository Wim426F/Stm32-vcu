/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2025 Wim Boone
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
#include "errormessage.h"
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

    can->RegisterUserMessage(0x107);
    can->RegisterUserMessage(0x126);
    can->RegisterUserMessage(0x315);
    can->RegisterUserMessage(0x118);
}

void EvControlsT2C::DecodeCAN(int id, uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;
    switch (id)
    {
    case 0x107: {// ID264DIR_torque, renamed id from 0x108 to avoid collision with PCS control
        // DIR_axleSpeed: 40|16@1- (0.1,0) [-2750|2750] "RPM"
        speed = ((int16_t)((bytes[6] << 8) | bytes[5])) * 0.9f; // Rotor RPM (axle * 9)
        Param::SetInt(Param::speed, speed);
        // DIR_torqueActual: 27|13@1- (2,0) [-7500|7500] "Nm"
        uint16_t raw_torque = ((bytes[4] & 0x1F) << 8) | bytes[3]; // Extract 13 bits as unsigned
        int16_t signed_torque = (int16_t)raw_torque;
        if (raw_torque & 0x1000) signed_torque |= 0xE000; // Sign-extend 13-bit value to 16-bit
        float torque = (float)signed_torque * 2.0f / 9.0f; // divide by 9 because motor has reduction of 9
        Param::SetFloat(Param::torque, torque); }
        break;

    case 0x126: // ID126RearHVStatus
        // RearHighVoltage126: 0|10@1+ (1,0) [0|500] "V"
        voltage = (bytes[0] | (bytes[1] & 0x03) << 8);
        Param::SetFloat(Param::INVudc, voltage);
        Param::SetFloat(Param::udc, voltage); // udc is bus voltage, udc2 is battery voltage
        // RearMotorCurrent126: 11|11@1+ (1,0) [0|2047] "A"
        idcMotor = ((bytes[1] >> 3) | (bytes[2] & 0x1F) << 5);
        Param::SetFloat(Param::idcMotor, idcMotor);
        motorPower = (idcMotor * voltage) / 1000.0f; // kW
        Param::SetFloat(Param::motorPower, motorPower);
        // Reset timeout
        timeoutCounterInv = (uint8_t)(Param::GetInt(Param::InvTimeout) * 10);
        break;

    case 0x315: { // ID315RearInverterTemps
        // Check for invalid message (0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00)
        if (bytes[0] == 0x02 && 
            bytes[1] == 0x00 && bytes[2] == 0x00 && bytes[3] == 0x00 &&
            bytes[4] == 0x00 && bytes[5] == 0x00 && bytes[6] == 0x00 && bytes[7] == 0x00) {
            break; // Ignore invalid message
        }
        // Extract quality flag DIR_inverterTQF (bits 56-57)
        uint8_t qf = bytes[7] & 0x03;
        if (qf != 0 && qf != 2) { // Ignore if not INIT (0) or RATIONAL (2)
            break;
        }
        // Check if RearTempInvHeatsink315 raw is 0x00 (-40C)
        if (bytes[1] == 0x00 || bytes[2] == 0x00 || bytes[4] == 0x00) {
            break; // Ignore empty temps
        }
        // RearTempStator315 : 16|8@1+ (1,-40) [-40|215] "C"
        motor_temp = (int)bytes[2] - 40; // Corrected to unsigned byte - offset
        Param::SetInt(Param::tmpm, motor_temp);
        // RearTempInverter315 : 8|8@1+ (1,-40) [-40|215] "C"
        inv_temp = (int)bytes[1] - 40;   // Corrected to unsigned byte - offset
        Param::SetInt(Param::tmpinv, inv_temp);
        // RearTempInvHeatsink315 : 32|8@1+ (1,-40) [-40|215] "C"
        int heatsink_temp = (int)bytes[4] - 40; // Corrected to unsigned byte - offset
        Param::SetInt(Param::tmphs, heatsink_temp);
        break;
        }

    case 0x118: {
        //enum Gear { DI_GEAR_INVALID = 0, DI_GEAR_P = 1, DI_GEAR_R = 2, DI_GEAR_N = 3, DI_GEAR_D = 4, DI_GEAR_SNA = 7 };
        //enum KeepDrivePowerStateRequest { NO_REQUEST = 0, KEEP_ALIVE = 1 };
        uint8_t brakePedalState = (bytes[2] >> 3) & 0x03;
        Param::SetInt(Param::din_brake, brakePedalState);

        uint8_t Gear = (bytes[2] >> 5) & 0x07;    
        uint8_t regenlight = (bytes[3] >> 2) & 0x01;
        Param::SetInt(Param::regen_brakelight, regenlight);

        uint8_t keepDrivePowerStateRequest = (bytes[5] >> 7) & 0x01;
        break;
        }
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

    // Update timeout
   if (timeoutCounterInv > 0) timeoutCounterInv--;
   if (timeoutCounterInv < 1)
   {
      ErrorMessage::Post(ERR_INV_COMM);
      //Set important variables to 0 when timeout
      voltage = 0;
      Param::SetFloat(Param::INVudc, voltage);
      Param::SetFloat(Param::udc, voltage); // udc is bus voltage, udc2 is battery voltage
      idcMotor = 0;
      Param::SetFloat(Param::idcMotor, idcMotor);
      speed = 0;
      Param::SetInt(Param::speed, speed);
      torque = 0;
      Param::SetFloat(Param::torque, torque);
      motor_temp = 0;
      Param::SetInt(Param::tmpm, motor_temp);
      inv_temp = 0;
      Param::SetInt(Param::tmphs, inv_temp);
   }

    if (counter >= 5) { // 5 * 100ms = run actually at 500ms loop
        counter = 0;

        int opmode = Param::GetInt(Param::opmode);
        if (opmode == MOD_RUN) {
            // Power and Regen Control (ID 0x696)
            //float derated_idc = Param::GetFloat(Param::derated_idc);
            float derated_idc = Param::GetFloat(Param::idcmax);
            float max_power = derated_idc * (float)(Param::GetInt(Param::udc)) / 1000.0f; // kW
            Param::SetFloat(Param::maxPower, max_power);

            //float derated_regen = Param::GetFloat(Param::derated_regen);
            float derated_regen = Param::GetFloat(Param::regenmax);
            float max_regen_current_val = -derated_regen;  // Negative for charge direction
            float max_regen = max_regen_current_val * (float)(Param::GetInt(Param::udc)) / 1000.0f;
            if (Param::GetInt(Param::din_brake)) // dont mix regen with mechanical brake
            {
                max_regen = 0;
            }          

            // Convert to positive kW for CAN (assuming DU expects positive limits for both)
            uint16_t power_kW_x100 = (uint16_t)(max_power * 100.0f + 0.5f);         // power limit in kW, factor 0.01, manual rounding
            uint16_t regen_kW_x100 = (uint16_t)(ABS(max_regen) * 100.0f + 0.5f);    // regen limit in kW, factor 0.01, manual rounding

            uint8_t bytes[8] = 
            {
            (uint8_t)(power_kW_x100 >> 8), (uint8_t)(power_kW_x100 & 0xFF), // Big-endian discharge limit
            (uint8_t)(regen_kW_x100 >> 8), (uint8_t)(regen_kW_x100 & 0xFF), // Big-endian regen limit
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
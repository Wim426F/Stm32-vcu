/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2022 Charlie Smurthwaite
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

#include "DilithiumMCU.h"

/*
 * This module receives messages from DilithiumMCU and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 */

void DilithiumMCU::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x293); // MCU display PDO2 MISO
}

bool DilithiumMCU::BMSDataValid()
{
   if (timeoutCounter < 1) return false;
   return true;
}

bool DilithiumMCU::ChargeAllowed()
{
   if (!BMSDataValid()) return false;
   if (maxCellV > Param::GetFloat(Param::BMS_VmaxLimit)) return false;
   if (minCellV < Param::GetFloat(Param::BMS_VminLimit)) return false;
   if (maxTempC > Param::GetFloat(Param::BMS_TmaxLimit)) return false;
   if (minTempC < Param::GetFloat(Param::BMS_TminLimit)) return false;
   return true;
}

float DilithiumMCU::MaxChargeCurrent()
{
   if (!ChargeAllowed()) return 0;
   return 0; // No chargeCurrentLimit available
}

void DilithiumMCU::DecodeCAN(int id, uint8_t *data)
{
   if (id == 0x293)
   {
      switch (data[0]) // B0 indicates message type
      {
         case 0x03: // Cell Voltage Summary
            // CV Low: bytes 2-3, 16-bit unsigned, 1 mV
            minCellV = ((data[2] | (data[3] << 8)) / 1000.0f);
            // CV High: bytes 6-7, 16-bit unsigned, 1 mV
            maxCellV = ((data[6] | (data[7] << 8)) / 1000.0f);
            break;
         case 0x04: // Thermistor Summary
            // THMin: bytes 2-3, THMax: bytes 4-5, 16-bit signed, 0.1°C
            minTempC = ((int16_t)(data[3] << 8) | data[2]) * 0.1f;
            maxTempC = ((int16_t)(data[5] << 8) | data[4]) * 0.1f;
            break;
      }
      // Reset timeout
      timeoutCounter = (uint8_t)(Param::GetInt(Param::BMS_Timeout) * 10);
   }
}

void DilithiumMCU::Task100Ms()
{
   // Send PDO2 MOSI request (0x313)
   uint8_t request[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // All 0 for one-shot request
   can->Send(0x313, request, 8);

   // Update timeout
   if (timeoutCounter > 0) timeoutCounter--;

   Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

   if (BMSDataValid())
   {
      Param::SetFloat(Param::BMS_Vmin, minCellV);
      Param::SetFloat(Param::BMS_Vmax, maxCellV);
      Param::SetFloat(Param::BMS_Tmin, minTempC);
      Param::SetFloat(Param::BMS_Tmax, maxTempC);
   }
   else
   {
      Param::SetFloat(Param::BMS_Vmin, 0);
      Param::SetFloat(Param::BMS_Vmax, 0);
      Param::SetFloat(Param::BMS_Tmin, 0);
      Param::SetFloat(Param::BMS_Tmax, 0);
   }
}
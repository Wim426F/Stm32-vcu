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
   can->RegisterUserMessage(0x351); // ZEVCCS BM3_LIMITS for future support
}

bool DilithiumMCU::BMSDataValid()
{
   // Return false if primary BMS is not sending data.
   if (timeoutCounter < 1) return false;
   return true;
}

bool DilithiumMCU::ChargeAllowed()
{  
   // Refuse to charge if the BMS is not sending data.
   if (!BMSDataValid()) return false;

   // Refuse to charge if the voltage or temperature is out of range.
   if (maxCellV > Param::GetFloat(Param::BMS_VmaxLimit)) return false;
   if (minCellV < Param::GetFloat(Param::BMS_VminLimit)) return false;
   if (maxTempC > Param::GetFloat(Param::BMS_TmaxLimit)) return false;
   if (minTempC < Param::GetFloat(Param::BMS_TminLimit)) return false;

   // Refuse to charge if the current limit is zero.
   if (chargeCurrentLimit < 0.5) return false;

   // Otherwise, charging is permitted.
   return true;
}

// Return the maximum charge current allowed by the BMS.
float DilithiumMCU::MaxChargeCurrent()
{
   if (!ChargeAllowed()) return 0;
   return chargeCurrentLimit * 0.1;
}

// Process voltage and temperature message from DilithiumMCU.
void DilithiumMCU::DecodeCAN(int id, uint8_t *data)
{
   if (id == 0x293)
   {
      switch (data[0]) // B0 indicates message type
      {
         case 0x02: // Pack Summary
            // Pack Voltage: bytes 2-3, 16-bit unsigned, 0.1 V
            udc = ((data[2] | (data[3] << 8)) * 0.1f);
            // Pack Current: bytes 4-5, 16-bit signed, 0.1 A
            idc = ((int16_t)(data[5] << 8) | data[4]) * 0.1f;
            break;
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
         case 0x05: // SOC Summary
            // SOC: byte 1, 8-bit unsigned, 1%
            SOC = data[1] * 1.0f;
            // PackKWHr: bytes 2-3, 16-bit unsigned, 0.1 kWh
            KWh = ((data[2] | (data[3] << 8)) * 0.1f);
            break;
         case 0x0f: // GFM Summary
            // Isolation: byte 2, 8-bit unsigned, Ohms/V * udc
            BMS_Isolation = data[2] * udc;
            break;
      }
      // Reset timeout
      timeoutCounter = (uint8_t)(Param::GetInt(Param::BMS_Timeout) * 10);
   }
   else if (id == 0x351) // ZEVCCS BM3_LIMITS
   {
      // CCHi,CCLo: bytes 2-3, 16-bit signed, 0.1 A
      chargeCurrentLimit = ((int16_t)(data[3] << 8) | data[2]) * 0.1f;
   }
}

void DilithiumMCU::Task100Ms()
{
   // Send PDO2 MOSI request (0x313)
   uint8_t request[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // All 0 for continuous
   can->Send(0x313, request, 8);

   // Update timeout
   if (timeoutCounter > 0) timeoutCounter--;

   // Calculate derived values
   deltaV = maxCellV - minCellV;
   power = (udc * idc) / 1000.0f;
   AMPh = (KWh * 1000.0f) / (3.7f * 73.0f);
   BMS_Tavg = (minTempC + maxTempC) / 2.0f;

   Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

   if (BMSDataValid())
   {
      Param::SetFloat(Param::BMS_Vmin, minCellV);
      Param::SetFloat(Param::BMS_Vmax, maxCellV);
      Param::SetFloat(Param::BMS_Tmin, minTempC);
      Param::SetFloat(Param::BMS_Tmax, maxTempC);
      Param::SetFloat(Param::udc, udc);
      Param::SetFloat(Param::udcsw, udc - 10); //Set for precharging based on actual voltage
      Param::SetFloat(Param::deltaV, deltaV);
      Param::SetFloat(Param::power, power);
      Param::SetFloat(Param::idc, idc);
      Param::SetFloat(Param::KWh, KWh);
      Param::SetFloat(Param::AMPh, AMPh);
      Param::SetFloat(Param::SOC, SOC);
      Param::SetFloat(Param::BMS_Tavg, BMS_Tavg);
      Param::SetFloat(Param::BMS_Isolation, BMS_Isolation);
   }
   else
   {
      Param::SetFloat(Param::BMS_Vmin, 0);
      Param::SetFloat(Param::BMS_Vmax, 0);
      Param::SetFloat(Param::BMS_Tmin, 0);
      Param::SetFloat(Param::BMS_Tmax, 0);
      Param::SetFloat(Param::udc, 0);
      Param::SetFloat(Param::udcsw, Param::GetFloat(Param::udcmin)); // not 0 otherwise precharge succeeds doing nothing
      Param::SetFloat(Param::deltaV, 0);
      Param::SetFloat(Param::power, 0);
      Param::SetFloat(Param::idc, 0);
      Param::SetFloat(Param::KWh, 0);
      Param::SetFloat(Param::AMPh, 0);
      Param::SetFloat(Param::SOC, 0);
      Param::SetFloat(Param::BMS_Tavg, 0);
      Param::SetFloat(Param::BMS_Isolation, 0);
   }
}
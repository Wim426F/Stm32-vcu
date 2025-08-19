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
#include "errormessage.h"

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
   if (timeoutCounterBMS < 1)
   {
      ErrorMessage::Post(ERR_BMS_COMM);
      return false;
   }
   if (timeoutCounterGFM < 1)
   {
      ErrorMessage::Post(ERR_GFM_COMM);
      return false;
   }
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
            battVoltage = ((data[2] | (data[3] << 8)) * 0.1f); // PackVoltage
            battCurrent = ((int16_t)(data[5] << 8) | data[4]) * 0.1f; // PackCurrent
            break;
         case 0x03: // Cell Voltage Summary
            minCellV = ((data[2] | (data[3] << 8)) / 10.0f); // CVLow
            maxCellV = ((data[6] | (data[7] << 8)) / 10.0f); // CVHigh
            break;
         case 0x04: // Thermistor Summary
            minTempC = (int8_t)data[2] * 0.1f; // TMin
            maxTempC = (int8_t)data[3] * 0.1f; // TMax
            break;
         case 0x05: // SOC Summary
            SOC = data[1] * 1.0f; // SOC
            KWh = ((data[2] | (data[3] << 8)) * 0.1f); // PackKWHr
            MaxkWh = ((data[4] | (data[5] << 8)) * 0.1f); // PackMaxKWHr
            break;
         case 0x0f: // GFM Summary
            BMS_Isolation = (data[2] | (data[3] << 8)); // GroundFaultIsolation in Ohm/v
            timeoutCounterGFM = (uint8_t)(Param::GetInt(Param::BMS_Timeout) * 10);
            break;
         case 0x01: // MCU Summary
            uint16_t alerts = (data[6] | (data[7] << 8)); // Alerts
            break;
      }
      // Reset timeout
      if(data[0] != 0x0f) // make sure its not the GFM that we hear
         timeoutCounterBMS = (uint8_t)(Param::GetInt(Param::BMS_Timeout) * 10);
   }
   else if (id == 0x351) // ZEVCCS BMS_LIMITS
{
   // TargetChargeCurrent: bytes 3-2, 16-bit unsigned, 0.1 A
   chargeCurrentLimit = ((data[3] << 8) | data[2]) * 0.1f;
   // DischargeCurrentLimit: byte 4, 8-bit unsigned, 0.1 A
   dischargeCurrentLimit = (data[4]) * 0.1f;
}
}

void DilithiumMCU::Task100Ms()
{
   // Send PDO2 MOSI request (0x313)
   uint8_t request[8] = {0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00};
   can->Send(0x313, request, 8);

   // Update timeout
   if (timeoutCounterBMS > 0) timeoutCounterBMS--;
   if (timeoutCounterGFM > 0) timeoutCounterGFM--;

   // Calculate derived values
   deltaV = (maxCellV - minCellV);
   power = (battVoltage * battCurrent) / 1000.0f;
   AMPh = (KWh * 1000.0f) / (3.7f * 73.0f);
   BMS_Tavg = (minTempC + maxTempC) / 2.0f;

   Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

   if (BMSDataValid())
   {
      Param::SetFloat(Param::BMS_Vmin, minCellV);
      Param::SetFloat(Param::BMS_Vmax, maxCellV);
      Param::SetFloat(Param::BMS_Tmin, minTempC);
      Param::SetFloat(Param::BMS_Tmax, maxTempC);
      Param::SetFloat(Param::udc2, battVoltage); // udc2 is battery voltage, udc is bus voltage
      Param::SetFloat(Param::udcsw, battVoltage - 15); //Set for precharging based on actual voltage
      Param::SetFloat(Param::deltaV, deltaV);
      Param::SetFloat(Param::power, power);
      Param::SetFloat(Param::idc, battCurrent);
      Param::SetFloat(Param::KWh, KWh);
      Param::SetFloat(Param::BattCap, MaxkWh);
      Param::SetFloat(Param::AMPh, AMPh);
      Param::SetFloat(Param::SOC, SOC);
      Param::SetFloat(Param::BMS_Tavg, BMS_Tavg);
      Param::SetFloat(Param::BMS_IsoMeas, BMS_Isolation); // isolation in Ohm/v
      Param::SetFloat(Param::BMS_Isolation, (BMS_Isolation*battVoltage)); // total isolation in Ohm
      Param::SetFloat(Param::BMS_ChargeLim, chargeCurrentLimit);
   }
   else
   {
      Param::SetFloat(Param::BMS_Vmin, 0);
      Param::SetFloat(Param::BMS_Vmax, 0);
      Param::SetFloat(Param::BMS_Tmin, 0);
      Param::SetFloat(Param::BMS_Tmax, 0);
      Param::SetFloat(Param::udc2, 0);
      Param::SetFloat(Param::udcsw, Param::GetFloat(Param::udcmin)); // not 0 otherwise precharge succeeds at 0v
      Param::SetFloat(Param::deltaV, 0);
      Param::SetFloat(Param::power, 0);
      Param::SetFloat(Param::idc, 0);
      Param::SetFloat(Param::KWh, 0);
      Param::SetFloat(Param::AMPh, 0);
      Param::SetFloat(Param::SOC, 0);
      Param::SetFloat(Param::BMS_Tavg, 0);
      Param::SetFloat(Param::BMS_Isolation, 0);
   }

   if (BMS_Isolation < Param::GetInt(Param::BMS_IsoLimit) && timeoutCounterGFM > 1)
   {
      ErrorMessage::Post(ERR_ISOLATION);
   }
}
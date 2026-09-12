/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
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

#include "teslaCharger.h"
#include "params.h"
#include "my_math.h"
#include "errormessage.h"

static bool ChRun = false;

static uint8_t grid_config = 0;
static float AC_line_voltage = 0;
static float charger_max_power = 0; // Watt
static int timeoutCounterPCS = 0;

enum gridConfig
{
   GRID_NONE = 0,
   GRID_1PHASE,
   GRID_3PHASE,
   GRID_3PHASE_DELTA
};


void teslaCharger::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x108);
}

void teslaCharger::DecodeCAN(int id, uint32_t data[2])
{  
   uint8_t* bytes = (uint8_t*)data;
   if (id == 0x108)
   {
      // Unpack GridCFG (2 bits) and uac (10 bits) from bytes[0] and bytes[1]
      // AC voltage bits 0-7 in byte[0]
      // AC voltage bits 8-9 in byte[1] 0-1
      AC_line_voltage = (float)(bytes[0] | ((bytes[1] & 0x03) << 8));   
      Param::SetInt(Param::ChgAcVolt, (int)AC_line_voltage);
      grid_config = (bytes[1] >> 2) & 0x03;  // grid_config 2 bits in byte[1] bits 2-3

      // Charger_Fault (bit 12) | DCDC_Fault (bit 13) -> byte[1] bits 4-5. PCS_Other_Alert (bit 14 / byte[1] bit 6) ignored for now.
      bool chargerFault = (bytes[1] & 0x30) != 0;
      Param::SetInt(Param::errlights, chargerFault ? 4 : 0); // Set EPC warn light in dashboard

      charger_max_power = bytes[2] * 100.0f; // 0.1 kW to Watt

      // Reset timeout
      timeoutCounterPCS = (uint8_t)(Param::GetInt(Param::CanTimeout) * 10);
   }
}

void teslaCharger::Task100Ms()
{
   // Update timeout
   if (timeoutCounterPCS > 0) timeoutCounterPCS--;
   if (timeoutCounterPCS < 1)
   {
      ErrorMessage::Post(ERR_PCS_COMM);
   }

   if (!Param::GetInt(Param::T15Stat)) return;

   uint8_t bytes[8];
   
   int HVvolts = Param::GetInt(Param::udc);
   int HVspnt = Param::GetInt(Param::Voltspnt);
   
   // What the battery can absorb (W). The estimator accounts for cell voltage
   // headroom and pack temperature; PwrAcMax below is the user's preference and
   // this can only pull us under it.
   float bms_max_power = Param::GetFloat(Param::BMS_MaxInput) * 1000.0f;
   
   // Get EVSE current limit: minimum of PilotLim and CableLim
   int evse_max_current = MIN(Param::GetInt(Param::PilotLim), Param::GetInt(Param::CableLim));
   // current limit for CAN message (map 1-16A to 0-15)
   int currentLimitCapped = MIN(evse_max_current, 16);
   uint8_t currentLimit = (currentLimitCapped > 0) ? (currentLimitCapped - 1) : 0;
   
   // Calculate EVSE maximum power based on grid configuration (W)
   int num_phases = (grid_config == GRID_3PHASE || grid_config == GRID_3PHASE_DELTA) ? 3 : 1;
   float evse_max_power = AC_line_voltage * (float)num_phases * (float)evse_max_current;
   
   // Get user power limit (W)
   float vcu_userlimit = Param::GetFloat(Param::PwrAcMax);
   
   // Take minimum of all limits
   float final_setpoint = MIN(bms_max_power, evse_max_power);
   final_setpoint = MIN(final_setpoint, charger_max_power);
   final_setpoint = MIN(final_setpoint, vcu_userlimit);
   
   // Apply efficiency factor
   //final_setpoint *= Param::GetFloat(Param::ChgEff) / 100.0f;
   
   // When charge is stopping (ChRun false) command 0W so the charger spools
   // down gracefully instead of seeing "stop" while still commanded full power.
   int HVpwr = (int)final_setpoint;
   if (!ChRun) HVpwr = 0;

   Param::SetFloat(Param::chgPsetp, final_setpoint/1000.0f); // kW

   bytes[0] = Param::GetInt(Param::opmode);
   bytes[1] = (HVvolts & 0xFF);
   bytes[2] = ((HVvolts & 0xFF00) >> 8);
   bytes[3] = (HVspnt & 0xFF);
   bytes[4] = ((HVspnt & 0xFF00) >> 8);
   bytes[5] = (HVpwr & 0xFF);
   bytes[6] = ((HVpwr & 0xFF00) >> 8);
   bytes[7] = (ChRun ? (0xA << 4) : (0xC << 4)) | (currentLimit & 0xF);
   can->Send(0x109, (uint32_t*)bytes, 8);
}

bool teslaCharger::ControlCharge(bool RunCh, bool ACReq)
{
   (void)RunCh;
   ChRun = ACReq;
   return ACReq;
}
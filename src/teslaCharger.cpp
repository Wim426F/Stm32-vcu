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

//static bool HVreq=true;
static bool ChRun=false;
static uint8_t counter_109 = 0;
static uint16_t HVvolts=0;
static uint16_t HVspnt=0;
static uint16_t HVpwr=0;
static uint16_t calcBMSpwr=0;


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
     //if(bytes[0]==0xAA) HVreq=true;
     //if(bytes[0]==0xCC) HVreq=false;
   }
}

void teslaCharger::Task100Ms()
{
   uint8_t bytes[8];
   HVvolts = Param::GetInt(Param::udc);
   HVspnt = Param::GetInt(Param::Voltspnt);
   HVpwr = Param::GetInt(Param::Pwrspnt);
   calcBMSpwr = (HVvolts * Param::GetInt(Param::BMS_ChargeLim));
   HVpwr = MIN(HVpwr, calcBMSpwr);
   // Get EVSE current limit: minimum of PilotLim and CableLim, capped at 16A
   uint8_t currentLimit = MIN(Param::GetInt(Param::PilotLim), Param::GetInt(Param::CableLim));
   currentLimit = MIN(currentLimit, 16); // Cap at 16A
   currentLimit = (currentLimit > 0) ? currentLimit - 1 : 0; // Map 1–16A to 0–15 for 4-bit field
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
   bool dummy = RunCh;
   ChRun = ACReq;
   if(ACReq) return true;
   else return false;
}

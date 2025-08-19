/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
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

 This code is specific to the custom high voltage box in the BMW E90 (Wim426F)
 In this box is a small controller that manages the contactors and various other things
 It's called the HVCB (High Voltage Control Unit)
*/

#include <hvcu_box.h>
#include "errormessage.h"

#define RELAY_ON 0x02
#define RELAY_OFF 0x03

int timeoutCounterHVCU = 0;

enum ContactorState {CONTACTOR_OFF, CONTACTOR_STARTUP, CONTACTOR_ECONOMIZED };

void HVCU::RegisterCanMessages(CanHardware* can)
{
   can->RegisterUserMessage(0x398);//HVCU MSG
}

void HVCU::DecodeCAN(int id, uint32_t data[2])
{
   switch (id)
   {
   case 0x398:
      HVCU::handle398(data);//HVCU CAN MESSAGE
      break;
   }
}

void HVCU::Task100Ms()
{
   // Update timeout
   if (timeoutCounterHVCU > 0) timeoutCounterHVCU--;
   if (timeoutCounterHVCU < 1)
   {
      ErrorMessage::Post(ERR_HVCU_COMM);
   }
}


void HVCU::handle398(uint32_t data[2])  //HVCU Status
{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
   uint8_t mconpState = (ContactorState)bytes[0];       // Main positive contactor state
   uint8_t mconnState = (ContactorState)bytes[1];       // Main negative contactor state
   uint8_t mainContactorState = (ContactorState)bytes[2]; // CCS positive contactor state
   uint8_t mcuState = (ContactorState)bytes[3];         // CCS negative contactor state
   uint8_t regenState = bytes[4];                      // Regen input (0 or 1)
   // Reset timeout
   timeoutCounterHVCU = (uint8_t)(Param::GetInt(Param::CanTimeout) * 10);
}


void HVCU::ControlContactors(int opmode, CanHardware* can)
{
   static int timerCount = 0;
   
   if (++timerCount >= 10) // 10 * 10ms = 100ms
   {
      uint8_t bytes[4];
      bytes[0] = 0x39; // identifier for HVCU

      switch (opmode)
      {
         case 0: // Mode off
            bytes[1]=RELAY_OFF; // precharge relay
            bytes[2]=RELAY_OFF; // negative relay
            bytes[3]=RELAY_OFF; // positive relay
            break;

         case 2: // Mode Precharge
            bytes[1]=RELAY_ON;  // precharge relay 
            bytes[2]=RELAY_ON;  // negative relay
            bytes[3]=RELAY_OFF; // positive relay
            break;

         case 3: // Mode Precharge fail
            bytes[1]=RELAY_OFF; // precharge relay 
            bytes[2]=RELAY_OFF; // negative relay
            bytes[3]=RELAY_OFF; // positive relay
            break;

         case 1: // Mode Run
            //precharge relay stays on incase main contactor drops out with motor running, which could spike voltage and blow up the inverter
            bytes[1]=RELAY_OFF; // precharge relay
            bytes[2]=RELAY_ON; // negative relay
            bytes[3]=RELAY_ON; // positive relay
            break;

         case 4: // Mode Charge
            bytes[1]=RELAY_OFF; // precharge relay 
            bytes[2]=RELAY_ON;  // negative relay
            bytes[3]=RELAY_ON;  // positive relay
            break;
      }

      can->Send(0x397, (uint32_t*)bytes,4);
      timerCount = 0; // Reset counter
   }
}
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

#ifndef DILITHIUMMCU_H
#define DILITHIUMMCU_H

#include "bms.h"
#include "canhardware.h"
#include <stdint.h>

class DilithiumMCU: public BMS
{
   public:
      void SetCanInterface(CanHardware* c) override;
      void DecodeCAN(int id, uint8_t * data) override;
      float MaxChargeCurrent() override;
      void Task100Ms() override;
   private:
      bool BMSDataValid();
      bool ChargeAllowed();
      float chargeCurrentLimit = 0;
      float dischargeCurrentLimit = 0;
      int timeoutCounterBMS = 0;
      int timeoutCounterGFM = 0;
      float minCellV = 0;
      float maxCellV = 0;
      float minTempC = 0;
      float maxTempC = 0;
      float battVoltage = 0; // udc2 is battery voltage, udc is bus voltage
      float deltaV = 0;
      float power = 0;
      float battCurrent = 0;
      float KWh = 0;
      float MaxkWh = 0;
      float AMPh = 0;
      float SOC = 0;
      float BMS_Tavg = 0;
      float BMS_Isolation = 0;
};
#endif // DILITHIUMMCU_H

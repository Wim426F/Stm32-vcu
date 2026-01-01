/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 *               2025 Wim Boone <Github:Wim426F>
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

#ifndef BMW_E90_h
#define BMW_E90_h

/*  This library supports the Powertrain CAN messages for the BMW E90 for driving dash gauges, putting out malf lights etc
    Also reads gear lever, brake lights etc

*/

#include <stdint.h>
#include "vehicle.h"
#include "my_math.h"

class BMW_E90: public Vehicle
{
public:
   BMW_E90() : terminal15On(false), dashInit(false), gear(PARK) { }
   void SetCanInterface(CanHardware*);
   void Task10Ms();
   void Task100Ms();
   void Task200Ms();
   void SetRevCounter(int speed) { revCounter = speed; }
   void SetTemperatureGauge(float temp) { temperature = temp; }
   void DecodeCAN(int, uint32_t* data);
   bool Ready() { return terminal15On; }
   bool Start() { return terminal15On; }
   void DashOff();
   void handle130(uint32_t data[2]);
   void handle1A0(uint32_t data[2]);
   void handle2FC(uint32_t data[2]);
   void handle480(uint32_t data[2]);
   void handleB6(uint32_t data[2]);
   void handle3FE(uint32_t data[2]);
   void SetE90(bool e90) { isE90 = e90; }
   void Engine_Data();
   void SetFuelGauge(float level);

private:
   void SendAbsDscMessages(bool Brake_In);

   bool terminal15On;
   bool terminalROn;
   bool terminal50On;
   bool dashInit;
   Vehicle::gear gear;
   int revCounter;
   float temperature;
   bool  CANWake;
   bool  StartButt;
   bool isE90;
};

#endif /* BMW_E90_h */


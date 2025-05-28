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

#ifndef EVCONTROLST2C_H
#define EVCONTROLST2C_H

#include <inverter.h>

class EvControlsT2C : public Inverter
{
public:
   EvControlsT2C();
   void SetCanInterface(CanHardware* c);
   void DecodeCAN(int id, uint32_t data[2]);
   void Task10Ms();
   void Task100Ms();
   void SetTorque(float torque);
   float GetMotorTemperature() { return motor_temp; }
   float GetInverterTemperature() { return inv_temp; }
   float GetInverterVoltage() { return voltage; }
   float GetMotorSpeed() { return speed; }
   int GetInverterState() { return error; }

private:
   uint8_t run10ms;
   uint32_t lastRecv;
   int16_t inv_temp;
   int16_t speed;
   int16_t voltage;
   int16_t motor_temp;
   bool error;
   uint32_t final_torque_request;
   static float temp_1, temp_2;

   static void handle289(uint32_t data[2]);
   static void handle299(uint32_t data[2]);
};

#endif // EVCONTROLST2C_H

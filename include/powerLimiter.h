/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Wim Boone
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

#ifndef POWER_LIMITER_H
#define POWER_LIMITER_H

#include "printf.h"

/* Battery power limiter. Publishes what the pack can safely deliver and absorb
 * (BattPwrDis / BattPwrChg, kW) from the learned internal resistance plus the
 * measured worst-cell headroom and battery temperature. It only ever reports
 * the battery's own capability - the manual caps (idcmax / idcmin) are applied
 * by the consumers, which take the MIN of the two. */
namespace PowerLimiter
{
   void Init();
   void Task100Ms();

   /* Flash commit. Called from the main loop, never from the scheduler ISR:
    * a page erase stalls the bus for tens of ms. */
   void PollPersist();

   /* Terminal helpers: dump the learned tables, or restore the prefill. */
   void DumpTables(IPutChar* out);
   void ResetTables();
}

#endif

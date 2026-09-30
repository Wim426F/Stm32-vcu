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

#ifndef SOC_ESTIMATOR_H
#define SOC_ESTIMATOR_H

#include "printf.h"

/* SOC estimator for a Dilithium-managed pack.
 *
 * f is the pack's charge as a fraction of absolute capacity: 0 at 2.8 V
 * on the weakest cell, 1 at 4.2 V. Coulomb counting carries f; a rest
 * OCV on the table pulls it back. Displayed SOC / remaining Ah are the
 * usable slice between BMS_VminLimit and BMS_VmaxLimit.
 *
 * Flash (one record on the page below the power-limiter journal) holds f,
 * capacity and Wh/km. Rewritten on leaving RUN/CHARGE.
 *
 * Publishes SOC, AMPh, KWh, BMS_CapUsable, BMS_KwhUsable, BMS_KwhActual,
 * BMS_OcvCell, BMS_KmRem. */
namespace SocEstimator
{
    void Init();
    void Task100Ms();

    /* Flash commit. Called from the main loop, never from the scheduler ISR:
     * a page erase stalls the bus for tens of ms. */
    void PollPersist();

    /* Terminal helpers: dump the stored record, or erase it. */
    void DumpRecord(IPutChar* out);
    void ResetRecord();
}

#endif

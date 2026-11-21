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

#include "soc_estimator.h"
#include "params.h"
#include "hwdefs.h"
#include "my_math.h"
#include <math.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/cortex.h>

/* f = weakest-cell charge fraction (0 at 2.8 V, 1 at 4.2 V). Coulomb count,
 * OCV set at RUN/CHARGE start after 30 min in OFF, small OCV correction at
 * shorter rests. Displayed SOC is VminLimit..VmaxLimit. Flash in OFF. */

namespace SocEstimator
{

static const float CELLS = 73.0f;            // series cells
static const float V_NOM = 3.7f * CELLS;     // Ah -> Wh
static const float TICK_S = 0.1f;
static const float REST_I = 6.0f;            // |idc| below this is rest
static const uint16_t REST_TICKS = 150;      // 15 s
static const float GAIN_REST = 0.03f;        // blend toward table-f
static const float CAP_ALPHA = 0.10f;        // EWMA on a capacity sample
static const float CAP_MIN_DF = 0.25f;       // need this much Δf to size the pack
static const float CAP_TOL = 0.30f;          // reject sample >30% off current cap
static const float CAP_MIN = 50.0f;          // flash / learn sanity, matches PARAM
static const float CAP_MAX = 500.0f;
static const float CAP_DEF = 215.0f;         // BMS_CapActual default, Ah at 4200 mV in the table
static const float WHKM_ALPHA = 0.20f;       // EWMA on a 20 km window
static const float WHKM_KM = 20.0f;
static const float WHKM_VMIN = 5.0f;         // skip idle
static const float WHKM_MIN = 50.0f;         // flash / learn sanity, matches PARAM
static const float WHKM_MAX = 500.0f;
static const float WHKM_DEF = 130.0f;        // BMS_WhPerKm default
static const float SPAN_MIN = 0.05f;         // reject a degenerate usable window
static const uint16_t SETTLE_TICKS = 18000;  // 30 min in OFF: vmin is a settled OCV

/* LG INR21700-M50. Ah column / CAP_DEF so f is pack-size independent.
   Ah of 3798 to 4075 mV placed from the slopes on each side of the two gaps. */
static const int N_OCV = 23;
static const uint16_t OCV_MV[N_OCV] = {
   2800, 2956, 3038, 3118, 3134, 3167, 3228, 3255, 3281, 3335, 3360, 3368,
   3509, 3542, 3572, 3593, 3658,                       // measured
   3798, 3877, 3937,                                   // measured, 15.92 Ah and 17.57 Ah apart
   4042, 4075,                                         // measured, 15.07 Ah apart
   4200                                                // estimated
};
static const float OCV_F[N_OCV] = {
   0,         1.6f/CAP_DEF,  2.6f/CAP_DEF,  3.6f/CAP_DEF,  4.6f/CAP_DEF,  5.6f/CAP_DEF,
   6.6f/CAP_DEF, 7.6f/CAP_DEF,  9.6f/CAP_DEF,  10.6f/CAP_DEF, 11.6f/CAP_DEF, 12.1f/CAP_DEF,
   36.3f/CAP_DEF, 45.6f/CAP_DEF, 57.0f/CAP_DEF, 65.7f/CAP_DEF, 85.1f/CAP_DEF,
   118.8f/CAP_DEF, 134.7f/CAP_DEF, 152.3f/CAP_DEF,
   189.7f/CAP_DEF, 204.8f/CAP_DEF,
   1.0f
};

/* Weakest-cell rest mV -> f. Clamps outside 2.8–4.2 V.
 * Linear interpolate between the two table points that bracket mv. */
static float TableF(float mv)
{
   if (mv <= OCV_MV[0]) return OCV_F[0];
   if (mv >= OCV_MV[N_OCV - 1]) return OCV_F[N_OCV - 1];
   int i = 0;
   while (mv > OCV_MV[i + 1]) i++;   // i such that OCV_MV[i] <= mv <= OCV_MV[i+1]
   float g = (mv - OCV_MV[i]) / (OCV_MV[i + 1] - OCV_MV[i]);  // 0 at i, 1 at i+1
   return OCV_F[i] + g * (OCV_F[i + 1] - OCV_F[i]);
}

static const uint32_t SOC_MAGIC = 0x34434F53u; // "SOC4"

/* Page immediately below the power-limiter journal. */
static uint32_t SocBase()
{
   return FLASH_BASE + desig_get_flash_size() * 1024u - (uint32_t)SOC_BLKNUM * FLASH_PAGE_SIZE;
}

struct SocRecord
{
   uint32_t magic;
   float capActual, whPerKm, f;
   uint32_t crc;   // STM32 CRC of the four words before this
};

static uint32_t RecordCrc(const SocRecord* r)
{
   crc_reset();
   return crc_calculate_block((uint32_t*)(void*)r, 4);
}

static float f = -1.0f;          // -1 = no estimate
static float winLow, winSpan = 1.0f;  // usable window in f
static float counted;            // Ah out since last settled start
static uint16_t restTicks;
static bool anchorTaken;         // already corrected this rest
static uint16_t offTicks = SETTLE_TICKS;  // time in OFF; boot counts as settled, flash f may be stale
static int prevMode;
static float prevF;              // table f at previous settled start, for capacity
static bool prevValid;
static float whAcc, kmAcc;       // 20 km Wh/km window
static float storedCap, storedWhk, storedF = -1.0f;  // last flash commit

/* Restore f, cap, Wh/km. Blank page keeps parm_load defaults. */
static void LoadFromFlash()
{
   const SocRecord* r = (const SocRecord*)SocBase();
   if (r->magic == SOC_MAGIC && RecordCrc(r) == r->crc
       && r->capActual >= CAP_MIN && r->capActual <= CAP_MAX
       && r->whPerKm >= WHKM_MIN && r->whPerKm <= WHKM_MAX)
   {
      Param::SetFloat(Param::BMS_CapActual, r->capActual);
      Param::SetFloat(Param::BMS_WhPerKm, r->whPerKm);
      storedCap = r->capActual;
      storedWhk = r->whPerKm;
      if (r->f >= 0.0f && r->f <= 1.0f) f = storedF = r->f;
      return;
   }
   /* Blank or corrupt page: keep parm_load values. */
   storedCap = Param::GetFloat(Param::BMS_CapActual);
   storedWhk = Param::GetFloat(Param::BMS_WhPerKm);
   if (storedCap < CAP_MIN || storedCap > CAP_MAX)
      Param::SetFloat(Param::BMS_CapActual, storedCap = CAP_DEF);
   if (storedWhk < WHKM_MIN || storedWhk > WHKM_MAX)
      Param::SetFloat(Param::BMS_WhPerKm, storedWhk = WHKM_DEF);
}

/* Write the record. Erase stalls the bus; IRQs off, kick the watchdog. */
static void Commit()
{
   SocRecord ram = {};
   ram.magic = SOC_MAGIC;
   ram.capActual = Param::GetFloat(Param::BMS_CapActual);
   ram.whPerKm = Param::GetFloat(Param::BMS_WhPerKm);
   ram.f = (f >= 0.0f && f <= 1.0f) ? f : -1.0f;
   ram.crc = RecordCrc(&ram);

   uint32_t dest = SocBase();
   iwdg_reset();
   cm_disable_interrupts();
   flash_unlock();
   flash_set_ws(2);
   iwdg_reset();
   flash_erase_page(dest);
   for (unsigned i = 0; i < sizeof(ram) / 4; i++)
      flash_program_word(dest + i * 4, ((const uint32_t*)&ram)[i]);
   flash_lock();
   cm_enable_interrupts();
   iwdg_reset();

   storedCap = ram.capActual;
   storedWhk = ram.whPerKm;
   storedF = ram.f;
}

/* Only in OFF: contactors open. The erase stalls the bus with IRQs off. */
void PollPersist()
{
   if (Param::GetInt(Param::opmode) != MOD_OFF) return;
   float cap = Param::GetFloat(Param::BMS_CapActual);
   float whk = Param::GetFloat(Param::BMS_WhPerKm);
   if (fabsf(cap - storedCap) <= 0.5f && fabsf(whk - storedWhk) <= 2.0f
       && (f < 0.0f || fabsf(f - storedF) <= 1.0e-4f))
      return;
   Commit();
}

/* Capacity = Ah discharged / Δf between two settled starts (30 min in OFF).
 * Only on discharge (counted > 0, f dropped). Sample must span CAP_MIN_DF
 * and land within CAP_TOL of the current estimate, then EWMA in. */
static void LearnCapacity(float fNow, float cap)
{
   if (prevValid && (prevF - fNow) >= CAP_MIN_DF && counted > 0.0f)
   {
      float capNew = counted / (prevF - fNow);
      if (capNew > cap * (1.0f - CAP_TOL) && capNew < cap * (1.0f + CAP_TOL))
         Param::SetFloat(Param::BMS_CapActual, cap + CAP_ALPHA * (capNew - cap));
   }
   prevF = fNow;       // this rest's OCV becomes the next pair's start
   prevValid = true;
   counted = 0.0f;
}

/* SOC, remaining Ah/kWh/km, usable and actual capacity. */
static void Publish()
{
   /* Driver-empty / driver-full in f. Degenerate limit settings keep the old window. */
   float fL = TableF(Param::GetFloat(Param::BMS_VminLimit) * 1000.0f);
   float fH = TableF(Param::GetFloat(Param::BMS_VmaxLimit) * 1000.0f);
   if (fH - fL > SPAN_MIN) { winLow = fL; winSpan = fH - fL; }

   float cap = Param::GetFloat(Param::BMS_CapActual);
   float kwhScale = V_NOM / 1000.0f;   // nominal pack V, not live udc
   float ahUsable = winSpan * cap;
   Param::SetFloat(Param::BMS_CapUsable, ahUsable);
   Param::SetFloat(Param::BMS_KwhUsable, ahUsable * kwhScale);
   Param::SetFloat(Param::BMS_KwhActual, cap * kwhScale);
   if (f < 0.0f) return;

   float ahRem = MAX(f - winLow, 0.0f) * cap;
   float kwhRem = ahRem * kwhScale;
   float whk = Param::GetFloat(Param::BMS_WhPerKm);
   float kmRem = (whk > 0.0f) ? kwhRem * 1000.0f / whk : 0.0f;
   Param::SetFloat(Param::AMPh, ahRem);
   Param::SetFloat(Param::KWh, kwhRem);
   Param::SetFloat(Param::BMS_KmRem, MAX(MIN(kmRem, 999.0f), 0.0f));
   Param::SetFloat(Param::SOC, MAX(MIN(100.0f * (f - winLow) / winSpan, 100.0f), 0.0f));  // 0% at VminLimit
}

/* Coulomb count, OCV correction, Wh/km learn, publish. */
void Task100Ms()
{
   int mode = Param::GetInt(Param::opmode);
   /* Contactors are open in OFF, so time in OFF is rest time, also while the BMS sleeps. */
   if (mode == MOD_OFF && offTicks < SETTLE_TICKS) offTicks++;

   /* Weakest cell. Dilithium publishes mV; some BMS send volts. */
   float vmin = Param::GetFloat(Param::BMS_Vmin);
   if (vmin < 1000.0f) vmin *= 1000.0f;
   if (vmin < 1500.0f || vmin > 4500.0f)
   {
      Publish();   // keep showing f so the dash never drops to 0
      return;
   }

   /* Coulomb count. idc is charge-positive, so dAh > 0 charges the pack.
    * counted is Ah out: discharge makes dAh negative, so -= increases it.
    * Not in OFF: contactors are open, any idc there is sensor offset. */
   float cap = Param::GetFloat(Param::BMS_CapActual);
   float idc = Param::GetFloat(Param::idc);
   if (mode != MOD_OFF && f >= 0.0f && cap >= CAP_MIN)
   {
      float dAh = idc * TICK_S / 3600.0f;   // A * s / 3600 = Ah this tick
      f += dAh / cap;                       // fraction of absolute capacity
      counted -= dAh;
      f = MAX(MIN(f, 1.0f), 0.0f);
   }

   /* Rest = |idc| < REST_I for REST_TICKS. Load resets the timer. */
   if (fabsf(idc) < REST_I) { if (restTicks < REST_TICKS) restTicks++; }
   else { restTicks = 0; anchorTaken = false; }

   /* Display-only IR-compensated cell voltage. rpack is pack mOhm, vmin is
    * cell mV; /CELLS. idc charge-positive: minus a negative discharge adds sag. */
   float rpack = Param::GetFloat(Param::BMS_Rpack);
   float ocv = vmin;
   if (rpack >= 5.0f && rpack <= 2000.0f)
      ocv = vmin - idc * rpack / CELLS;
   Param::SetFloat(Param::BMS_OcvCell, ocv);

   /* Start of RUN or CHARGE. prevMode only moves on valid ticks, so a BMS that
    * answers late still gets the start check on its first valid vmin. */
   bool live = (mode == MOD_RUN || mode == MOD_CHARGE);
   bool started = live && prevMode != MOD_RUN && prevMode != MOD_CHARGE;
   prevMode = mode;

   /* After 30 min in OFF, vmin is a settled OCV: set f and take a capacity
    * sample. A shorter stop or a 15 s rest still sags 20-40 mV, so it only
    * moves f GAIN_REST toward the table (sets it if f is unknown), once per rest. */
   float fNow = TableF(vmin);
   if (started && offTicks >= SETTLE_TICKS)
   {
      f = fNow;
      LearnCapacity(fNow, cap);
      anchorTaken = true;
   }
   else if ((started || restTicks >= REST_TICKS) && !anchorTaken)
   {
      f = (f < 0.0f) ? fNow : f * (1.0f - GAIN_REST) + fNow * GAIN_REST;
      anchorTaken = true;
   }
   if (started) offTicks = 0;

   /* Net pack Wh per km while moving. -idc so regen credits. */
   if (mode == MOD_RUN)
   {
      float speed = Param::GetFloat(Param::Veh_Speed);
      if (speed >= WHKM_VMIN)
      {
         whAcc += Param::GetFloat(Param::udc2) * (-idc) * TICK_S / 3600.0f;
         kmAcc += speed * TICK_S / 3600.0f;
      }
      if (kmAcc >= WHKM_KM)
      {
         float inst = whAcc / kmAcc;
         if (inst >= WHKM_MIN && inst <= WHKM_MAX)
         {
            float whk = Param::GetFloat(Param::BMS_WhPerKm);
            Param::SetFloat(Param::BMS_WhPerKm, whk + WHKM_ALPHA * (inst - whk));
         }
         whAcc = kmAcc = 0.0f;
      }
   }

   Publish();
}

/* Publish straight from flash so the dash has SOC before the BMS wakes. */
void Init()
{
   LoadFromFlash();
   Publish();
}

void DumpRecord(IPutChar* out)
{
   fprintf(out, "f=%f capActual=%f whkm=%f\r\n",
           f, Param::Get(Param::BMS_CapActual), Param::Get(Param::BMS_WhPerKm));
}

void ResetRecord()
{
   Param::SetFloat(Param::BMS_CapActual, CAP_DEF);
   Param::SetFloat(Param::BMS_WhPerKm, WHKM_DEF);
   f = -1.0f;
   counted = 0.0f;
   prevValid = false;
   Commit();
}

}

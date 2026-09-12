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

#include "powerLimiter.h"
#include "params.h"
#include "hwdefs.h"
#include "my_math.h"
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/cortex.h>

/* Battery power limiter for a Dilithium-managed pack.
 *
 * What the pack can give or take is limited by how far the *worst* cell is from
 * its floor/ceiling, divided by the pack's internal resistance at the present
 * temperature, SOC and C-rate. R is held in two small 3D tables (one per
 * direction) that are learned in the background and kept in flash.
 *
 * This module reports the battery's capability only. The user's power
 * preferences (PwrMotMax / PwrRegenMax / PwrCcsMax / PwrAcMax) are applied by the
 * consumers, which take MIN(preference, what we publish here). We only reduce.
 *
 * Runs at 100 ms because that is how often the BMS publishes: every input we
 * read is written by DilithiumMCU::Task100Ms, and 0x293 is multiplexed on
 * data[0] so the cell-voltage sub-frame can land slower still.
 */

namespace PowerLimiter
{

/* ---- table geometry: 5 temperatures x 3 SOC x 5 C-rates, per direction ---- */
static const int NT = 5;
static const int NS = 3;
static const int NC = 5;
static const int NBINS = NT * NS * NC; // 75
static const float TAXIS[NT] = { -10.0f, 0.0f, 25.0f, 40.0f, 50.0f };
static const float SAXIS[NS] = { 10.0f, 50.0f, 90.0f };
static const float CAXIS[NC] = { 0.5f, 1.0f, 2.0f, 3.0f, 4.0f };

/* ---- learning ---- */
static const float EWMA_ALPHA = 0.05f;   // heavy: pack IR moves over months
static const float R_MIN = 8.0f;         // mOhm, pack
static const float R_MAX = 2000.0f;
static const float R_OUTLIER = 3.0f;     // reject > 3x or < 1/3 of the stored bin
static const float I_REST = 8.0f;        // |idc| under this counts as resting
static const float I_LEARN_MIN = 15.0f;  // need this much load to measure at all
static const float I_STABLE_BAND = 8.0f; // peak-to-peak allowed across the window
static const float DI_MIN = 25.0f;       // plateau-to-plateau step worth using
static const float DV_CELL_MIN = 0.005f; // below this it is all measurement noise
static const uint16_t STABLE_TICKS = 15; // 1.5 s
static const uint16_t REST_TICKS = 20;   // 2.0 s
static const uint16_t REF_MAX_AGE = 600; // 60 s - a reference older than this has
                                         // drifted in OCV and would inflate R
static const float REF_MAX_DSOC = 2.0f;  // ...as has one taken 2% of SOC ago

/* ---- output ---- */
static const float PWR_SLEW = 5.0f;      // kW per tick -> 50 kW/s
static const float PWR_NOLIMIT = 1000.0f;// "battery imposes nothing" - consumers clamp
static const float COLD_SPAN = 5.0f;     // degrees below BMS_TminLimit to reach 0 charge
static const float T_HOT_FLOOR_SPAN = 0.1f;

/* Series cell count. Fixed for this pack; DilithiumMCU hardcodes the same 73. */
static const float CELLS = 73.0f;

enum LimSrc { SRC_NONE = 0, SRC_SAG, SRC_DRAG, SRC_HOT, SRC_COLD, SRC_NODATA };

/* ---- flash journal: 2 pages x 6 slots, below the param/CAN pages ---- */
static const uint32_t PE_MAGIC = 0x52454646u; // 'REFF'
static const uint16_t PE_VERSION = 1;
static const int PE_SLOTS = 6;
static const uint32_t PE_SLOT_SIZE = 320;

struct PeSlot
{
   uint32_t magic;
   uint32_t generation;
   uint16_t version;
   uint16_t reserved;
   uint16_t dis[NBINS];
   uint16_t chg[NBINS];
   uint32_t crc;
};

static_assert(sizeof(PeSlot) <= PE_SLOT_SIZE, "PeSlot does not fit journal slot");

/* Everything the tick needs, read from the param database exactly once. */
struct Inputs
{
   float vMinCell;
   float vMaxCell;
   float vFloor;   // BMS_VsagLimit
   float vCeil;    // BMS_VdragLimit
   float tMin;
   float tMax;
   float t;        // representative pack temperature for the table lookup
   float soc;
   float idc;      // + charge, - discharge (Dilithium / i3LIM convention)
   float absI;
   float vPack;
   float ah;
   bool  haveCells;
};

/* ---- state ---- */
static float rDis[NBINS];
static float rChg[NBINS];
static float pwrDis;
static float pwrChg;
static float rUsed;
static float iDisMax;
static float iChgMax;
static int   limSrc;

static volatile bool dirty;
static volatile bool persistWanted;
static int prevMode = -1;
static uint32_t lastGen;
static int lastPage;
static int lastSlot;

/* A voltage reference to subtract from the loaded cell voltage. Either an
 * open-circuit sample (rest) or an earlier steady load (plateau). Both go stale:
 * once SOC has moved, the difference is mostly OCV drift, not IR drop. */
struct Ref
{
   bool     valid;
   uint16_t age;      // ticks
   float    soc;
   float    vMinCell;
   float    vMaxCell;
   float    absI;
   bool     charging;
};

static Ref rest;
static Ref plat;
static uint16_t restTicks;
static uint16_t stableTicks;
static float iWinLo;
static float iWinHi;

/* ------------------------------------------------------------------ helpers */

static uint16_t PackR(float mohm)
{
   if (mohm < 0.0f) mohm = 0.0f;
   if (mohm > 6553.4f) mohm = 6553.4f;
   return (uint16_t)(mohm * 10.0f + 0.5f);
}

static float UnpackR(uint16_t raw)
{
   return (float)raw * 0.1f;
}

static int BinIndex(int it, int is, int ic)
{
   return it * (NS * NC) + is * NC + ic;
}

/* Locate x on an axis: the two bracketing bins and the fraction between them.
 * Off either end clamps to the end bin with fraction 0. */
static void Bracket(const float* axis, int n, float x, int* i0, int* i1, float* f)
{
   if (x <= axis[0])
   {
      *i0 = 0;
      *i1 = 0;
      *f = 0.0f;
      return;
   }
   if (x >= axis[n - 1])
   {
      *i0 = n - 1;
      *i1 = n - 1;
      *f = 0.0f;
      return;
   }
   for (int i = 0; i < n - 1; i++)
   {
      if (x <= axis[i + 1])
      {
         *i0 = i;
         *i1 = i + 1;
         *f = (x - axis[i]) / (axis[i + 1] - axis[i]);
         return;
      }
   }
}

static int Nearest(const float* axis, int n, float x)
{
   int best = 0;
   float bd = ABS(x - axis[0]);
   for (int i = 1; i < n; i++)
   {
      float d = ABS(x - axis[i]);
      if (d < bd)
      {
         bd = d;
         best = i;
      }
   }
   return best;
}

/* Trilinear over T / SOC / C-rate. */
static float Lookup(const float* table, float t, float soc, float crate)
{
   int t0, t1, s0, s1, c0, c1;
   float ft, fs, fc;
   Bracket(TAXIS, NT, t, &t0, &t1, &ft);
   Bracket(SAXIS, NS, soc, &s0, &s1, &fs);
   Bracket(CAXIS, NC, crate, &c0, &c1, &fc);

   float c00 = table[BinIndex(t0, s0, c0)] * (1.0f - fc) + table[BinIndex(t0, s0, c1)] * fc;
   float c01 = table[BinIndex(t0, s1, c0)] * (1.0f - fc) + table[BinIndex(t0, s1, c1)] * fc;
   float c10 = table[BinIndex(t1, s0, c0)] * (1.0f - fc) + table[BinIndex(t1, s0, c1)] * fc;
   float c11 = table[BinIndex(t1, s1, c0)] * (1.0f - fc) + table[BinIndex(t1, s1, c1)] * fc;
   float lo = c00 * (1.0f - fs) + c01 * fs;
   float hi = c10 * (1.0f - fs) + c11 * fs;
   float r = lo * (1.0f - ft) + hi * ft;
   return (r < R_MIN) ? R_MIN : r;
}

/* Straight line from (x0,y0) to (x1,y1), evaluated at x. x1 may be below x0. */
static float Lerp(float x, float x0, float x1, float y0, float y1)
{
   if (x1 == x0) return y1;
   return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

/* --------------------------------------------------------------- the tables */

static void Prefill()
{
   // Conservative physics-based starting point for pack R_eff (mOhm).
   // Axes: T {-10,0,25,40,50}, SOC {10,50,90}, C {0.5,1,2,3,4}.
   static const float init[NT][NS][NC] =
   {
      { { 62.0f, 66.0f, 72.0f, 78.0f, 84.0f },
        { 54.0f, 58.0f, 64.0f, 70.0f, 76.0f },
        { 62.0f, 66.0f, 72.0f, 78.0f, 84.0f } },
      { { 40.0f, 43.0f, 48.0f, 53.0f, 58.0f },
        { 34.0f, 36.0f, 41.0f, 46.0f, 51.0f },
        { 40.0f, 43.0f, 48.0f, 53.0f, 58.0f } },
      { { 22.0f, 24.0f, 28.0f, 32.0f, 36.0f },
        { 20.0f, 22.0f, 26.0f, 30.0f, 34.0f },
        { 22.0f, 24.0f, 28.0f, 32.0f, 36.0f } },
      { { 16.0f, 18.0f, 22.0f, 26.0f, 30.0f },
        { 15.0f, 17.0f, 21.0f, 25.0f, 29.0f },
        { 16.0f, 18.0f, 22.0f, 26.0f, 30.0f } },
      { { 13.0f, 15.0f, 19.0f, 23.0f, 26.0f },
        { 12.5f, 14.5f, 18.5f, 22.5f, 25.5f },
        { 13.0f, 15.0f, 19.0f, 23.0f, 26.0f } }
   };

   for (int it = 0; it < NT; it++)
      for (int is = 0; is < NS; is++)
         for (int ic = 0; ic < NC; ic++)
         {
            int i = BinIndex(it, is, ic);
            rDis[i] = init[it][is][ic];
            rChg[i] = init[it][is][ic];
         }
}

/* ----------------------------------------------------------------- flash I/O */

static uint32_t PageAddr(int page)
{
   uint32_t flashSize = desig_get_flash_size();
   return FLASH_BASE + flashSize * 1024u - (uint32_t)(PE_BLKNUM + page) * FLASH_PAGE_SIZE;
}

static const PeSlot* SlotAt(int page, int slot)
{
   return (const PeSlot*)(PageAddr(page) + (uint32_t)slot * PE_SLOT_SIZE);
}

static bool SlotBlank(const PeSlot* s)
{
   const uint32_t* w = (const uint32_t*)s;
   uint32_t acc = 0xFFFFFFFFu;
   for (unsigned i = 0; i < PE_SLOT_SIZE / 4; i++)
      acc &= w[i];
   return acc == 0xFFFFFFFFu;
}

static uint32_t SlotCrc(const PeSlot* s)
{
   crc_reset();
   return crc_calculate_block((uint32_t*)(void*)s, (int)((sizeof(PeSlot) - sizeof(uint32_t)) / 4));
}

static bool SlotValid(const PeSlot* s)
{
   if (s->magic != PE_MAGIC) return false;
   if (s->version != PE_VERSION) return false;
   return SlotCrc(s) == s->crc;
}

static bool LoadFromFlash()
{
   lastGen = 0;
   lastPage = -1;
   lastSlot = -1;
   const PeSlot* best = 0;

   for (int p = 0; p < PE_PAGES; p++)
   {
      for (int s = 0; s < PE_SLOTS; s++)
      {
         const PeSlot* sl = SlotAt(p, s);
         if (!SlotValid(sl)) continue;
         if (lastPage < 0 || (int32_t)(sl->generation - lastGen) > 0)
         {
            lastGen = sl->generation;
            lastPage = p;
            lastSlot = s;
            best = sl;
         }
      }
   }

   if (!best) return false;

   for (int i = 0; i < NBINS; i++)
   {
      rDis[i] = UnpackR(best->dis[i]);
      rChg[i] = UnpackR(best->chg[i]);
   }
   dirty = false;
   return true;
}

/* Append the next generation. Slots fill in order; when a page is full we move
 * to the other one and erase it, so the newest record on the opposite page
 * always survives a power loss mid-commit. */
static void CommitFlash()
{
   int page;
   int slot;

   if (lastPage < 0)
   {
      page = 0;
      slot = 0;
   }
   else if (lastSlot + 1 < PE_SLOTS)
   {
      page = lastPage;
      slot = lastSlot + 1;
   }
   else
   {
      page = (lastPage == 0) ? 1 : 0;
      slot = 0;
   }

   uint32_t pageBase = PageAddr(page);
   const PeSlot* dest = SlotAt(page, slot);
   bool needErase = false;
   if (!SlotBlank(dest))
   {
      needErase = true;
      slot = 0;
      dest = SlotAt(page, slot);
   }

   PeSlot ram;
   for (unsigned i = 0; i < sizeof(ram); i++)
      ((uint8_t*)&ram)[i] = 0;
   ram.magic = PE_MAGIC;
   ram.generation = lastGen + 1;
   ram.version = PE_VERSION;
   ram.reserved = 0;
   for (int i = 0; i < NBINS; i++)
   {
      ram.dis[i] = PackR(rDis[i]);
      ram.chg[i] = PackR(rChg[i]);
   }
   ram.crc = SlotCrc(&ram);

   iwdg_reset();
   cm_disable_interrupts();
   flash_unlock();
   flash_set_ws(2);
   if (needErase)
   {
      iwdg_reset();
      flash_erase_page(pageBase);
   }

   const uint32_t* src = (const uint32_t*)&ram;
   uint32_t addr = (uint32_t)dest;
   unsigned words = (sizeof(PeSlot) + 3) / 4;
   for (unsigned i = 0; i < words; i++)
      flash_program_word(addr + i * 4, src[i]);
   flash_lock();
   cm_enable_interrupts();
   iwdg_reset();

   if (SlotValid(dest))
   {
      lastGen = dest->generation;
      lastPage = page;
      lastSlot = slot;
   }
}

void PollPersist()
{
   if (!persistWanted) return;

   /* Only one thing sets persistWanted on a normal drive: the live -> not-live
    * transition in Task100Ms, so this runs once per journey. The guard below is
    * not a second trigger - it is a refusal, for the one other caller
    * (ResetTables, from the pereset command) which can fire at any time. Erasing
    * a page stops the bus for tens of ms, so defer it to shutdown by leaving the
    * flag set. The normal trigger is never blocked: by the time it fires, opmode
    * has already left RUN/CHARGE. */
   int mode = Param::GetInt(Param::opmode);
   if (mode == MOD_RUN || mode == MOD_CHARGE) return;

   persistWanted = false;
   /* Clear before writing, not after: anything the scheduler learns while the
    * erase is in progress must still mark the table dirty. */
   if (!dirty) return;
   dirty = false;
   CommitFlash();
}

/* ------------------------------------------------------------------ learning */

static void RefUpdate(Ref* r, const Inputs& in)
{
   r->valid = true;
   r->age = 0;
   r->soc = in.soc;
   r->vMinCell = in.vMinCell;
   r->vMaxCell = in.vMaxCell;
   r->absI = in.absI;
   r->charging = in.idc > 0.0f;
}

static void RefAge(Ref* r, const Inputs& in)
{
   if (!r->valid) return;
   if (r->age < 0xFFFF) r->age++;
   if (r->age > REF_MAX_AGE || ABS(in.soc - r->soc) > REF_MAX_DSOC)
      r->valid = false;
}

/* EWMA the measurement into the single nearest bin. Reject anything that is not
 * within a factor of R_OUTLIER of what that bin already holds - a good
 * measurement never moves pack IR by 3x. */
static void Learned(float* table, const Inputs& in, float crate, float rMeas)
{
   if (rMeas < R_MIN || rMeas > R_MAX) return;

   int i = BinIndex(Nearest(TAXIS, NT, in.t),
                    Nearest(SAXIS, NS, in.soc),
                    Nearest(CAXIS, NC, crate));

   if (rMeas > table[i] * R_OUTLIER || rMeas < table[i] / R_OUTLIER) return;

   float next = table[i] * (1.0f - EWMA_ALPHA) + rMeas * EWMA_ALPHA;
   next = MIN(MAX(next, R_MIN), R_MAX);
   if (next != table[i])
   {
      table[i] = next;
      dirty = true;
   }
}

static void Learn(const Inputs& in)
{
   if (!in.haveCells) return;

   RefAge(&rest, in);
   RefAge(&plat, in);

   /* Resting: keep refreshing an open-circuit reference. */
   if (in.absI < I_REST)
   {
      if (restTicks < REST_TICKS) restTicks++;
      if (restTicks >= REST_TICKS) RefUpdate(&rest, in);
   }
   else
   {
      restTicks = 0;
   }

   /* Stable current means peak-to-peak inside the band across the whole window,
    * which a slow ramp fails - unlike comparing against a fast-tracking filter. */
   if (stableTicks > 0 &&
       (MAX(iWinHi, in.idc) - MIN(iWinLo, in.idc)) > I_STABLE_BAND)
   {
      stableTicks = 0;
   }
   if (stableTicks == 0)
   {
      iWinLo = in.idc;
      iWinHi = in.idc;
      stableTicks = 1;
   }
   else
   {
      iWinLo = MIN(iWinLo, in.idc);
      iWinHi = MAX(iWinHi, in.idc);
      if (stableTicks < 0xFFFF) stableTicks++;
   }

   if (stableTicks < STABLE_TICKS || in.absI < I_LEARN_MIN) return;

   bool charging = in.idc > 0.0f;
   float rMeas = 0.0f;
   bool got = false;

   /* Prefer plateau-to-plateau: dV/dI cancels OCV entirely, so it stays honest
    * even when the pack has moved a long way from any resting sample.
    * Both deltas are signed and must agree - more current has to mean more
    * voltage drop. If they disagree, OCV drift is outrunning the IR change and
    * the pair is junk, so throw it away rather than abs() it into looking sane. */
   if (plat.valid && plat.charging == charging)
   {
      float di = in.absI - plat.absI;
      float dv = charging ? (in.vMaxCell - plat.vMaxCell)
                          : (plat.vMinCell - in.vMinCell);
      if (ABS(di) >= DI_MIN && ABS(dv) >= DV_CELL_MIN && dv * di > 0.0f)
      {
         rMeas = (ABS(dv) * CELLS * 1000.0f) / ABS(di);
         got = true;
      }
   }

   if (!got && rest.valid)
   {
      float dv = charging ? (in.vMaxCell - rest.vMaxCell)
                          : (rest.vMinCell - in.vMinCell);
      if (dv >= DV_CELL_MIN)
      {
         rMeas = (dv * CELLS * 1000.0f) / in.absI;
         got = true;
      }
   }

   float crate = (in.ah > 1.0f) ? (in.absI / in.ah) : CAXIS[0];
   if (got)
      Learned(charging ? rChg : rDis, in, crate, rMeas);

   /* This steady stretch becomes the reference for the next one. */
   RefUpdate(&plat, in);
   stableTicks = 0;
}

/* ------------------------------------------------------------------ limiting */

/* How much current drags the worst cell from where it is now to its limit.
 *
 * R depends on C-rate, and the C-rate depends on the answer, so solve it:
 * guess from R at the present rate, recompute the rate, look R up again. Three
 * passes is plenty - without this, a lookup at rest lands on the 0.5C bin (the
 * lowest R in the table) and predicts thousands of amps.
 */
static float HeadroomCurrent(const float* table, const Inputs& in,
                             float headCell, float iNow, float* rOut)
{
   float crate = (in.ah > 1.0f) ? (in.absI / in.ah) : CAXIS[0];
   float iTotal = iNow;
   float r = R_MIN;

   for (int pass = 0; pass < 3; pass++)
   {
      r = Lookup(table, in.t, in.soc, crate);
      iTotal = iNow + (headCell * CELLS) / (r * 0.001f);
      if (in.ah > 1.0f) crate = iTotal / in.ah;
   }

   *rOut = r;
   return iTotal;
}

/* Linear derate to floorKw between tStart and tEnd, held at floorKw beyond.
 * Only ever reduces. */
static float TempDerate(float p, float t, float tStart, float tEnd, float floorKw)
{
   if (t <= tStart) return p;
   float ramp = (t >= tEnd) ? floorKw : Lerp(t, tStart, tEnd, p, floorKw);
   return MIN(p, ramp);
}

static void Compute(const Inputs& in)
{
   /* No cell data - BMS timed out, or this BMS does not report cells. Say the
    * battery imposes nothing and let the user's preferences rule; publishing 0
    * here would cut drive power to nothing on a momentary CAN dropout. */
   if (!in.haveCells)
   {
      pwrDis = PWR_NOLIMIT;
      pwrChg = PWR_NOLIMIT;
      rUsed = 0.0f;
      iDisMax = 0.0f;
      iChgMax = 0.0f;
      limSrc = SRC_NODATA;
      return;
   }

   float iNowDis = (in.idc < 0.0f) ? -in.idc : 0.0f;
   float iNowChg = (in.idc > 0.0f) ? in.idc : 0.0f;
   float vFloorPack = in.vFloor * CELLS;
   float vCeilPack = in.vCeil * CELLS;
   int src = SRC_NONE;

   /* Discharge: power at the moment the worst cell reaches its floor, so the
    * voltage in P = I * V is the floor itself. (The old code computed the same
    * number the long way round - vmin*n - iRem*R is vFloor*n by construction.) */
   float crateNow = (in.ah > 1.0f) ? (in.absI / in.ah) : CAXIS[0];
   float rD = Lookup(rDis, in.t, in.soc, crateNow);
   float dis = 0.0f;
   float headDis = in.vMinCell - in.vFloor;
   if (headDis > 0.0f)
   {
      float i = HeadroomCurrent(rDis, in, headDis, iNowDis, &rD);
      dis = i * vFloorPack / 1000.0f;
   }
   else
   {
      src = SRC_SAG;
   }
   /* Published even in the SAG branch, where it is the R at the present rate
    * rather than at the limit - otherwise it would read a bare R_MIN. */
   rUsed = rD;

   /* Charge: mirror image against the ceiling. */
   float rC = R_MIN;
   float chg = 0.0f;
   float headChg = in.vCeil - in.vMaxCell;
   if (headChg > 0.0f)
   {
      float i = HeadroomCurrent(rChg, in, headChg, iNowChg, &rC);
      chg = i * vCeilPack / 1000.0f;
   }
   else if (src == SRC_NONE)
   {
      src = SRC_DRAG;
   }

   /* Hot ramp on tMax. Both directions taper to BMS_PwrHot, not to zero.
    * 152 kW at 40 C = 502 A = 5.6 kW I2R. Must already be derating.
    *  15 kW at 50 C =  50 A =   31 W    = Negligible.
    * 15 kW holds highway speed, or adds 15 kWh/h with charging.
    * Charge starts earlier (BMS_TderateChg) because cell temp sensor lags.
    * Sag/drag still pull below the floor - TempDerate only reduces. */
   float tHotLimit = Param::GetFloat(Param::BMS_TmaxLimit);
   float pwrHot = Param::GetFloat(Param::BMS_PwrHot);

   float tDisStart = Param::GetFloat(Param::BMS_Tderate);
   if (tHotLimit < tDisStart + T_HOT_FLOOR_SPAN)
      tDisStart = tHotLimit - T_HOT_FLOOR_SPAN;

   float tChgStart = Param::GetFloat(Param::BMS_TderateChg);
   if (tHotLimit < tChgStart + T_HOT_FLOOR_SPAN)
      tChgStart = tHotLimit - T_HOT_FLOOR_SPAN;

   if (in.tMax > tDisStart)
   {
      dis = TempDerate(dis, in.tMax, tDisStart, tHotLimit, pwrHot);
      if (src == SRC_NONE) src = SRC_HOT;
   }
   if (in.tMax > tChgStart)
   {
      chg = TempDerate(chg, in.tMax, tChgStart, tHotLimit, pwrHot);
      if (src == SRC_NONE) src = SRC_HOT;
   }

   /* Cold, charge only. Plating is real, but discharge must stay available so
    * the car can be driven. Ramped, not stepped, so regen does not vanish in a
    * single tick mid-drive. */
   float tColdLimit = Param::GetFloat(Param::BMS_TminLimit);
   if (in.tMin < tColdLimit)
   {
      float ramp = (in.tMin <= tColdLimit - COLD_SPAN)
                   ? 0.0f
                   : Lerp(in.tMin, tColdLimit, tColdLimit - COLD_SPAN, chg, 0.0f);
      chg = MIN(chg, ramp);
      if (src == SRC_NONE) src = SRC_COLD;
   }

   dis = MAX(dis, 0.0f);
   chg = MAX(chg, 0.0f);

   /* Slew the limit upwards only. The worst cell is noisy and the DU sees this
    * every 500 ms, so ramping up stops us chasing that noise - but a reduction
    * is the safe direction and has to land immediately. It also means recovering
    * from SRC_NODATA drops straight to the real limit instead of leaving the
    * pack unprotected while a 1000 kW placeholder ramps down. */
   pwrDis = (dis < pwrDis) ? dis : MIN(dis, pwrDis + PWR_SLEW);
   pwrChg = (chg < pwrChg) ? chg : MIN(chg, pwrChg + PWR_SLEW);

   /* The same limits as current, for display. Divided by the limit voltage
    * because that is the voltage the power figure was built on. */
   iDisMax = pwrDis * 1000.0f / vFloorPack;
   iChgMax = pwrChg * 1000.0f / vCeilPack;
   limSrc = src;
}

/* ---------------------------------------------------------------------- tick */

void Task100Ms()
{
   Inputs in;

   /* Dilithium publishes cell voltages in mV, some other BMS drivers in volts. */
   float rawMin = Param::GetFloat(Param::BMS_Vmin);
   float rawMax = Param::GetFloat(Param::BMS_Vmax);
   in.vMinCell = (rawMin > 10.0f) ? rawMin * 0.001f : rawMin;
   in.vMaxCell = (rawMax > 10.0f) ? rawMax * 0.001f : rawMax;
   in.haveCells = in.vMinCell > 0.5f && in.vMaxCell > 0.5f;

   in.vFloor = Param::GetFloat(Param::BMS_VsagLimit);
   in.vCeil = Param::GetFloat(Param::BMS_VdragLimit);
   in.tMin = Param::GetFloat(Param::BMS_Tmin);
   in.tMax = Param::GetFloat(Param::BMS_Tmax);
   in.t = Param::GetFloat(Param::BMS_Tavg);
   in.soc = Param::GetFloat(Param::SOC);
   in.idc = Param::GetFloat(Param::idc);
   in.absI = ABS(in.idc);

   /* udc2 is the battery, udc the bus. Prefer the battery's own reading. */
   in.vPack = Param::GetFloat(Param::udc2);
   if (in.vPack < 1.0f) in.vPack = Param::GetFloat(Param::udc);

   float kwh = Param::GetFloat(Param::BattCap);
   in.ah = (kwh * 1000.0f) / (CELLS * 3.7f);

   Learn(in);
   Compute(in);

   Param::SetFloat(Param::BMS_MaxOutput, pwrDis);
   Param::SetFloat(Param::BMS_MaxInput, pwrChg);
   Param::SetFloat(Param::BMS_IdisMax, iDisMax);
   Param::SetFloat(Param::BMS_IchgMax, iChgMax);
   Param::SetFloat(Param::BMS_Rpack, rUsed);
   Param::SetInt(Param::BMS_LimSrc, limSrc);

   /* Save on the way out of a live mode. Never on a timer while driving: the
    * page erase in CommitFlash runs with interrupts off and stalls the bus for
    * tens of ms, and the table only moves over weeks anyway. */
   int mode = Param::GetInt(Param::opmode);
   bool live = (mode == MOD_RUN || mode == MOD_CHARGE);
   bool wasLive = (prevMode == MOD_RUN || prevMode == MOD_CHARGE);
   if (wasLive && !live && dirty)
      persistWanted = true;
   prevMode = mode;
}

void Init()
{
   Prefill();
   LoadFromFlash();
   persistWanted = false;
   prevMode = -1;
   rest.valid = false;
   plat.valid = false;
   restTicks = 0;
   stableTicks = 0;
   pwrDis = 0.0f;
   pwrChg = 0.0f;
   iDisMax = 0.0f;
   iChgMax = 0.0f;
   limSrc = SRC_NODATA;
}

/* ----------------------------------------------------------------- terminal */

void DumpTables(IPutChar* out)
{
   fprintf(out, "gen=%u page=%d slot=%d dirty=%d R=%d.%d mOhm src=%d\r\n",
           (unsigned)lastGen, lastPage, lastSlot, dirty ? 1 : 0,
           (int)rUsed, (int)((rUsed - (int)rUsed) * 10.0f), limSrc);

   for (int dir = 0; dir < 2; dir++)
   {
      const float* table = dir ? rChg : rDis;
      fprintf(out, "%s R_eff mOhm, columns C=0.5/1/2/3/4\r\n", dir ? "charge" : "discharge");
      for (int it = 0; it < NT; it++)
      {
         for (int is = 0; is < NS; is++)
         {
            /* This printf has no '+' flag - using one would swallow the format
             * and shift every later argument on the line. */
            fprintf(out, " T%4d SOC%3d:", (int)TAXIS[it], (int)SAXIS[is]);
            for (int ic = 0; ic < NC; ic++)
            {
               float v = table[BinIndex(it, is, ic)];
               fprintf(out, " %4d.%d", (int)v, (int)((v - (int)v) * 10.0f));
            }
            fprintf(out, "\r\n");
         }
      }
   }
}

void ResetTables()
{
   Prefill();
   dirty = true;
   persistWanted = true;
}

}

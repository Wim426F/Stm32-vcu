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
#include <math.h>
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
static const int NS = 4;
static const int NC = 5;
static const int NBINS = NT * NS * NC; // 100
static const float TAXIS[NT] = { -10.0f, 0.0f, 20.0f, 35.0f, 50.0f };
// R doubles from 10 % to 0 %; aged cells rise from 30 % down
static const float SAXIS[NS] = { 0.0f, 10.0f, 30.0f, 90.0f };
static const float CAXIS[NC] = { 0.5f, 1.0f, 2.0f, 3.0f, 4.0f };

/* ---- learning ---- */
static const float EWMA_ALPHA = 0.05f;   // heavy: pack IR moves over months
static const float R_MIN = 8.0f;         // mOhm, pack
static const float R_OUTLIER = 3.0f;     // reject > 3x or < 1/3 of the prefill bin
static const float T_NODE_BAND = 2.0f;   // K, learn only this close to a temperature node
static const float I_LEARN_MIN = 15.0f;  // need this much load to measure at all
static const float I_STABLE_BAND = 8.0f; // peak-to-peak allowed across the window
static const float DI_MIN = 25.0f;       // plateau-to-plateau step worth using
static const float DV_CELL_MIN = 0.005f; // below this it is all measurement noise
static const uint16_t STABLE_TICKS = 15; // 1.5 s
static const uint16_t STABLE_TICKS_HI = 5;// 0.5 s at 1C and above: high current comes in
                                         // short peaks, and R grows ~6 % from 0.2 to 4 s
static const uint16_t REF_MAX_AGE = 300; // 30 s - a reference older than this has
                                         // drifted in OCV and would inflate R
static const float REF_MAX_DSOC = 2.0f;  // ...as has one taken 2% of SOC ago

/* ---- output ---- */
static const float PWR_SLEW = 5.0f;      // kW per tick -> 50 kW/s
static const float PWR_NOLIMIT = 1000.0f;// "battery imposes nothing" - consumers clamp
static const float I_NOCAP = 1.0e6f;     // A, HeadroomCurrent without a current cap

/* Series cell count. Fixed for this pack; DilithiumMCU hardcodes the same 73. */
static const float CELLS = 73.0f;

/* ---- plating ----
 * Charge-mode current ceiling against SOC, in C. Tesla Model 3 LR on a 250 kW
 * Supercharger, same 21700 NMC cell family, hot pack (cells 45 to 59 C):
 * measured pack current / 221 Ah (Bjorn Nyland). Off either end it holds the end value.
 * Used as measured: it is the only source for current against SOC. At 80 to 90 % the cells
 * sat at 4.2 V, so those points are a voltage limit and on the low side of Tesla's map.
 * The LG M50LT datasheet (0.3C at 0 to 25 C, 0.7C at 25 to 45 C, complete charge) gives no
 * per-SOC limit; its temperature step implies Ea 25 kJ/mol, confirming the 26 below. */
static const int NP = 9;
static const float PSOC[NP] = { 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f };
static const float PCRATE[NP] = { 3.08f, 2.71f, 2.17f, 1.67f, 1.13f, 0.90f, 0.72f, 0.51f, 0.34f };

/* Temperature factor on the coldest cell, Arrhenius: f = exp(-Ea/R * (1/T - 1/Tref)), max 1.
 * Ea 26 kJ/mol fits plating-free 1C at 20 C and 2C at 40 C (Yang 2018, NMC/graphite).
 * Tref 50 C is the temperature the curve above was measured at.
 * f: 0 C 0.17, 10 C 0.25, 20 C 0.37, 30 C 0.53, 40 C 0.74, 50 C 1.0. */
static const float PLATING_EA_OVER_R = 26000.0f / 8.314f; // K
static const float PLATING_TREF = 323.15f;                // K

/* Regen reference: a TUNED value, not physics. Same Ea, but f reaches 1 at 25 C instead
 * of 50 C, which is the charge factor x 2.25. No data gives a safe limit for a regen pulse
 * of seconds; the 50 C reference comes from sustained charging and allowed only 20 kW at
 * 15 C / 80 % SOC. 25 C is chosen to match owner reports of Tesla regen (about half on a
 * cold pack, full once warm). See Power_estimater_idea.md, Plating limit. */
static const float PLATING_TREF_REGEN = 298.15f;          // K

enum LimSrc { SRC_NONE = 0, SRC_SAG, SRC_DRAG, SRC_HOT, SRC_COLD, SRC_NODATA, SRC_PLATING };

/* ---- flash journal: 2 pages x 4 slots, below the param/CAN pages ---- */
static const uint32_t PE_MAGIC = 0x52454646u; // 'REFF'
static const uint16_t PE_VERSION = 3;   // 3: T {-10,0,20,35,50}, SOC {0,10,30,90}
static const int PE_SLOTS = 4;
static const uint32_t PE_SLOT_SIZE = 416;

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
static_assert(PE_SLOTS * PE_SLOT_SIZE <= FLASH_PAGE_SIZE, "journal slots do not fit a page");

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
   float ah;
   int   mode;
   bool  haveCells;
};

/* ---- state ---- */
static float rDis[NBINS];
static float rChg[NBINS];
static float pwrDis;
static float pwrChg;
static float pwrRegen;
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

/* An earlier steady load, the voltage reference for the next one. It goes stale:
 * once SOC has moved, the difference is mostly OCV drift, not IR drop.
 * No resting reference: a loaded cell keeps building slow polarisation for tens
 * of seconds, so a step from rest reads 20 % to 70 % above the IR the prefill holds. */
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

static Ref plat;
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

// Starting point for pack R_eff (mOhm), worst-cell basis, discharge.
// R = 73 x dVmin / dI between two steady loads (plateau method, as Learn()), 422
// measurements from 11 drive logs: BMS temperature 15.5 C to 24.5 C, VCU SOC 0 % to 93 %,
// 0.2C to 2.6C. R = R20(SOC) x fT x (1 + 0.09 x (C - 0.5)), least squares on all points.
// R20 at 20 C / 0.5C, measured: 0 % 96.0, 10 % 52.4, 30 % 46.9, 90 % 49.9.
// fT: 20 C 1.00; 35 C 0.826, 50 C 0.694 from the measured slope (Ea 9.6 kJ/mol, 90 % range
//     0 to 18.5), extrapolated; 0 C 1.567, -10 C 2.507 not measured.
// C: +9 % per C; measured flat to 1.1C, +5 % at 1.6C, +10 % at 2.6C; 3C/4C extrapolated.
// Axes: T {-10,0,20,35,50}, SOC {0,10,30,90}, C {0.5,1,2,3,4}.
// Also the anchor for the learning outlier check.
static const float PREFILL[NT][NS][NC] =
{
   { { 240.7f, 251.5f, 273.2f, 294.8f, 316.5f },
     { 131.4f, 137.3f, 149.1f, 160.9f, 172.7f },
     { 117.6f, 122.9f, 133.5f, 144.0f, 154.6f },
     { 125.1f, 130.7f, 142.0f, 153.2f, 164.5f } },
   { { 150.4f, 157.2f, 170.7f, 184.3f, 197.8f },
     {  82.1f,  85.8f,  93.2f, 100.6f, 108.0f },
     {  73.5f,  76.8f,  83.4f,  90.0f,  96.6f },
     {  78.2f,  81.7f,  88.7f,  95.8f, 102.8f } },
   { {  96.0f, 100.3f, 109.0f, 117.6f, 126.2f },
     {  52.4f,  54.8f,  59.5f,  64.2f,  68.9f },
     {  46.9f,  49.0f,  53.2f,  57.5f,  61.7f },
     {  49.9f,  52.1f,  56.6f,  61.1f,  65.6f } },
   { {  79.3f,  82.9f,  90.0f,  97.1f, 104.3f },
     {  43.3f,  45.2f,  49.1f,  53.0f,  56.9f },
     {  38.7f,  40.5f,  44.0f,  47.5f,  50.9f },
     {  41.2f,  43.1f,  46.8f,  50.5f,  54.2f } },
   { {  66.6f,  69.6f,  75.6f,  81.6f,  87.6f },
     {  36.4f,  38.0f,  41.3f,  44.5f,  47.8f },
     {  32.5f,  34.0f,  36.9f,  39.9f,  42.8f },
     {  34.6f,  36.2f,  39.3f,  42.4f,  45.5f } }
};
// Charge (regen) measures 1.11 x discharge on the same logs (20 points, to 0.7C).
static const float CHG_OVER_DIS = 1.11f;

static void Prefill()
{
   for (int it = 0; it < NT; it++)
      for (int is = 0; is < NS; is++)
         for (int ic = 0; ic < NC; ic++)
         {
            int i = BinIndex(it, is, ic);
            rDis[i] = PREFILL[it][is][ic];
            rChg[i] = PREFILL[it][is][ic] * CHG_OVER_DIS;
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

   /* Erasing a page stalls the bus for tens of ms with IRQs off. Only in OFF,
    * contactors open; until then the flag stays set and the write waits. */
   if (Param::GetInt(Param::opmode) != MOD_OFF) return;

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

/* EWMA the raw measurement into the nearest bin, only when the pack temperature is
 * within T_NODE_BAND of a temperature node. The lookup interpolates between nodes, so a
 * value measured between them belongs to neither: a 12 C value in the 20 C node would
 * add the cold-side slope a second time at 12 C.
 * Reject a value outside 1/R_OUTLIER..R_OUTLIER x the prefill of the bin. Checked against
 * the fixed prefill, not the bin itself: against the bin, every accepted high value widens
 * the window for the next, and the bin can creep up without limit. */
static void Learned(float* table, float prefillScale, const Inputs& in, float crate, float rMeas)
{
   int it = Nearest(TAXIS, NT, in.t);
   if (ABS(in.t - TAXIS[it]) > T_NODE_BAND) return;

   int is = Nearest(SAXIS, NS, in.soc);
   int ic = Nearest(CAXIS, NC, crate);
   float pre = PREFILL[it][is][ic] * prefillScale;
   if (rMeas > pre * R_OUTLIER || rMeas < pre / R_OUTLIER) return;

   int i = BinIndex(it, is, ic);
   float next = table[i] * (1.0f - EWMA_ALPHA) + rMeas * EWMA_ALPHA;
   if (next != table[i])
   {
      table[i] = next;
      dirty = true;
   }
}

static void Learn(const Inputs& in)
{
   if (!in.haveCells) return;

   RefAge(&plat, in);

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

   uint16_t stableNeed = (in.ah > 1.0f && in.absI >= in.ah) ? STABLE_TICKS_HI : STABLE_TICKS;
   if (stableTicks < stableNeed || in.absI < I_LEARN_MIN) return;

   bool charging = in.idc > 0.0f;

   /* Plateau-to-plateau: dV/dI cancels OCV entirely.
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
         float rMeas = (ABS(dv) * CELLS * 1000.0f) / ABS(di);
         /* Filed under the higher of the two currents: with the lower one small,
          * dV/dI is R at the higher current, also on a step down after a peak. */
         float crate = (in.ah > 1.0f) ? (MAX(in.absI, plat.absI) / in.ah) : CAXIS[0];
         if (charging)
            Learned(rChg, CHG_OVER_DIS, in, crate, rMeas);
         else
            Learned(rDis, 1.0f, in, crate, rMeas);
      }
   }

   /* This steady stretch becomes the reference for the next one. */
   RefUpdate(&plat, in);
   stableTicks = 0;
}

/* ------------------------------------------------------------------ limiting */

/* How much current drags the worst cell from where it is now to its limit.
 *
 * R depends on C-rate, and the C-rate depends on the answer, so solve it:
 * guess from R at the present rate, recompute the rate, look R up again. Four
 * passes end on the low side of the answer - without this, a lookup at rest lands
 * on the 0.5C bin (the lowest R in the table) and predicts thousands of amps.
 * The answer never exceeds iCap, and R comes from the C-rate of the capped answer.
 */
static float HeadroomCurrent(const float* table, const Inputs& in,
                             float headCell, float iNow, float iCap, float* rOut)
{
   float crate = (in.ah > 1.0f) ? (in.absI / in.ah) : CAXIS[0];
   float iTotal = iNow;
   float r = R_MIN;

   for (int pass = 0; pass < 4; pass++)
   {
      r = Lookup(table, in.t, in.soc, crate);
      iTotal = MIN(iNow + (headCell * CELLS) / (r * 0.001f), iCap);
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
      pwrRegen = PWR_NOLIMIT;
      rUsed = 0.0f;
      iDisMax = 0.0f;
      iChgMax = 0.0f;
      limSrc = SRC_NODATA;
      return;
   }

   /* Signed present current per direction: a cell under discharge load sits
    * below its OCV, so the charge headroom it shows includes that discharge. */
   float iNowDis = -in.idc;
   float iNowChg = in.idc;
   float vFloorPack = in.vFloor * CELLS;
   int src = SRC_NONE;

   /* Discharge: power at the moment the worst cell reaches its floor, so the
    * voltage in P = I * V is the floor itself. Past the floor the headroom is
    * negative, so the current drops by the overshoot and holds the cell at the
    * floor instead of cutting to 0. */
   float rD;
   float headDis = in.vMinCell - in.vFloor;
   float dis = HeadroomCurrent(rDis, in, headDis, iNowDis, I_NOCAP, &rD) * vFloorPack / 1000.0f;
   if (headDis <= 0.0f) src = SRC_SAG;
   rUsed = rD;

   /* Charge and regen both stop at the ceiling (drag) and both scale with the
    * plating factor of the coldest cell, regen with the tuned PLATING_TREF_REGEN.
    * Charge mode is also capped by the SOC
    * curve: a sustained charge plates at high SOC, a regen pulse of seconds does not. */
   float tK = in.tMin + 273.15f;
   float fT = MIN(expf(-PLATING_EA_OVER_R * (1.0f / tK - 1.0f / PLATING_TREF)), 1.0f);
   float fRegen = MIN(expf(-PLATING_EA_OVER_R * (1.0f / tK - 1.0f / PLATING_TREF_REGEN)), 1.0f);

   int p0, p1;
   float fp;
   Bracket(PSOC, NP, in.soc, &p0, &p1, &fp);
   float iCurve = (PCRATE[p0] + (PCRATE[p1] - PCRATE[p0]) * fp) * in.ah * fT;

   float headChg = in.vCeil - in.vMaxCell;
   float rC;
   float iChg = HeadroomCurrent(rChg, in, headChg, iNowChg, iCurve, &rC);
   float rR;
   float iRegen = HeadroomCurrent(rChg, in, headChg, iNowChg, I_NOCAP, &rR) * fRegen;
   rR = Lookup(rChg, in.t, in.soc, ABS(iRegen) / MAX(in.ah, 1.0f));
   if (headChg <= 0.0f && src == SRC_NONE) src = SRC_DRAG;

   /* Power at the pack voltage this current produces, so a charger that divides
    * by its measured pack voltage gets the same current back. Built on the lowest
    * cell, so it reads a little low and the current comes out a little low. */
   float vChgPack = MAX(in.vMinCell * CELLS + (iChg - in.idc) * rC * 0.001f, 1.0f);
   float vRegenPack = MAX(in.vMinCell * CELLS + (iRegen - in.idc) * rR * 0.001f, 1.0f);
   float chg = iChg * vChgPack / 1000.0f;
   float regen = iRegen * vRegenPack / 1000.0f;

   /* Hot ramp on tMax, BMS_TderateSpan long, ending at each direction's maximum:
    * BMS_TmaxLimit for discharge, BMS_TmaxChg for charge and regen (the datasheet
    * allows a lower cell temperature while charging). Tapers to BMS_PwrHot, not to zero.
    * 15 kW holds highway speed, or adds 15 kWh/h with charging.
    * Sag/drag still pull below the floor - TempDerate only reduces. */
   float span = Param::GetFloat(Param::BMS_TderateSpan);
   float pwrHot = Param::GetFloat(Param::BMS_PwrHot);
   float tMaxDis = Param::GetFloat(Param::BMS_TmaxLimit);
   float tMaxChg = Param::GetFloat(Param::BMS_TmaxChg);

   if (in.tMax > tMaxDis - span)
   {
      dis = TempDerate(dis, in.tMax, tMaxDis - span, tMaxDis, pwrHot);
      if (src == SRC_NONE) src = SRC_HOT;
   }
   if (in.tMax > tMaxChg - span)
   {
      chg = TempDerate(chg, in.tMax, tMaxChg - span, tMaxChg, pwrHot);
      regen = TempDerate(regen, in.tMax, tMaxChg - span, tMaxChg, pwrHot);
      if (src == SRC_NONE) src = SRC_HOT;
   }

   /* Cold, charge and regen only, the same span below BMS_TminLimit. Discharge must
    * stay available so the car can be driven. Ramped, not stepped, so regen does not
    * vanish in a single tick mid-drive. */
   float tColdLimit = Param::GetFloat(Param::BMS_TminLimit);
   if (in.tMin < tColdLimit)
   {
      float k = (in.tMin <= tColdLimit - span)
                ? 0.0f
                : Lerp(in.tMin, tColdLimit, tColdLimit - span, 1.0f, 0.0f);
      chg *= k;
      regen *= k;
      if (src == SRC_NONE) src = SRC_COLD;
   }

   /* Last in line: below 25 C the regen plating factor is always below 1, so in drive
    * it would otherwise hide every other source. */
   bool plating = (in.mode == MOD_CHARGE) ? (iChg >= iCurve) : (fRegen < 1.0f);
   if (plating && src == SRC_NONE) src = SRC_PLATING;

   dis = MAX(dis, 0.0f);
   chg = MAX(chg, 0.0f);
   regen = MAX(regen, 0.0f);

   /* Slew the limit upwards only. The worst cell is noisy and the DU sees this
    * every 500 ms, so ramping up stops us chasing that noise - but a reduction
    * is the safe direction and has to land immediately. It also means recovering
    * from SRC_NODATA drops straight to the real limit instead of leaving the
    * pack unprotected while a 1000 kW placeholder ramps down. */
   pwrDis = (dis < pwrDis) ? dis : MIN(dis, pwrDis + PWR_SLEW);
   pwrChg = (chg < pwrChg) ? chg : MIN(chg, pwrChg + PWR_SLEW);
   pwrRegen = (regen < pwrRegen) ? regen : MIN(regen, pwrRegen + PWR_SLEW);

   /* The same limits as current, for display. Divided by the voltage the power
    * figure was built on. */
   iDisMax = pwrDis * 1000.0f / vFloorPack;
   iChgMax = pwrChg * 1000.0f / vChgPack;
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

   in.ah = Param::GetFloat(Param::BMS_CapActual);
   in.mode = Param::GetInt(Param::opmode);

   Learn(in);
   Compute(in);

   /* In charge mode, a hold or a lost BMS means 0 W. NODATA's "no limit" is
    * for driving only. Every charger obeys BMS_MaxInput, so this gates them all. */
   int mode = in.mode;
   if (mode == MOD_CHARGE && (Param::GetInt(Param::ChgHold) != 0 || !in.haveCells))
   {
      pwrChg = 0.0f;
      iChgMax = 0.0f;
   }

   /* Highest cell's resting voltage: loaded voltage minus the IR rise of the
    * charge current. Unsubtracted polarisation only makes it read high, so a
    * charge that ends on it ends early, never late. */
   float ocvMax = in.vMaxCell;
   if (in.idc > 0.0f && in.ah > 1.0f)
      ocvMax -= in.idc * Lookup(rChg, in.t, in.soc, in.absI / in.ah) * 0.001f / CELLS;
   Param::SetFloat(Param::BMS_OcvMax, in.haveCells ? ocvMax * 1000.0f : 0.0f);

   /* Pack backstop for the chargers' own voltage target: only reached when
    * every cell sits at the hard stop. */
   Param::SetFloat(Param::Voltspnt, MAX(Param::GetFloat(Param::BMS_VmaxLimit), in.vCeil) * CELLS);

   Param::SetFloat(Param::BMS_MaxOutput, pwrDis);
   Param::SetFloat(Param::BMS_MaxInput, pwrChg);
   Param::SetFloat(Param::BMS_MaxRegen, pwrRegen);
   Param::SetFloat(Param::BMS_IdisMax, iDisMax);
   Param::SetFloat(Param::BMS_IchgMax, iChgMax);
   Param::SetFloat(Param::BMS_Rpack, rUsed);
   Param::SetInt(Param::BMS_LimSrc, limSrc);

   /* Save on the way out of a live mode. Never on a timer while driving: the
    * page erase in CommitFlash runs with interrupts off and stalls the bus for
    * tens of ms, and the table only moves over weeks anyway. */
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
   plat.valid = false;
   stableTicks = 0;
   pwrDis = 0.0f;
   pwrChg = 0.0f;
   pwrRegen = 0.0f;
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

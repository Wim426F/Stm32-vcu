# Battery power limiter

Implementation notes for `src/power_estimator.cpp` / `include/power_estimator.h`.

## Purpose

Publish how much power the pack can safely deliver and absorb, derived from the worst cell's
remaining voltage headroom divided by the pack's internal resistance at the present temperature,
SOC and C-rate. Resistance is learned in the background and persisted to flash.

## Authority model

The module reports battery capability only. The user's power preferences are applied by the consumers:

```
final limit = MIN(user preference, battery capability, what the other end can supply)
```

The estimator can only reduce a limit, never raise one. It does not read the preferences.

## Interface

Written by `PowerLimiter` only:

| Value | Unit | Meaning |
|---|---|---|
| `BMS_MaxOutput` | kW | discharge capability |
| `BMS_MaxInput` | kW | charge capability |
| `BMS_IdisMax` | A | same discharge limit as current, informational |
| `BMS_IchgMax` | A | same charge limit as current, informational |
| `BMS_Rpack` | mΩ | pack R_eff at the present discharge operating point |
| `BMS_LimSrc` | enum | binding constraint: `None/Sag/Drag/Hot/Cold/NoData` |

`BMS_MaxOutput` / `BMS_MaxInput` are the values a BMS is meant to report. The estimator is the missing
part of the BMS rather than a separate subsystem, so it uses the `BMS_` namespace throughout, and it is
now the only writer — the equivalent writes were dropped from `leafbms` and `kangoobms`, where task
ordering silently decided the winner.

Battery parameters:

| Param | Unit | Default | Meaning |
|---|---|---|---|
| `BMS_VsagLimit` | V | 2.5 | worst cell may be dragged here under load |
| `BMS_VdragLimit` | V | 4.2 | worst cell may be pushed here under charge |
| `BMS_Tderate` | °C | 40 | hot ramp start |
| `BMS_PwrHot` | kW | 15 | power held at and above `BMS_TmaxLimit` |

`BMS_VsagLimit` / `BMS_VdragLimit` are deliberately separate from `BMS_VminLimit` / `BMS_VmaxLimit`.
The latter are resting fault thresholds used by `DilithiumMCU::ChargeAllowed()`; sagging to 2.5 V under
500 A is normal, resting at 2.5 V is not.

User power preferences, all kW. Set in power because that is what the T2C accepts and what a CCS
session negotiates. The equivalent current is a consequence of pack sag, so it is published as a value
and never set:

| Param | Unit | Default | Applied by |
|---|---|---|---|
| `PwrMotMax` | kW | 150 | `EvControlsT2C`, and `Throttle::IdcLimitCommand` for any other inverter |
| `PwrRegenMax` | kW | 60 | same |
| `PwrCcsMax` | kW | 50 | `i3LIM`, `chademo`, `Foccci` (filed under Charger Control) |
| `PwrAcMax` | W | — | `teslaCharger` (pre-existing, left in watts) |

Series cell count is the compile-time constant `CELLS` in `power_estimator.cpp`, not a parameter — it
never changes for this pack, and `DilithiumMCU` hardcodes the same 73.

Reused: `BMS_TmaxLimit` (hot ramp end), `BMS_TminLimit` (cold ramp start).

Inputs read: `BMS_Vmin`, `BMS_Vmax`, `BMS_Tmin`, `BMS_Tmax`, `BMS_Tavg`, `SOC`, `idc`, `udc2`
(falling back to `udc`), `BattCap`, `opmode`.

## Task rate

One `Task100Ms()`. Every input is written once per 100 ms by `DilithiumMCU::Task100Ms`, and 0x293 is
multiplexed on `data[0]` so the cell-voltage sub-frame can arrive slower still. A faster task would
reprocess identical values.

`PollPersist()` is called from the main loop, never from the scheduler ISR — see Persistence.

## Tables

Two tables, one per direction, holding pack R_eff in mΩ:

- temperature: `-10, 0, 25, 40, 50` °C
- SOC: `10, 50, 90` %
- C-rate: `0.5, 1, 2, 3, 4`

5 × 3 × 5 = 75 bins per table. Lookup is trilinear, clamped to the end bin off either axis end.
Initialised from a conservative physics-based prefill, never from empty.

## Learning

A measurement requires a stable load and a valid voltage reference.

**Stability** — peak-to-peak of raw `idc` must stay within `I_STABLE_BAND` (8 A) across the whole
`STABLE_TICKS` (15 tick = 1.5 s) window. Peak-to-peak rather than deviation from a filtered value:
a filter with a short time constant tracks a slow ramp, so the residual stays small and a ramp
passes as stable. With τ ≈ 62 ms an 8 A band admitted ramps up to ~140 A/s.

**References** — two kinds, both aged out after `REF_MAX_AGE` (60 s) or a `REF_MAX_DSOC` (2 %) shift
in SOC. Beyond that the difference is mostly OCV drift, not IR drop, and R comes out inflated.

- *plateau*: an earlier steady load. Preferred, because ΔV/ΔI cancels OCV entirely. Requires
  `|ΔI| ≥ DI_MIN` (25 A), same direction, and ΔV and ΔI agreeing in sign — more current must mean
  more voltage drop. A disagreeing pair is discarded rather than absolute-valued.
- *rest*: an open-circuit sample taken after `REST_TICKS` (2 s) below `I_REST` (8 A). Fallback only.

**Measurement** — R_eff at the pack, from the worst cell:

```
R_pack[mΩ] = ΔV_cell × CELLS × 1000 / ΔI
```

using `minCellV` on discharge and `maxCellV` on charge.

**Update** — EWMA with α = 0.05 into the single nearest T/SOC/C-rate bin. No extrapolation to
neighbours. Rejected if outside 8…2000 mΩ, or more than a factor of 3 from the value the bin already
holds; a valid measurement does not move pack IR threefold. Note the factor-of-3 gate also bounds how
far learning can pull away from the prefill — if the prefill is badly wrong, `pereset` and a corrected
prefill is the route, not waiting for convergence.

## Limit computation

Headroom current, per direction:

```
headroom_cell = vCell_now − vCell_limit
I_total       = I_now + headroom_cell × CELLS / R_pack
P             = I_total × vCell_limit × CELLS
```

`P` uses the limit voltage, not the present voltage, because it is the power available at the moment
the worst cell arrives at its limit.

`R_pack` depends on C-rate and C-rate depends on `I_total`, so it is solved by fixed point: look R up
at the present rate, recompute the rate from the result, look up again. Three passes. Without this, a
lookup at standstill lands on the 0.5C bin — the lowest R in the table — and over-predicts current by
roughly 1.7×.

The iteration converges to an interior R only when the predicted current is inside the 4C axis end.
Above that it pegs at 4C and the figure stays optimistic. For a 216 Ah pack this is the normal case
at moderate temperature, where the preference binds regardless; the iteration converges properly in
the cold and low-SOC region where the estimator actually becomes the binding limit.

## Thermal ramps

Applied after the headroom calculation, battery temperature only. Inverter and motor temperature are
not battery protection and are not consulted.

**Hot**, on `BMS_Tmax`, both directions: full power below `BMS_Tderate`, linear down to `BMS_PwrHot`
at `BMS_TmaxLimit`, held at `BMS_PwrHot` above. Reduction only — an already-lower limit is never
raised toward the floor.

A hard cut to 0 kW is itself unsafe: the pack is commonly at 40–50 °C after a fast charge, and the
vehicle must still be able to hold motorway speed. Loop capacity is unknown, so the defaults are
sized by I²R heat generation at the pack instead (303 V, R from the table):

| condition | current | R | heat |
|---|---|---|---|
| 152 kW at 40 °C | 502 A | 22.2 mΩ | 5.6 kW |
| 84 kW at 45 °C | 277 A | 17.0 mΩ | 1.3 kW |
| 15 kW at 50 °C | 50 A | 12.5 mΩ | 31 W |

5.6 kW of self-heating exceeds any plausible loop, so derating must already be under way at 40 °C.
31 W is two orders of magnitude below it, so the pack cools even while sitting at the floor. Loop
capacity therefore never needs measuring: even a pessimistic 250 W rejection budget would sustain
43 kW. `BMS_PwrHot` = 15 kW is chosen to meet the requirement (≈10–15 kW holds motorway speed for
this car), not as the maximum the pack could sustain — raise it if logged cell temperatures show
margin.

**Cold**, on `BMS_Tmin`, charge only: linear from full at `BMS_TminLimit` to 0 at
`BMS_TminLimit − 5 °C`. Discharge is not derated by cold; the vehicle must remain drivable. Ramped
rather than stepped so regen does not disappear in a single tick while driving.

## Degraded input

If `BMS_Vmin`/`BMS_Vmax` are absent (BMS timeout, or a BMS that does not report cells) both outputs
are set to `PWR_NOLIMIT` (1000 kW) and `BMS_LimSrc = NoData`. Publishing 0 would cut drive power to
nothing on a momentary CAN dropout, and the T2C faults on a 0 kW limit. The preferences remain in
force at the consumers.

## Output slew

Increases are limited to `PWR_SLEW` (5 kW/tick = 50 kW/s); decreases take effect immediately. Ramping
up avoids chasing worst-cell noise; a reduction is the safe direction and must not be delayed. This
also means recovery from `NoData` drops straight to the real limit rather than leaving the pack
unprotected while a 1000 kW placeholder ramps down.

## Persistence

Two flash pages (`PE_BLKNUM 5`, `PE_PAGES 2` → `0x0801D800` and `0x0801D000`), six 320-byte slots
each, below the param and CAN map pages. ROM is 112 K and ends exactly at `0x0801D000`.

Each slot holds magic, generation counter, version, both tables as `uint16` at 0.1 mΩ, and a CRC.
Load scans all slots and takes the highest valid generation. Commit appends to the next free slot;
when a page fills, it moves to the other page and erases that one, so the newest record on the
opposite page always survives a power loss mid-commit. One erase per six saves.

Committed only on the live → not-live `opmode` transition, never on a timer while driving:
`flash_erase_page` runs with interrupts disabled and stalls the bus for 20–40 ms, masking the TIM4
scheduler and all CAN RX. R_eff moves over weeks, so a periodic save buys nothing.

`dirty` is set from the scheduler ISR and cleared in the main loop before the write, so anything
learned during the erase still marks the table dirty.

`PollPersist()` additionally refuses to erase while `opmode` is RUN or CHARGE and keeps the request
pending. The normal trigger already fires after the mode has left live, so this only affects
`pereset`, which defers its save to the next shutdown rather than stalling the bus mid-drive.

## Consumers

| Path | File | Preference | Expression |
|---|---|---|---|
| Motor drive | `EvControlsT2C.cpp` | `PwrMotMax` | `MIN(PwrMotMax, BMS_MaxOutput)` |
| Motor regen | `EvControlsT2C.cpp` | `PwrRegenMax` | `MIN(PwrRegenMax, BMS_MaxInput)` |
| Motor, other inverters | `throttle.cpp` | `PwrMotMax` / `PwrRegenMax` | `IdcLimitCommand` derives its amp limits at the present `udc` |
| AC charge | `teslaCharger.cpp` | `PwrAcMax` | `BMS_MaxInput × 1000` into the existing MIN chain |
| DC charge | `i3LIM.cpp` | `PwrCcsMax` | both converted to amps at `udc`, then MIN |

T2C clamps the 0x696 fields to 1…650 kW. The frame carries `uint16` at 0.01 kW, so 655.35 kW is the
largest representable value and an unclamped cast wraps; the lower bound keeps the limit off exactly
zero, which the drive unit treats as a fault.

`regenmax` is **not** usable as a regen power cap. Despite its `"A"` unit label it is a torque
percent — `throttle.cpp` feeds it to `changeFloat(potnom, 0, 100, regenlim*10, throtmax*10)`.

### DC fast charge ceilings

`PwrCcsMax` is a global preference and is not capped by any one charge interface — 350 kW is allowed
because a Foccci-based setup can carry it. Each interface clamps internally to what its own protocol
can express.

For i3LIM that is two ceilings, now the only things limiting the DC path:

- `FC_Cur` in 0x3E9 is a 10-bit field (bits 0-7 in byte 5, bits 8-9 in byte 6) → **511 A**, about
  155 kW at 303 V.
- `CHG_Pwr` is a 12-bit field at 25 W scale → **102.375 kW**.

The arbitrary limits were removed: `CCSI_Spnt` widened from `uint8_t` to `uint16_t`, the hardcoded
`>150 A` clamp dropped, and the `CHG_Pwr` forecast driven from `MIN(PwrCcsMax, BMS_MaxInput)` rather
than a constant 44 kW. `FC_Cur`'s high bits were packed with `>>12` instead of `>>8`, which was correct
only while the setpoint could not exceed 255 — widening the type without fixing that would have
transmitted 330 A as 74 A. The ramp is ±1 A per 100 ms, so a 330 A request takes about 33 s to reach.

One real defect was fixed alongside: the decrements in `CCS_Pwr_Con()` had no zero floor, so an
end-of-charge taper wrapped `0 → 255` on an unsigned counter and then walked back down from the clamp
instead of settling at zero. The `>250 → 0` guard in the original was written for that case but sat
below the `>150` clamp, which rewrote 255 before it could fire.

## Terminal

- `pedump` — journal state plus both tables, by temperature and SOC row.
- `pereset` — restore the prefill in RAM and queue a save for the next shutdown.

## Operating envelope

Discharge limit in kW at SOC-consistent OCV, prefill tables, against a `PwrMotMax` of 150 kW. Values
below that are where the estimator overrides the preference:

```
        SOC:    90%    70%    50%    30%    20%    10%     5%     2%
   -10 C        246    225    202    167    152   *134   *121   *106
     0 C        356    330    300    244    218    184    161   *144
    25 C        574    514    451    381    345    296    259    222
    40 C        688    610    528    452    412    355    311    266
```

With one cell diverging below the pack average, at 25 °C:

```
  offset:  90%    70%    50%    30%    20%    10%     5%     2%
  -0.20 V  500    438    372    305    270    222    185    152
  -0.40 V  426    362    294    228    195    152   *128    *94
  -0.60 V  352    285    216    155   *134    *94    *53      0
```

Sag protection therefore engages in three situations: cold below ~20 % SOC, near-empty, or a
diverging cell. With a healthy pack at moderate temperature `PwrMotMax` is the binding limit and
`BMS_MaxOutput` reads high without reaching the CAN bus. Bench testing should target the cold and
low-SOC cases; a normal drive will not exercise it.

Raising `BMS_VsagLimit` toward `BMS_VminLimit` (3.0 V) moves engagement up to roughly 10 % SOC at
room temperature.

## Known limitations

- R pegs at the 4C axis end whenever the predicted current exceeds 4C; the reported figure is
  optimistic there. Not reached in practice because the preference binds first in that region.
- The prefill has identical R for 10 % and 90 % SOC, so that axis carries no information until
  learned.
- `BMS_Rpack` reports the discharge operating point only.
- `BMS_Vmin`/`BMS_Vmax` are labelled `"mV"` but `simpbms` publishes volts, so the reader sniffs on
  `raw > 10`.
- `BMS_VdragLimit` 4.2 V stores as 4.1875 V — `FP_FROMFLT` truncates at 1/32. Conservative.
- Learning is bounded to a factor of 3 around the prefill, per bin.
- `PwrAcMax` (AC charge) is still in watts while every other preference is in kW.
- `IdcTerm` and `CCS_ICmd` were removed as part of this work; both had only commented-out references.
- `Foccci.cpp` was not given the same treatment. It already uses a `uint16_t` setpoint and has its
  `>150 A` clamp commented out, but it still carries the unfloored decrements and a live
  `>250 → 0` guard, which there is reachable and caps it at 250 A. It also still reads
  `BMS_ChargeLim` rather than `BMS_MaxInput`.

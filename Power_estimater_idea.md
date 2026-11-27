# Battery power limiter

This page describes the battery power limiter in `src/powerLimiter.cpp` and `include/powerLimiter.h`.

## Principle

- Limit current = present current + headroom of the worst cell × 73 / pack R.
- Pack R comes from two learned tables, one per direction, kept in flash.
- Charge and regen also have a plating limit on the coldest cell.
- User temperature ramps reduce the result.
- The limiter only reduces. Each consumer takes MIN(user preference, limiter value).

## Data flow

```
DilithiumMCU -> BMS_Vmin, BMS_Vmax, BMS_Tmin, BMS_Tmax, BMS_Tavg, idc, SOC
             -> Learn() -> R tables -> PollPersist() -> flash record
             -> Compute() -> headroom current -> plating limit -> thermal ramps -> slew
             -> BMS_MaxOutput, BMS_MaxInput, BMS_MaxRegen -> consumers
```

## Functions

| Function | Rate | Action |
|---|---|---|
| `Init()` | once | loads the prefill, then the newest valid flash record |
| `Task100Ms()` | 100 ms | reads the inputs, runs `Learn()` and `Compute()`, publishes the values |
| `Learn()` | 100 ms | measures pack R between two steady loads and updates one bin |
| `Compute()` | 100 ms | calculates the three power limits |
| `PollPersist()` | main loop | writes a record in `MOD_OFF` |

## Tables

Two tables hold pack R in mΩ: `rDis` for discharge, `rChg` for charge and regen. `Lookup()`
interpolates on all three axes and holds the end node off an axis end.

| Axis | Nodes |
|---|---|
| Temperature (`BMS_Tavg`) | -10, 0, 20, 35, 50 °C |
| SOC | 0, 10, 30, 90 % |
| C-rate | 0.5, 1, 2, 3, 4 |

R doubles from 10 % to 0 % SOC. The 30 % node follows ageing: in an aged LG M50T cell, R rises
below 30 % SOC and is flat above it (Natella, Onori, Vasca, IEEE TIE 2023).

**Prefill** (`PREFILL`), a least-squares fit on 422 plateau measurements in 11 drive logs at 15.5 to
24.5 °C and 0 % to 93 % SOC:

```
R = R20(SOC) × fT × (1 + 0.09 × (C − 0.5))
```

| Part | Value | Basis |
|---|---|---|
| R20, 20 °C, 0.5C, SOC 0 / 10 / 30 / 90 % | 96.0 / 52.4 / 46.9 / 49.9 mΩ | measured |
| fT, 35 / 50 °C | 0.826 / 0.694 | measured slope, Ea 9.6 kJ/mol (90 % range 0 to 18.5) |
| fT, 0 / -10 °C | 1.567 / 2.507 | not measured |
| C-rate | +9 % per C | measured: flat to 1.1C, +10 % at 2.6C |
| Charge table | 1.11 × discharge (`CHG_OVER_DIS`) | measured on regen |

A valid flash record overrides the prefill. `pereset` loads the prefill.

## Learning

`Learn()` compares two steady loads. ΔV / ΔI cancels the OCV.

```
R_pack[mΩ] = ΔV_cell × 73 × 1000 / ΔI
```

ΔV_cell comes from `BMS_Vmin` on discharge and `BMS_Vmax` on charge.

| Condition | Value |
|---|---|
| Steady load | `idc` within 8 A peak-to-peak, 15 A or more |
| Window | 1.5 s below 1C, 0.5 s at 1C and above |
| Reference | the previous steady load, maximum 30 s and 2 % SOC old |
| Step | ΔI 25 A or more, ΔV 5 mV or more, same direction, same sign |

`Learned()` applies an EWMA with gain 0.05 to the raw value in one bin:

| Axis | Bin | Condition |
|---|---|---|
| Temperature | nearest node | within 2 K of the node (`T_NODE_BAND`) |
| SOC | nearest node | - |
| C-rate | node of the higher of the two currents | - |

The value must be within 1/3 to 3 × the prefill of the bin. Values between two temperature nodes
belong to neither node, so the code skips them.

Closed-loop replay of `drive_home_work3` (worst cell 4 % to 0 % SOC, 5 s drive unit smoothing,
`BMS_VsagLimit` 2.6 V): the lowest cell reaches 2.49 V, 2 s below 2.5 V in total.

## Limit computation

`HeadroomCurrent()`:

```
I = MIN(I_now + (vCell_now − vCell_limit) × 73 / R_pack, I_cap)
```

`I_now` is `−idc` for discharge and `+idc` for charge.

| Output | Limit | Cap | Power |
|---|---|---|---|
| `BMS_MaxOutput` | `BMS_VsagLimit` | - | I × `BMS_VsagLimit` × 73 |
| `BMS_MaxInput` | `BMS_VdragLimit` | SOC curve × `BMS_CapActual` × f | I × V_pack(I) |
| `BMS_MaxRegen` | `BMS_VdragLimit` | -, result × f_regen | I × V_pack(I) |

`V_pack(I) = BMS_Vmin × 73 + (I − idc) × R_pack`. A charger that divides by its measured pack voltage
gets the same current back.

Past the limit, the headroom is negative and the current drops below `I_now`. The worst cell stays
at its limit. R depends on the C-rate of the result, so the function makes 4 passes. Above 4C the
lookup holds the 4C node.

## Plating limit

| Output | Rule |
|---|---|
| `BMS_MaxInput` (AC and CCS) | MIN(drag current, SOC curve × `BMS_CapActual` × f) |
| `BMS_MaxRegen` | drag current × f_regen |
| `BMS_MaxOutput` | none |

**SOC curve** (`PSOC`, `PCRATE`): Tesla Model 3 LR on a 250 kW Supercharger, cells 45 to 59 °C
(Bjørn Nyland), pack current / 221 Ah. It holds the end value off either end. Regen pulses last
seconds and have no SOC curve.

| SOC % | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 |
|---|---|---|---|---|---|---|---|---|---|
| C | 3.08 | 2.71 | 2.17 | 1.67 | 1.13 | 0.90 | 0.72 | 0.51 | 0.34 |

**Temperature factor** on `BMS_Tmin`:

```
f = MIN(1, exp(−Ea / R × (1 / T − 1 / Tref)))
```

| Constant | Value | Source |
|---|---|---|
| Ea | 26 kJ/mol | Yang 2018, NMC/graphite; the M50LT datasheet step from 0.3C to 0.7C gives 25 |
| `PLATING_TREF` | 50 °C | the temperature of the SOC curve |
| `PLATING_TREF_REGEN` | 25 °C | tuned, see below |

| T °C | 0 | 5 | 10 | 15 | 20 | 25 | 40 | 50 |
|---|---|---|---|---|---|---|---|---|
| f | 0.17 | 0.21 | 0.25 | 0.31 | 0.37 | 0.44 | 0.74 | 1.00 |
| f_regen | 0.38 | 0.47 | 0.57 | 0.69 | 0.84 | 1.00 | 1.00 | 1.00 |

**f_regen is a tuned value, not physics.** It is f × 2.25, maximum 1. No source gives a safe regen
pulse limit against temperature. The 25 °C rests on:

- Plating grows with time at current: double the pulse length gave 1.5 to 3.3 × the plating in
  21700 cells (J. Electrochem. Soc., sub-zero pulse study).
- Tesla owner reports: half regen on a cold pack, full after 20 to 30 min of driving.
- Pack R comes from the table at the present temperature, so the drag limit tightens in the cold.

The one measured Tesla regen limit, 0.78C at 91 % SOC with the pack temperature unknown, gives 1.00C
with f_regen at 40 °C. At 0 °C and 30 % SOC f_regen allows 0.82C pulses.

`BMS_LimSrc` shows Plating when no other source applies: in charge mode when the curve caps the
current, in other modes when f_regen < 1.

## Thermal ramps

User preferences on battery temperature. They only reduce the limit.

**Hot ramp**, on `BMS_Tmax`: linear from full power at the maximum − `BMS_TderateSpan` to `BMS_PwrHot`
at the maximum, held above.

| Output | Maximum | Default |
|---|---|---|
| `BMS_MaxOutput` | `BMS_TmaxLimit` | 50 °C |
| `BMS_MaxInput`, `BMS_MaxRegen` | `BMS_TmaxChg` | 45 °C |

`BMS_PwrHot` 15 kW holds motorway speed. At 50 °C it puts 81 W into the pack (49.5 A, 33.2 mΩ). The
pack loses 60 to 225 W passively at 60 °C in 30 °C still air.

**Cold ramp**, on `BMS_Tmin`: linear from full at `BMS_TminLimit` to 0 at `BMS_TminLimit −
BMS_TderateSpan`. The charge gate stops charging at `BMS_TminLimit`, so the ramp acts on regen only.
Discharge has no cold ramp.

## Degraded input

With `BMS_Vmin` or `BMS_Vmax` at 0.5 V or below, all three outputs are `PWR_NOLIMIT` (1000 kW) and
`BMS_LimSrc` shows NoData. Missing cell data never gives a 0 kW discharge limit. In `MOD_CHARGE`,
`BMS_MaxInput` is 0.

## Empty pack

Below `BMS_VminLimit` (SOC 0 %), only the sag limit sets the discharge power. `BMS_MaxOutput` is 0
when the IR-compensated lowest cell is at `BMS_VsagLimit`.

| Condition | Result |
|---|---|
| `BMS_MaxOutput` 0 | The T2C sends 0 kW. The drive unit faults and coasts. A key cycle clears the fault. |
| `BMS_MaxOutput` 0 and `speed` below 50 rpm | `MOD_RUN` goes to `MOD_SHUTDOWN_REQUEST`, in any gear. |

`MOD_RUN` also goes to `MOD_SHUTDOWN_REQUEST` at key off with `speed` below 50 rpm, in any gear. The
lever goes to Park on `MOD_OFF`.

## Charge gate

In `MOD_CHARGE`, `Task100Ms()` sets `BMS_MaxInput` and `BMS_IchgMax` to 0 when `ChgHold` is not
None or there is no cell data. Every charger obeys `BMS_MaxInput`.

`BMS_OcvMax = BMS_Vmax − idc × R / 73`, with R from the charge table, while charging. It reads high,
so a charge that ends on it ends early.

The decision block in `stm32_vcu.cpp` (`Ms200Task`), `MOD_CHARGE`:

| Condition | AC | DC |
|---|---|---|
| `BMS_Tmin` below `BMS_TminLimit` or `BMS_Tmax` above `BMS_TmaxChg` | `ChgHold` = WaitTemp, resumes | same |
| `SOC ≥ ChgSocLimit` or `BMS_OcvMax ≥ BMS_VmaxLimit`, 4 s | `ChgHold` = Full, HV stays live | end and lockout |
| BMS charge current 0 for another reason, 4 s | end and lockout | same |
| `udc ≥ Voltspnt`, or the charge stop button | end and lockout | same |

`DilithiumMCU::ChargeAllowed()` refuses charge above `MAX(BMS_VmaxLimit, BMS_VdragLimit)`, below
2.00 V (`V_CELL_UVP`, LG INR21700-M50LT spec 2.11, under-voltage protection), above `BMS_TmaxChg`
and below `BMS_TminLimit`.

## Output slew

An increase is limited to 5 kW per tick (`PWR_SLEW`, 50 kW/s). A decrease applies at once.

## Published values

| Value | ID | Unit | Meaning |
|---|---|---|---|
| `BMS_MaxOutput` | 2106 | kW | discharge capability |
| `BMS_MaxInput` | 2105 | kW | charge capability, AC and CCS |
| `BMS_MaxRegen` | 2134 | kW | regen capability |
| `BMS_IdisMax` | 2119 | A | `BMS_MaxOutput` as current |
| `BMS_IchgMax` | 2118 | A | `BMS_MaxInput` as current |
| `BMS_Rpack` | 2120 | mΩ | pack R at the discharge operating point |
| `BMS_LimSrc` | 2121 | enum | 0 None, 1 Sag, 2 Drag, 3 Hot, 4 Cold, 5 NoData, 6 Plating |
| `BMS_OcvMax` | 2133 | mV | resting voltage estimate of the highest cell |
| `Voltspnt` | 2132 | V | `73 × MAX(BMS_VmaxLimit, BMS_VdragLimit)` |

## Parameters

| Entry | Unit | ID | Default | Role |
|---|---|---|---|---|
| `BMS_VsagLimit` | V | 154 | 2.6 | worst cell minimum under load |
| `BMS_VminLimit` | V | 92 | 3.0 | resting empty, SOC 0 % |
| `BMS_VmaxLimit` | V | 93 | 4.2 | resting full, SOC 100 %, charge target |
| `BMS_VdragLimit` | V | 155 | 4.2 | worst cell maximum under charge current |
| `BMS_TminLimit` | °C | 94 | 0 | charge gate, cold ramp start |
| `BMS_TmaxLimit` | °C | 95 | 50 | discharge hot ramp end |
| `BMS_TmaxChg` | °C | 163 | 45 | charge and regen hot ramp end, charge gate |
| `BMS_TderateSpan` | K | 162 | 5 | length of the hot and cold ramps |
| `BMS_PwrHot` | kW | 157 | 15 | power at and above each maximum temperature |
| `BMS_CapActual` | Ah | 160 | 215 | capacity for the C-rate |
| `PwrMotMax` | kW | 151 | 150 | user preference, drive |
| `PwrRegenMax` | kW | 152 | 35 | user preference, regen |
| `PwrCcsMax` | kW | 153 | 50 | user preference, DC charge |
| `PwrAcMax` | W | 41 | 9000 | user preference, AC charge |
| `ChgSocLimit` | % | 44 | 100 | charge target, AC and DC |

The series cell count is the constant `CELLS` (73).

## Flash record

Two 2 KB pages, 4 slots of 416 bytes each. On a 128 KB part page 0 is at `0x0801D800` and page 1 at
`0x0801D000` (`PE_BLKNUM` 5, `PE_PAGES` 2).

| Field | Type | Content |
|---|---|---|
| `magic` | uint32 | `0x52454646` ('REFF') |
| `generation` | uint32 | +1 per record |
| `version` | uint16 | `PE_VERSION` 3 |
| `reserved` | uint16 | 0 |
| `dis[100]` | uint16 | discharge table, 0.1 mΩ |
| `chg[100]` | uint16 | charge table, 0.1 mΩ |
| `crc` | uint32 | STM32 CRC over the fields above |

`LoadFromFlash()` takes the highest valid generation. `CommitFlash()` writes the next free slot.
When a page is full, it erases the other page and writes there.

`Task100Ms()` requests a save after the exit from RUN or CHARGE when a bin changed. `PollPersist()`
writes only in `MOD_OFF`: the erase runs with interrupts off.

CAUTION: A record with a different `PE_VERSION` is ignored. A change of the table axes without a new
`PE_VERSION` loads old values into the wrong bins.

## Consumers

| Path | File | Preference | Battery limit |
|---|---|---|---|
| Drive, T2C | `EvControlsT2C.cpp` | `PwrMotMax` | `BMS_MaxOutput` |
| Regen, T2C | `EvControlsT2C.cpp` | `PwrRegenMax` | `BMS_MaxRegen` |
| Drive and regen, other inverters | `throttle.cpp` | `PwrMotMax`, `PwrRegenMax` | - |
| AC charge, TeslaOI | `teslaCharger.cpp` | `PwrAcMax` | `BMS_MaxInput` |
| AC charge, Elcon | `ElconCharger.cpp` | `PwrAcMax` | `BMS_MaxInput` |
| DC charge, i3LIM | `i3LIM.cpp` | `PwrCcsMax` | `BMS_MaxInput`, in A at `udc` |
| DC charge, Foccci | `Foccci.cpp` | `PwrCcsMax` | `BMS_MaxInput`, in A at `udc` |
| DC charge, CHAdeMO | `chademo.cpp` | `PwrCcsMax` | `BMS_MaxInput`, in A at `udc`, maximum 150 A |

The T2C clamps the 0x696 discharge field to 0 to 650 kW and the regen field to 1 to 650 kW (uint16
at 0.01 kW). A 0 kW limit faults the drive unit until a key cycle. Regen is 0 kW on a full or cold
pack, so the regen field stays at 1 kW or more.

| Field | Limit |
|---|---|
| i3LIM `FC_Cur` (0x3E9, 10 bits) | 511 A |
| i3LIM `CHG_Pwr` (12 bits at 25 W) | 102.375 kW |
| Foccci `CCS_Ireq` | 16 bits |

## Terminal

| Command | Action |
|---|---|
| `pedump` | prints generation, page, slot, dirty flag, `BMS_Rpack`, `BMS_LimSrc` and both tables |
| `pereset` | loads the prefill and requests a save at the next `MOD_OFF` |

## Operating envelope

At rest, balanced pack, prefill tables, `BMS_VsagLimit` 2.6 V, `BMS_VdragLimit` 4.2 V, no thermal
ramps (`power_grapher.py`). `*` marks a value below `PwrMotMax` 150 kW.

Discharge, kW:

| T | 90 % | 70 % | 50 % | 30 % | 20 % | 10 % | 5 % | 2 % |
|---|---|---|---|---|---|---|---|---|
| -10 °C | *129 | *123 | *111 | *100 | *91 | *78 | *54 | *38 |
| 0 °C | 188 | 178 | 162 | *146 | *134 | *116 | *81 | *58 |
| 15 °C | 217 | 210 | 199 | 188 | 172 | *150 | *106 | *77 |
| 25 °C | 231 | 224 | 212 | 202 | 194 | 175 | *125 | *90 |
| 40 °C | 242 | 234 | 223 | 212 | 205 | 194 | *145 | *106 |

Discharge at 25 °C with one cell below the pack OCV, kW:

| Offset | 90 % | 70 % | 50 % | 30 % | 20 % | 10 % | 5 % | 2 % |
|---|---|---|---|---|---|---|---|---|
| -0.20 V | 216 | 209 | 197 | 184 | 166 | *141 | *98 | *64 |
| -0.40 V | 202 | 194 | 173 | *146 | *129 | *104 | *68 | *36 |
| -0.60 V | 184 | 167 | *136 | *105 | *88 | *61 | *34 | *3 |

Regen / charge, kW:

| T | 90 % | 80 % | 70 % | 50 % | 30 % | 20 % | 10 % |
|---|---|---|---|---|---|---|---|
| 0 °C | 13/4 | 19/5 | 24/8 | 38/11 | 49/21 | 50/26 | 53/29 |
| 5 °C | 18/5 | 25/7 | 32/9 | 51/14 | 66/26 | 67/33 | 72/36 |
| 10 °C | 24/6 | 34/8 | 43/11 | 69/17 | 90/32 | 91/40 | 97/44 |
| 15 °C | 33/7 | 46/10 | 59/14 | 93/21 | 122/39 | 124/49 | 133/54 |
| 25 °C | 57/10 | 80/14 | 102/20 | 164/30 | 215/57 | 221/71 | 238/79 |
| 40 °C | 68/16 | 94/24 | 120/33 | 191/50 | 250/95 | 256/119 | 276/134 |

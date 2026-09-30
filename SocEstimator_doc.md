# SOC estimator

This page describes the state of charge model in `src/soc_estimator.cpp` and `include/soc_estimator.h`.

## Why

The Dilithium BMS calculates SOC from an energy counter and a fixed 58.3 kWh. The counter uses the pack mean and has no OCV correction.

| Condition | BMS result |
|---|---|
| Near empty | 32 % while the weakest cell group was at 2.77 V |
| Near full | The counter recorded 51 % to 61 % of the measured energy |

One cell group on this pack holds less charge at the bottom of the curve. The pack mean does not show it. The estimator tracks the weakest cell group.

## Principle

- `f` is the charge of the weakest cell group as a fraction of absolute capacity. `f` is 0 at 2800 mV and 1 at 4200 mV.
- Coulomb counting moves `f` on every tick.
- At the end of a rest, the OCV table corrects `f`.
- `BMS_VminLimit` and `BMS_VmaxLimit` set the displayed window. They do not change `f` or the learned capacity.
- The code learns `BMS_CapActual` from discharge between rests and `BMS_WhPerKm` from driving.
- `f`, `BMS_CapActual` and `BMS_WhPerKm` stay in flash across power cycles.

## Data flow

```
idc ------------------------------> coulomb count -----> f
BMS_Vmin, rest -------------------> OCV table ---------> correction of f
f, BMS_CapActual, limits ---------> window ------------> SOC, AMPh, KWh, BMS_KmRem
BMS_Vmin, idc, BMS_Rpack ---------> IR compensation ---> BMS_OcvCell
udc2, idc, Veh_Speed -------------> 20 km window ------> BMS_WhPerKm
f, BMS_CapActual, BMS_WhPerKm ----> record, outside RUN and CHARGE
```

## Functions

| Function | Rate | Action |
|---|---|---|
| `Init()` | start-up | Loads the record |
| `Task100Ms()` | 100 ms, after `PowerLimiter::Task100Ms()` | Counts the charge, corrects `f`, learns, publishes |
| `PollPersist()` | main loop | Writes the record outside RUN and CHARGE |
| `DumpRecord()` | terminal | Prints `f`, `BMS_CapActual` and `BMS_WhPerKm` |
| `ResetRecord()` | terminal | Sets the defaults, clears `f`, writes the record |

## Tick order

`Task100Ms()` does these steps in this order:

1. At the start of RUN or CHARGE, the code clears the correction flag. It keeps a completed rest timer.
2. The code reads `BMS_Vmin`. A value below 1000 is in volts, and the code converts it to mV.
3. If `BMS_Vmin` is outside the valid range, the tick stops. The published values hold.
4. The code calculates the displayed window.
5. Coulomb counting updates `f`.
6. The code updates the rest timer.
7. The code publishes `BMS_OcvCell`.
8. At the end of a rest, the code corrects `f` and takes a capacity sample.
9. In RUN, the code adds to the Wh/km sum.
10. The code publishes the values.

| Item | Value |
|---|---|
| Valid `BMS_Vmin` range | 1500 mV to 4500 mV |

## Coulomb counting

Each tick adds the charge of that tick, divided by `BMS_CapActual`, to `f`:

```
f = f + idc * 0.1 / 3600 / BMS_CapActual
```

`idc` is positive during charge, so discharge lowers `f`. The code also sums the Ah discharged since the last rest, for capacity learning.

| Item | Value |
|---|---|
| Tick | 0.1 s |
| Condition | `f` known, `BMS_CapActual` at least 50 Ah |
| Range of `f` | 0 to 1 |

## Rest correction

At the end of each rest, the code reads the table value at `BMS_Vmin`. If flash holds no `f`, the table value sets `f`. Otherwise `f` moves a fixed fraction toward the table value.

| Item | Value |
|---|---|
| Rest current | below 6 A |
| Rest time | 15 s |
| Correction gain | 0.03 |
| Corrections per rest | 1 |

A current at or above the rest current ends the rest. If the pack rested before the start of RUN or CHARGE, the first tick applies the correction.

## Displayed window

The code maps `BMS_VminLimit` and `BMS_VmaxLimit` through the OCV table on every tick. The limits are in volts. `SOC` is 0 at `BMS_VminLimit` and 100 at `BMS_VmaxLimit`.

| Item | Value |
|---|---|
| Minimum window span | 0.05 in `f` |
| Action on a smaller span | Keep the previous window |

## Capacity learning

A capacity sample is the Ah discharged between two consecutive rests, divided by the drop in `f`. Every rest starts a new sample. The sample pair is in RAM, so a restart starts a new pair.

| Item | Value |
|---|---|
| Minimum drop in `f` | 0.25 |
| Tolerance to present value | 30 % |
| EWMA gain | 0.10 |
| Default | 190 Ah |

`BMS_CapActual` is the capacity between 2800 mV and 4200 mV. Pack ageing lowers this value.

## Wh/km learning

`BMS_WhPerKm` is the net pack energy per km. In RUN and above the minimum speed, the code sums `udc2 * -idc`. `idc` is positive during regeneration, so regeneration lowers the sum. At the end of each window, the code applies an EWMA if the sample is inside the accepted range.

| Item | Value |
|---|---|
| Window | 20 km |
| Minimum speed | 5 km/h |
| Accepted range | 50 Wh/km to 500 Wh/km |
| EWMA gain | 0.20 |
| Default | 130 Wh/km |

## OCV table

The table maps the rest voltage of the weakest cell group to `f` for the LG INR21700-M50. `f` is the Ah value divided by 190. `TableF()` interpolates linearly and limits the result to 0 to 1. The table has no temperature axis.

| mV | Ah | Source |
|---|---|---|
| 2800 | 0.0 | Rest data from drive logs |
| 2956 | 1.6 | Rest data from drive logs |
| 3038 | 2.6 | Rest data from drive logs |
| 3118 | 3.6 | Rest data from drive logs |
| 3134 | 4.6 | Rest data from drive logs |
| 3167 | 5.6 | Rest data from drive logs |
| 3228 | 6.6 | Rest data from drive logs |
| 3255 | 7.6 | Rest data from drive logs |
| 3281 | 9.6 | Rest data from drive logs |
| 3335 | 10.6 | Rest data from drive logs |
| 3360 | 11.6 | Rest data from drive logs |
| 3368 | 12.1 | Rest data from drive logs |
| 3509 | 36.3 | Rest data from drive logs |
| 3542 | 45.6 | Rest data from drive logs |
| 3572 | 57.0 | Rest data from drive logs |
| 3593 | 65.7 | Rest data from drive logs |
| 3658 | 85.1 | Rest data from drive logs |
| 3700 | 93.8 | Linear interpolation, 3658 mV to 4042 mV |
| 3800 | 114.6 | Linear interpolation, 3658 mV to 4042 mV |
| 3900 | 135.4 | Linear interpolation, 3658 mV to 4042 mV |
| 4000 | 156.2 | Linear interpolation, 3658 mV to 4042 mV |
| 4042 | 164.9 | Rest data from drive logs |
| 4075 | 180.0 | Rest data from drive logs |
| 4200 | 190.0 | Sets `f` to 1.0 |

## Published values

| Value | ID | Unit | Meaning |
|---|---|---|---|
| `SOC` | 2015 | % | 0 at `BMS_VminLimit`, 100 at `BMS_VmaxLimit` |
| `AMPh` | 2014 | Ah | Charge above `BMS_VminLimit` |
| `KWh` | 2013 | kWh | `AMPh` at 270 V nominal |
| `BMS_KmRem` | 2124 | km | `KWh` divided by `BMS_WhPerKm`, limited to 0 to 999 |
| `BMS_CapUsable` | 2126 | Ah | Window span times `BMS_CapActual` |
| `BMS_KwhUsable` | 2129 | kWh | `BMS_CapUsable` at 270 V nominal |
| `BMS_KwhActual` | 2130 | kWh | `BMS_CapActual` at 270 V nominal |
| `BMS_OcvCell` | 2123 | mV | Weakest cell group with IR compensation, display only |

The nominal pack voltage is 3.7 V times 73 cells. The live pack voltage is not used. The code publishes `BMS_CapUsable`, `BMS_KwhUsable` and `BMS_KwhActual` also when `f` is unknown.

## Parameters

| Entry | Type | ID | Default | Role |
|---|---|---|---|---|
| `BMS_VminLimit` | param, V | 92 | 3.0 | Lower end of the displayed window |
| `BMS_VmaxLimit` | param, V | 93 | 4.2 | Upper end of the displayed window |
| `BMS_CapActual` | param, Ah | 160 | 190 | Learned capacity |
| `BMS_WhPerKm` | param, Wh/km | 161 | 130 | Learned net consumption |

## Record

The record is 20 bytes.

| Word | Content |
|---|---|
| 0 | Magic `0x34434F53` |
| 1 | `BMS_CapActual` |
| 2 | `BMS_WhPerKm` |
| 3 | `f` |
| 4 | STM32 hardware CRC of words 0 to 3 |

The page address is `FLASH_BASE + flash_size * 1024 - SOC_BLKNUM * FLASH_PAGE_SIZE`. `SOC_BLKNUM` is `PE_BLKNUM + PE_PAGES`, which puts the page below the `PowerLimiter` journal. `FLASH_PAGE_SIZE` is 2048 bytes.

`PollPersist()` writes the record outside RUN and CHARGE, if a value moved more than its threshold.

| Value | Threshold |
|---|---|
| `BMS_CapActual` | 0.5 Ah |
| `BMS_WhPerKm` | 2 Wh/km |
| `f` | 0.0001 |

A write erases the page. Interrupts stay disabled during the erase and the write. The code resets the watchdog before and after.

`Init()` loads the record. A blank page, a bad CRC or a value out of range leaves the parameter values in place. A parameter value out of range returns to its default.

> **CAUTION**
> Keep `SOC_BLKNUM` equal to `PE_BLKNUM + PE_PAGES`. Another value can overlap the `PowerLimiter` journal or the parameter page. The next write then destroys that data.

> **NOTE**
> Call `PollPersist()` from the main loop only. An erase from an interrupt blocks the CAN bus and the watchdog.

## Terminal

| Command | Action |
|---|---|
| `socdump` | Prints the record |
| `socreset` | Sets `BMS_CapActual` to 190 and `BMS_WhPerKm` to 130, clears `f`, writes the record |
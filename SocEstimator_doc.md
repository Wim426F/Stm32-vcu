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
- Coulomb counting moves `f` on every tick outside OFF.
- At the start of RUN or CHARGE after 30 min of settle time, the start correction sets `f` to the table value.
- After a shorter stop or a 15 s rest, the rest correction moves `f` toward the table value.
- `BMS_VminLimit` and `BMS_VmaxLimit` set the displayed window. They do not change `f` or the learned capacity.
- The code learns `BMS_CapActual` from discharge between two start corrections and `BMS_WhPerKm` from driving.
- `f`, `BMS_CapActual` and `BMS_WhPerKm` stay in flash across power cycles.

## Data flow

```
idc, opmode ----------------------> coulomb count -----> f
opmode, idc ----------------------> settle time -------> start correction or rest correction
BMS_Vmin, start or rest ----------> OCV table ---------> correction of f
f, BMS_CapActual, limits ---------> window ------------> SOC, AMPh, KWh, BMS_KmRem
BMS_Vmin, idc, BMS_Rpack ---------> IR compensation ---> BMS_OcvCell
udc2, idc, Veh_Speed -------------> 20 km window ------> BMS_WhPerKm
f, BMS_CapActual, BMS_WhPerKm ----> record, in OFF only
```

## Functions

| Function | Rate | Action |
|---|---|---|
| `Init()` | start-up | Loads the record, publishes the values |
| `Task100Ms()` | 100 ms, before `PowerLimiter::Task100Ms()` | Counts the charge, corrects `f`, learns, publishes |
| `PollPersist()` | main loop | Writes the record in OFF |
| `DumpRecord()` | terminal | Prints `f`, `BMS_CapActual` and `BMS_WhPerKm` |
| `ResetRecord()` | terminal | Sets the defaults, clears `f`, writes the record |

## Tick order

`Task100Ms()` does these steps in this order:

1. In OFF, the code adds one tick to the settle time.
2. The code reads `BMS_Vmin`. A value below 1000 is in volts, and the code converts it to mV.
3. If `BMS_Vmin` is outside the valid range, the code publishes the values from the last `f` and stops the tick.
4. Outside OFF, coulomb counting updates `f`.
5. Outside OFF, the code updates the settle time from `idc`.
6. The code updates the rest timer.
7. The code publishes `BMS_OcvCell`.
8. The code detects the start of RUN or CHARGE.
9. The code applies the start correction or the rest correction.
10. In RUN, the code adds to the Wh/km sum.
11. The code calculates the displayed window and publishes the values.

Step 1 runs also when `BMS_Vmin` is invalid. Step 8 compares `opmode` with the `opmode` of the last valid tick. A start during invalid `BMS_Vmin` is detected at the first valid tick.

| Item | Value |
|---|---|
| Valid `BMS_Vmin` range | 1500 mV to 4500 mV |

## Coulomb counting

Each tick adds the charge of that tick, divided by `BMS_CapActual`, to `f`:

```
f = f + idc * 0.1 / 3600 / BMS_CapActual
```

`idc` is positive during charge, so discharge lowers `f`. The code also sums the Ah discharged since the last start correction, for capacity learning.

The contactors are open in OFF, so `idc` in OFF is sensor offset. The code does not count in OFF. Below the settle current, `idc` is mostly sensor offset, so the code does not count. The next start correction removes the error of the skipped current.

| Item | Value |
|---|---|
| Tick | 0.1 s |
| Condition | `opmode` not OFF, `idc` at or above 2 A, `f` known, `BMS_CapActual` at least 50 Ah |
| Range of `f` | 0 to 1 |

## Start correction

At the start of RUN or CHARGE, the code reads the settle time. If the settle time is 30 min or more, the code sets `f` to the table value at `BMS_Vmin`. The code then takes a capacity sample.

The settle time is the time the pack rests. These conditions add one tick to the settle time:

- `opmode` is OFF. The contactors are open. The counter also runs when the BMS sends no data.
- `opmode` is not OFF, `BMS_Vmin` is valid, and `idc` is below the settle current. An AC charge hold at 0 W is an example.

Outside OFF, a current at or above the settle current sets the settle time to 0. The settle current is above the zero offset of the current sensor.

| Item | Value |
|---|---|
| Minimum settle time | 30 min (18000 ticks) |
| Settle current, outside OFF | below 2 A |
| Settle time at start-up | 30 min |
| Settle time after a start of RUN or CHARGE | 0 |

After a restart during a drive, the record holds a stale `f`. The settle time at start-up is 30 min, so the first start correction replaces that `f`.

## Rest correction

A rest correction moves `f` a fixed fraction toward the table value at `BMS_Vmin`. If `f` is unknown, the rest correction sets `f` to the table value. A rest correction takes no capacity sample.

These events apply a rest correction:

- the end of a rest
- a start of RUN or CHARGE with a settle time below 30 min

| Item | Value |
|---|---|
| Rest current | below 6 A |
| Rest time | 15 s |
| Correction gain | 0.03 |
| Corrections per rest | 1 |

After a short stop, `BMS_Vmin` is below the settled rest voltage, so the gain is small. A current at or above the rest current ends the rest. A start correction is also the correction of that rest.

## Displayed window

The code maps `BMS_VminLimit` and `BMS_VmaxLimit` through the OCV table at each publish. The limits are in volts. `SOC` is 0 at `BMS_VminLimit` and 100 at `BMS_VmaxLimit`.

| Item | Value |
|---|---|
| Minimum window span | 0.05 in `f` |
| Action on a smaller span | Keep the previous window |

## Capacity learning

A capacity sample is the Ah discharged between two consecutive start corrections, divided by the drop in `f`. Every start correction starts a new sample. The sample pair is in RAM, so a restart starts a new pair.

A rest correction takes no sample. After a short rest, `BMS_Vmin` reads low, and the full error goes into the drop in `f`.

| Item | Value |
|---|---|
| Minimum drop in `f` | 0.25 |
| Tolerance to present value | 30 % |
| EWMA gain | 0.10 |
| Default | 215 Ah |

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

The table maps the rest voltage of the weakest cell group to `f` for the LG INR21700-M50. `f` is the Ah value divided by 215. `TableF()` interpolates linearly and limits the result to 0 to 1. The table has no temperature axis.

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
| 3798 | 118.8 | Rest voltage from drive logs, Ah from the slopes next to the gaps |
| 3877 | 134.7 | Rest voltage from drive logs, Ah from the slopes next to the gaps |
| 3937 | 152.3 | Rest voltage from drive logs, Ah from the slopes next to the gaps |
| 4042 | 189.7 | Rest voltage from drive logs, Ah from the slopes next to the gaps |
| 4075 | 204.8 | Rest voltage from drive logs, Ah from the slopes next to the gaps |
| 4200 | 215.0 | Sets `f` to 1.0 |

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

`Init()` publishes the values from the record. When `BMS_Vmin` is invalid, the code publishes the values from the last `f`. `SOC` never drops to 0 because of missing BMS data.

## Parameters

| Entry | Type | ID | Default | Role |
|---|---|---|---|---|
| `BMS_VminLimit` | param, V | 92 | 3.0 | Lower end of the displayed window |
| `BMS_VmaxLimit` | param, V | 93 | 4.2 | Upper end of the displayed window |
| `BMS_CapActual` | param, Ah | 160 | 215 | Learned capacity |
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

`PollPersist()` writes the record in OFF only, if a value moved more than its threshold. The contactors are open in OFF.

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
| `socreset` | Sets `BMS_CapActual` to 215 and `BMS_WhPerKm` to 130, clears `f`, writes the record |
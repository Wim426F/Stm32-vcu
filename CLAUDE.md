# CAN Bus Refactoring Plan

## Goal
Replace the old monolithic CAN callback architecture with per-subsystem callbacks, making all 4 buses (CAN1/CAN2 internal, CAN3 MCP25625, CAN4 MCP251863) interchangeable. Fix the bug where same CAN ID on different buses collides because the callback has no bus awareness.

## Build Config
- `MAX_USER_MESSAGES=64` (CAN ID filter slots per bus, was 30)
- `MAX_RECV_CALLBACKS=32` (callback slots per bus, was 5)
- Both defined in Makefile as `-D` flags, override the defaults in `libopeninv/include/canhardware.h`
- RAM cost: ~3KB total across all buses, trivial for 64KB STM32F107

## libopeninv submodule
- Pointed to fork `https://github.com/Wim426F/libopeninv.git`, synced with upstream `jsphuebner/libopeninv` master
- Local additions on top of upstream:
  - `RemoveCallback(CanCallback*)` in `canhardware.h`/`canhardware.cpp` — replaces slot with NullCallback
  - `AddCallback` hardened: skips duplicates, reuses null slots before appending (prevents slot exhaustion across rebinds)
- Upgrade brought in: interrupt-safe `Stm32Can::Send()`, `Baud33`, `SendByIndex()`, canmap off-by-one fixes, `SdoCommands` extracted (now in `sdocommands.cpp/h`), CANopen param flags & `CLEAR_CAN`
- Project must poll `sdo.GetPendingUserspaceSdo()` in main loop and call `SdoCommands::ProcessStandardCommands` + `SendSdoReply` — otherwise save/load/reset/defaults over CAN silently break

## CanRxBinding — the core abstraction (in stm32_vcu.cpp)
Single class with function pointers, instantiated per subsystem:
- Constructor takes `void (*rx)(uint32_t, uint32_t[2])` and `void (*clear)()`
- `HandleRx` calls the rx function pointer (the handler)
- `HandleClear` calls the clear function pointer (re-registers CAN IDs on correct bus)
- Non-capturing lambdas decay to function pointers — zero overhead vs static functions

### The 11 instances
1. `inverterCanRx` — `selectedInverter->DecodeCAN`, bus from `InverterCan`
2. `vehicleCanRx` — `selectedVehicle->DecodeCAN`, bus from `VehicleCan`
3. `shifterCanRx` — `selectedShifter->DecodeCAN`, bus from `VehicleCan` (no ShifterCan param yet)
4. `chargerCanRx` — `selectedCharger->DecodeCAN`, bus from `ChargerCan`
5. `chargeIntCanRx` — `selectedChargeInt->DecodeCAN`, bus from `LimCan`
6. `bmsCanRx` — `selectedBMS->DecodeCAN(id, (uint8_t*)data)`, bus from `BMSCan`
7. `dcdcCanRx` — `selectedDCDC->DecodeCAN(id, (uint8_t*)data)`, bus from `DCDCCan`
8. `heaterCanRx` — `selectedHeater->DecodeCAN`, bus from `HeaterCan`
9. `shuntCanRx` — ISA/SBOX/VWBOX/HVCU static DecodeCAN switched on ShuntType, bus from `ShuntCan`
10. `obd2CanRx` — `canOBD2.DecodeCAN`, bus from `OBD2Can`
11. `sdoFilterCanRx` — no-op HandleRx, HandleClear registers 0x601 on both internal CAN buses (CAN1+CAN2) for CanSDO

`canBindings[]` table pairs each binding with its `Param::PARAM_NUM` bus param so `RebindAllCanCallbacks()` and `main()` initial registration share one source of truth.

## How ClearUserMessages/HandleClear Flow Works
1. `Update*()` or `Param::Change` calls `ClearUserMessages()` on all 4 buses (CAN3/CAN4 null-guarded — they may not exist)
2. `ClearUserMessages` resets the filter list, calls `ConfigureFilters()`, then calls `HandleClear()` on every callback registered on that bus
3. Each binding's `HandleClear` calls `subsystem->SetCanInterface(canInterface[bus_param])` which re-registers that subsystem's CAN IDs via `RegisterUserMessage`
4. `RegisterUserMessage` deduplicates and calls `ConfigureFilters()` after each add
5. End result: all subsystem IDs re-registered on their correct buses

## Phases

### Phase 1 — Per-subsystem callbacks (DONE)
- Replaced single `CanCallback` function + `SetCanFilters` with `CanRxBinding` class + 11 instances
- Each has its own HandleRx (→ DecodeCAN) and HandleClear (→ SetCanInterface)
- TX path completely untouched on CAN1/CAN2 — `can->Send()` works everywhere as before
- No subsystem code changed (BMW_E90.cpp, leafinv.cpp, etc.)

### Phase 2 — Bus-specific binding (DONE)
- Each binding registered ONLY on its designated bus via the `canBindings[]` table
- `sdoFilterCanRx` stays on both internal buses (CAN1+CAN2) since the SDO server lives on whichever the user prefers
- `RebindAllCanCallbacks()` removes all subsystem bindings from all buses, re-adds each on `canInterface[Param::GetInt(busParam)]`, then `ClearUserMessages` on all three buses
- `Param::Change` calls `RebindAllCanCallbacks()` for all 9 CAN-bus params (`InverterCan`, `VehicleCan`, `ShuntCan`, `LimCan`, `ChargerCan`, `BMSCan`, `OBD2Can`, `DCDCCan`, `HeaterCan`)
- Update*() functions clear all 3 buses; each binding's HandleClear reads its own bus param so IDs land correctly regardless of which bus the binding sits on
- Fixes same-ID-different-bus collision bug

### Phase 3 — CAN3 MCP25625 wrapper (DONE)
- `Mcp25625Can : CanHardware` in `include/mcp25625can.h` / `src/mcp25625can.cpp` — wraps low-level `MCP2515_*` register primitives (CANSPI.cpp's high-level wrapper is unused by subsystems now but kept as primitives)
- `Send(canId, data, len)`: derives standard/extended from `canId > 0x7FF`, packs ID via `convertCANid2Reg`, loads first free TX buffer, RTS — drops frame if all 3 TX buffers busy
- `HandleInterrupt()`: drains RX in a loop while `MCP2515_Get_RxStatus()` reports a buffered frame, reads via `MCP2515_Read_RxbSequence`, packs to `uint32_t data[2]`, calls base-class `HandleRx`, clears the IF flag, re-polls
- `SetBaudrate()`: only `Baud33` and `Baud500` (no Baud100 in libopeninv enum — old CAN3Speed=2 dropped)
- `ConfigureFilters()`: if `nextUserMessageIndex` ≤ 6 AND no extended IDs AND no masked entries → tight mode (mask 0x7FF on both RXB, one ID per filter); otherwise permissive (mask 0, accept-all + software filter in `DecodeCAN`)
- EXTI15 ISR now just does `exti_reset_request` + `Mcp25625Can::GetInstance()->HandleInterrupt()` — hardcoded `if(id==0x108||id==0x109)` dispatch is gone
- `Mcp25625Can mcp(...)` instantiated in `main()` after `c2`, assigned to `canInterface[2]` before CanMap/CanSdo construction
- `Param::Change(CAN3Speed)` calls `canInterface[2]->SetBaudrate(...)` + `ClearUserMessages`
- All `Update*()` and `RebindAllCanCallbacks()` clear CAN3 too (with null guard, since `canInterface[2]` is nullptr until `mcp` is constructed)
- AmperaHeater migrated: uses inherited `can->Send()` from `Heater` base class, dropped `CANSPI_Transmit`/`uCAN_MSG`; sw_mode GPIO sequence in `SendWakeup` stays (transceiver-level, not protocol-level)
- FCChademo migrated: added `SetCanInterface` override that registers 0x108/0x109 on the assigned bus; `Task100Ms` uses `can->Send()` for 0x100/0x101/0x102 frames
- `include/heater.h` no longer pulls in `CANSPI.h` (was leaking into every Heater-using TU); same for `include/chademo.h`
- Param changes: `CAN_DEV` string → "0=CAN1, 1=CAN2, 2=CAN3"; max 1→2 on 9 CAN-bus params (CanMapCan kept at 1); `CAN3Speed` max 2→1, `CAN3SPD` string → "0=k33.3, 1=k500"
- **User-visible behavior change**: AmperaHeater requires `HeaterCan=2`; CHAdeMO on CAN3 requires `LimCan=2`. Previously CAN3 was hardcoded for both.

### Phase 4 — CAN4 MCP251863 driver (DONE, Classic CAN only)
- MCP251863 = MCP2518FD CAN-FD controller + ATA6563 transceiver, on v1.2 boards only
- **Different SPI protocol from MCP2515**: 16-bit commands (4-bit opcode + 12-bit address), cannot reuse `MCP2515.cpp`. New low-level driver in `include/mcp2518fd.h` / `src/mcp2518fd.cpp` (Reset/ReadByte/ReadWord/WriteByte/WriteWord/ReadSeq/WriteSeq).
- `Mcp251863Can : CanHardware` in `include/mcp251863can.h` / `src/mcp251863can.cpp` — mirrors `Mcp25625Can` API shape
- **Hardware (user-confirmed v1.2)**:
  - SPI2 shared with CAN3 (different CS pins)
  - CS = PE10 → `DigIo::mcp4_cs`
  - INT = PE11 → EXTI11, falling-edge, falls into same `exti15_10_isr` handler as CAN3's EXTI15
  - Standby = PE12 → `DigIo::mcp4_sby` (active-low to enable transceiver)
  - Crystal = 16 MHz, no PLL, SYSCLK direct
- **Auto-detection**: constructor probes by polling `OSC.OSCRDY` for ~5 ms. If reads return all-0x00 or all-0xFF (chip absent on v1.1 boards), `present` stays false and the static `instance` pointer stays null; `canInterface[3]` is set to nullptr. All `RebindAllCanCallbacks` / `Update*()` clears are null-guarded for index 3. One binary works on both v1.1 and v1.2.
- **Classic CAN 2.0B only**: chip set to mode 6 (`OPMOD_NORMAL_CAN2`) after config. FD mode left as a future opt-in.
- **Bit timing for 500 kbps at 16 MHz**: `BRP=0, TSEG1=22, TSEG2=7, SJW=7` → 32 TQ, 75% sample point. Encoded in `C1NBTCFG` as `0x00160707`.
- `Send`: checks TXQ has free slot (`C1TXQSTA.TXQNIF`), reads `C1TXQUA`, writes 16-byte payload (T0 ID header + T1 control/DLC + 8 data bytes) at `0x400 + (ua & 0xFFF)`, then sets `UINC | TXREQ` in `C1TXQCON` to advance and transmit.
- `HandleInterrupt`: drains `FIFO1` while `C1FIFOSTA1.TFNRFNIF` set — reads 16 bytes from `0x400 + (ua & 0xFFF)`, unpacks T0/T1, dispatches via base-class `HandleRx`, advances pointer with `UINC` on `C1FIFOCON1`. Then clears `C1RXIF`.
- `ConfigureFilters`: 32 filter+mask pairs available. One-to-one with `userIds[]` — exact-match filters set `MIDE` in mask to require IDE bit agreement. Extended IDs use `EXIDE` bit in filter object. `C1FLTCON0..7` enables each filter, routes all to FIFO1. >32 IDs: extras dropped (very unlikely).
- ISR shared with CAN3:
  ```cpp
  extern "C" void exti15_10_isr() {
      if (exti_get_flag_status(EXTI15)) { exti_reset_request(EXTI15); Mcp25625Can::GetInstance()->HandleInterrupt(); }
      if (exti_get_flag_status(EXTI11)) { exti_reset_request(EXTI11); Mcp251863Can::GetInstance()->HandleInterrupt(); }
  }
  ```
  Both share NVIC entry → can't preempt each other → natural serialization on shared SPI2 bus.
- `canInterface[3]` populated in `main()` only when `mcp4.IsPresent()`; otherwise stays nullptr. `RebindAllCanCallbacks()` null-guards both the `RemoveCallback` and the `AddCallback` paths (so picking `CAN_DEV=3` on v1.1 hardware doesn't crash, just doesn't reach the subsystem).
- Param changes: `CAN_DEV` string → "0=CAN1, 1=CAN2, 2=CAN3, 3=CAN4"; max 2→3 on 9 CAN-bus params (CanMapCan stays at 1); new `CAN4Speed` param (ID 150, range 0-0 — only 500 kbps for now), `CAN4SPD` define "0=k500".
- **Hardware bring-up TODO**: bit timing, filter encoding, and TXQ data layout all need confirmation on a real v1.2 board with scope/analyzer before declaring fully validated.
- **Future CAN-FD work**: add `CAN4FD` param, configure `C1DBTCFG`, handle FDF/BRS bits in T1 header, extend PLSIZE beyond 8 bytes, expose larger DLCs through `Send`.

## Key Files
- `src/stm32_vcu.cpp` — main VCU logic, CanRxBinding class, `canBindings[]` table, `RebindAllCanCallbacks`, CAN3/CAN4 instantiation, shared `exti15_10_isr` ISR (EXTI15 + EXTI11), SDO userspace poll loop
- `libopeninv/include/canhardware.h` / `src/canhardware.cpp` — base class with `AddCallback`/`RemoveCallback`/`RegisterUserMessage`
- `libopeninv/include/stm32_can.h` / `src/stm32_can.cpp` — STM32 native CAN driver (CAN1, CAN2)
- `include/mcp25625can.h` / `src/mcp25625can.cpp` — CAN3 CanHardware wrapper
- `include/CANSPI.h` / `src/CANSPI.cpp` — legacy MCP25625 high-level API (kept for `convertCANid2Reg` helpers; not used by subsystems anymore)
- `include/MCP2515.h` / `src/MCP2515.cpp` — Low-level MCP2515/25625 register access
- `include/mcp251863can.h` / `src/mcp251863can.cpp` — CAN4 CanHardware wrapper (auto-detect, classic CAN only)
- `include/mcp2518fd.h` / `src/mcp2518fd.cpp` — Low-level MCP2518FD register access (16-bit SPI command protocol)
- `src/amperaheater.cpp` — TX migrated to `can->Send()`
- `src/chademo.cpp` — TX and RX-filter registration migrated to CanHardware
- `include/digio_prj.h` — adds `mcp4_cs` (PE10) and `mcp4_sby` (PE12) for CAN4
- `src/hwinit.cpp` — `nvic_setup()` enables both EXTI15 (CAN3) and EXTI11 (CAN4)
- `include/param_prj.h` — Parameter definitions: `CAN_DEV` 0-3, `CAN3Speed`, `CAN4Speed`

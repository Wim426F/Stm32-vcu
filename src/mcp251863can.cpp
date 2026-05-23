/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Wim Boone <wimboone38@gmail.com>
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
#include "mcp251863can.h"
#include "mcp2518fd.h"
#include "digio.h"
#include <libopencm3/stm32/rtc.h>

// CiCON bit layout
#define CON_REQOP_SHIFT    24
#define CON_REQOP_MASK     (0x7u << CON_REQOP_SHIFT)
#define CON_OPMOD_SHIFT    21
#define CON_OPMOD_MASK     (0x7u << CON_OPMOD_SHIFT)
#define CON_TXQEN          (1u << 20)

// CiFIFOCONn bit layout
#define FIFO_PLSIZE_8B     (0u << 29)
#define FIFO_FSIZE_8       (7u << 24)
#define FIFO_TXEN          (1u << 7)        // bit 7 = TXEN per datasheet
#define FIFO_TFNRFNIE      (1u << 0)        // bit 0 = Tx/Rx FIFO not full/empty IE
#define FIFO_UINC          (1u << 8)        // bit 8 of upper byte = UINC (CON byte1)
#define FIFO_TXREQ         (1u << 9)        // bit 9 = TXREQ
#define FIFO_FRESET        (1u << 10)       // bit 10 = FRESET

// CiFLTCONn byte layout (4 filters per 32-bit register)
#define FLTCON_ENABLE      0x80
#define FLTCON_FIFO_MASK   0x1F

// CiMASKn bits
#define MASK_MIDE          (1u << 30)

// CiINT bits
#define INT_RXIE           (1u << 17)
#define INT_RXIF           (1u << 1)

// Filter object (CiFLTOBJn) — EXIDE bit
#define FLTOBJ_EXIDE       (1u << 30)

Mcp251863Can* Mcp251863Can::instance = 0;

static void delayUs(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops * 8; i++) { __asm__("nop"); }
}

Mcp251863Can::Mcp251863Can(enum baudrates baudrate)
    : present(false)
{
    DigIo::mcp4_sby.Clear();        // enable transceiver
    delayUs(100);

    Mcp2518Fd_Reset();
    delayUs(100);

    if (!ProbeChip())
        return;                     // No chip detected; leave instance null

    present = true;
    instance = this;

    ConfigureController(baudrate);
}

bool Mcp251863Can::ProbeChip()
{
    // Poll OSC register for OSCRDY for up to ~5 ms
    for (int i = 0; i < 500; i++)
    {
        uint32_t osc = Mcp2518Fd_ReadWord(MCP2518FD_OSC);
        if (osc == 0x00000000 || osc == 0xFFFFFFFF)
        {
            delayUs(10);
            continue;
        }
        if (osc & OSC_OSCRDY)
            return true;
        delayUs(10);
    }
    return false;
}

void Mcp251863Can::EnterMode(uint32_t opmode)
{
    uint32_t con = Mcp2518Fd_ReadWord(MCP2518FD_C1CON);
    con &= ~CON_REQOP_MASK;
    con |= (opmode << CON_REQOP_SHIFT);
    Mcp2518Fd_WriteWord(MCP2518FD_C1CON, con);

    for (int i = 0; i < 100; i++)
    {
        con = Mcp2518Fd_ReadWord(MCP2518FD_C1CON);
        if (((con & CON_OPMOD_MASK) >> CON_OPMOD_SHIFT) == opmode)
            return;
        delayUs(10);
    }
}

void Mcp251863Can::ConfigureController(enum baudrates baudrate)
{
    EnterMode(MCP2518FD_OPMOD_CONFIG);

    SetBaudrate(baudrate);

    // TXQ: 8 deep, payload 8 bytes, reset
    Mcp2518Fd_WriteWord(MCP2518FD_C1TXQCON, (7u << 24) | (1u << 10));
    // FIFO1 as RX: 8 deep, payload 8 bytes, FRESET + RxFIFO-not-empty IE
    Mcp2518Fd_WriteWord(MCP2518FD_C1FIFOCON1, (7u << 24) | (1u << 10) | (1u << 0));

    // Re-enable TXQ in CiCON
    uint32_t con = Mcp2518Fd_ReadWord(MCP2518FD_C1CON);
    con |= CON_TXQEN;
    Mcp2518Fd_WriteWord(MCP2518FD_C1CON, con);

    ConfigureFilters();

    // Enable RX FIFO interrupt at the CAN module level
    Mcp2518Fd_WriteWord(MCP2518FD_C1INT, INT_RXIE);

    EnterMode(MCP2518FD_OPMOD_NORMAL_CAN2);
}

void Mcp251863Can::SetBaudrate(enum baudrates baudrate)
{
    // 16 MHz SYSCLK direct from crystal
    // C1NBTCFG: [BRP:31..24][TSEG1:23..16][TSEG2:15..8][SJW:7..0]
    uint32_t btcfg;
    switch (baudrate)
    {
    case Baud500:
    default:
        // BRP=0 (1:1), TSEG1=22, TSEG2=7, SJW=7 -> 32TQ, 75% sample, 500 kbps
        btcfg = (0u << 24) | (22u << 16) | (7u << 8) | 7u;
        break;
    }
    Mcp2518Fd_WriteWord(MCP2518FD_C1NBTCFG, btcfg);
}

// Encode a CAN ID into the MCP2518FD's filter/message header layout.
// Standard: bits 10..0 of ID -> bits 10..0 of T0.
// Extended: top 11 bits of 29-bit ID -> SID (bits 28..18); bottom 18 bits -> EID (bits 17..0).
static uint32_t encodeId(uint32_t canId, bool extended)
{
    if (extended)
    {
        uint32_t id = canId & 0x1FFFFFFFu;
        uint32_t sid = (id >> 18) & 0x7FFu;
        uint32_t eid = id & 0x3FFFFu;
        return (sid << 0) | (eid << 11);
    }
    return canId & 0x7FFu;
}

void Mcp251863Can::Send(uint32_t canId, uint32_t data[2], uint8_t len)
{
    // Check TXQ has space (TXQNIF = bit 0 of CiTXQSTA)
    uint32_t sta = Mcp2518Fd_ReadWord(MCP2518FD_C1TXQSTA);
    if ((sta & 1u) == 0)
        return;     // No free slot, drop frame

    // Read user address (where next TXQ entry goes)
    uint32_t ua = Mcp2518Fd_ReadWord(MCP2518FD_C1TXQUA);

    bool extended = canId > 0x7FFu;
    uint32_t t0 = encodeId(canId, extended);
    uint32_t t1 = (len & 0x0Fu) | (extended ? (1u << 4) : 0u);  // DLC + IDE bit

    // RAM addresses are offset 0x400 from base
    uint16_t base = 0x400 + (uint16_t)(ua & 0xFFF);

    uint8_t buf[16];
    buf[0]  = t0 & 0xFF; buf[1]  = (t0 >> 8) & 0xFF;
    buf[2]  = (t0 >> 16) & 0xFF; buf[3]  = (t0 >> 24) & 0xFF;
    buf[4]  = t1 & 0xFF; buf[5]  = (t1 >> 8) & 0xFF;
    buf[6]  = (t1 >> 16) & 0xFF; buf[7]  = (t1 >> 24) & 0xFF;
    buf[8]  = ((uint8_t*)data)[0];
    buf[9]  = ((uint8_t*)data)[1];
    buf[10] = ((uint8_t*)data)[2];
    buf[11] = ((uint8_t*)data)[3];
    buf[12] = ((uint8_t*)data)[4];
    buf[13] = ((uint8_t*)data)[5];
    buf[14] = ((uint8_t*)data)[6];
    buf[15] = ((uint8_t*)data)[7];

    Mcp2518Fd_WriteSeq(base, buf, 16);

    // Set UINC and TXREQ together (bits 8 and 9 of CiTXQCON)
    uint32_t txqcon = Mcp2518Fd_ReadWord(MCP2518FD_C1TXQCON);
    Mcp2518Fd_WriteWord(MCP2518FD_C1TXQCON, txqcon | (1u << 8) | (1u << 9));
}

void Mcp251863Can::HandleInterrupt()
{
    // Loop while FIFO1 has data (TFNRFNIF = bit 0 of CiFIFOSTA1)
    for (int safety = 0; safety < 16; safety++)
    {
        uint32_t fsta = Mcp2518Fd_ReadWord(MCP2518FD_C1FIFOSTA1);
        if ((fsta & 1u) == 0)
            break;

        uint32_t ua = Mcp2518Fd_ReadWord(MCP2518FD_C1FIFOUA1);
        uint16_t base = 0x400 + (uint16_t)(ua & 0xFFF);

        uint8_t buf[16];
        Mcp2518Fd_ReadSeq(base, buf, 16);

        uint32_t t0 = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
        uint32_t t1 = buf[4] | (buf[5] << 8) | (buf[6] << 16) | (buf[7] << 24);

        bool extended = (t1 >> 4) & 1u;
        uint32_t canId;
        if (extended)
        {
            uint32_t sid = t0 & 0x7FFu;
            uint32_t eid = (t0 >> 11) & 0x3FFFFu;
            canId = (sid << 18) | eid;
        }
        else
        {
            canId = t0 & 0x7FFu;
        }
        uint8_t dlc = t1 & 0x0Fu;

        uint32_t data[2];
        data[0] = buf[8]  | (buf[9]  << 8) | (buf[10] << 16) | (buf[11] << 24);
        data[1] = buf[12] | (buf[13] << 8) | (buf[14] << 16) | (buf[15] << 24);

        HandleRx(canId, data, dlc);
        lastRxTimestamp = rtc_get_counter_val();

        // Advance read pointer (UINC bit 8 of CiFIFOCON1)
        uint32_t cfg = Mcp2518Fd_ReadWord(MCP2518FD_C1FIFOCON1);
        Mcp2518Fd_WriteWord(MCP2518FD_C1FIFOCON1, cfg | (1u << 8));
    }

    // Clear RXIF bits in CiRXIF (write 0 to clear)
    Mcp2518Fd_WriteWord(MCP2518FD_C1RXIF, 0);
}

void Mcp251863Can::ConfigureFilters()
{
    EnterMode(MCP2518FD_OPMOD_CONFIG);

    // Disable all 32 filters first by zeroing CiFLTCON0..7
    for (int i = 0; i < 8; i++)
        Mcp2518Fd_WriteWord(MCP2518FD_C1FLTCON0 + i * 4, 0);

    int maxFilters = 32;
    int count = nextUserMessageIndex < maxFilters ? nextUserMessageIndex : maxFilters;

    for (int i = 0; i < count; i++)
    {
        uint32_t id = userIds[i] & 0x1FFFFFFFu;
        bool extended = id > 0x7FFu;

        uint32_t fltobj = encodeId(id, extended) | (extended ? FLTOBJ_EXIDE : 0u);
        uint32_t mask;
        if (userMasks[i] != 0)
        {
            // RegisterUserMessage(id, mask) — user-supplied mask
            mask = encodeId(userMasks[i], extended);
        }
        else
        {
            // Exact match: full ID width
            mask = extended ? 0x1FFFFFFFu : 0x7FFu;
            mask = encodeId(mask, extended);
        }
        mask |= MASK_MIDE;  // require IDE bit to match filter's EXIDE

        Mcp2518Fd_WriteWord(MCP2518FD_C1FLTOBJ0 + i * 8, fltobj);
        Mcp2518Fd_WriteWord(MCP2518FD_C1MASK0   + i * 8, mask);

        // Enable filter i and point at FIFO 1
        uint16_t fltconAddr = MCP2518FD_C1FLTCON0 + (i / 4) * 4 + (i % 4);
        Mcp2518Fd_WriteByte(fltconAddr, FLTCON_ENABLE | 1);
    }

    // If more userIds than filters, the extras are silently dropped at the
    // hardware level. Software DecodeCAN won't see them. With 32 slots this
    // is very unlikely for this project.

    EnterMode(MCP2518FD_OPMOD_NORMAL_CAN2);
}

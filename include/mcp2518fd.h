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
#ifndef MCP2518FD_H
#define MCP2518FD_H

#include <stdint.h>

// SFR addresses (CAN1 module)
#define MCP2518FD_C1CON         0x000
#define MCP2518FD_C1NBTCFG      0x004
#define MCP2518FD_C1DBTCFG      0x008
#define MCP2518FD_C1INT         0x01C
#define MCP2518FD_C1RXIF        0x020
#define MCP2518FD_C1TXIF        0x024
#define MCP2518FD_C1TREC        0x034
#define MCP2518FD_C1TXQCON      0x050
#define MCP2518FD_C1TXQSTA      0x054
#define MCP2518FD_C1TXQUA       0x058
#define MCP2518FD_C1FIFOCON1    0x05C
#define MCP2518FD_C1FIFOSTA1    0x060
#define MCP2518FD_C1FIFOUA1     0x064
#define MCP2518FD_C1FLTCON0     0x1D0
#define MCP2518FD_C1FLTOBJ0     0x1F0
#define MCP2518FD_C1MASK0       0x1F4

// Standalone SFRs (top of address space)
#define MCP2518FD_OSC           0xE00
#define MCP2518FD_IOCON         0xE04
#define MCP2518FD_CRC           0xE08
#define MCP2518FD_DEVID         0xE14

// OSC register bits
#define OSC_OSCRDY              (1u << 10)
#define OSC_SCLKRDY             (1u << 12)

// CiCON OPMOD/REQOP values
#define MCP2518FD_OPMOD_NORMAL_FD       0
#define MCP2518FD_OPMOD_SLEEP           1
#define MCP2518FD_OPMOD_INT_LOOPBACK    2
#define MCP2518FD_OPMOD_LISTEN_ONLY     3
#define MCP2518FD_OPMOD_CONFIG          4
#define MCP2518FD_OPMOD_EXT_LOOPBACK    5
#define MCP2518FD_OPMOD_NORMAL_CAN2     6
#define MCP2518FD_OPMOD_RESTRICTED      7

// SPI primitives — all toggle DigIo::mcp4_cs around the transaction
void Mcp2518Fd_Reset(void);
uint8_t  Mcp2518Fd_ReadByte(uint16_t addr);
uint32_t Mcp2518Fd_ReadWord(uint16_t addr);
void     Mcp2518Fd_WriteByte(uint16_t addr, uint8_t v);
void     Mcp2518Fd_WriteWord(uint16_t addr, uint32_t v);
void     Mcp2518Fd_ReadSeq (uint16_t addr, uint8_t* buf, uint16_t len);
void     Mcp2518Fd_WriteSeq(uint16_t addr, const uint8_t* buf, uint16_t len);

#endif // MCP2518FD_H

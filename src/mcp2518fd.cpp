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
#include "mcp2518fd.h"
#include "digio.h"
#include <libopencm3/stm32/spi.h>

#define SPI_BUS         SPI2
#define CS_LOW()        DigIo::mcp4_cs.Clear()
#define CS_HIGH()       DigIo::mcp4_cs.Set()

// MCP2518FD SPI opcodes (high nibble of 16-bit command)
#define OP_RESET        0x0
#define OP_WRITE        0x2
#define OP_READ         0x3

static inline void sendCmd(uint8_t op, uint16_t addr)
{
   spi_xfer(SPI_BUS, (op << 4) | ((addr >> 8) & 0x0F));
   spi_xfer(SPI_BUS, addr & 0xFF);
}

void Mcp2518Fd_Reset(void)
{
   CS_LOW();
   spi_xfer(SPI_BUS, 0x00);
   spi_xfer(SPI_BUS, 0x00);
   CS_HIGH();
}

uint8_t Mcp2518Fd_ReadByte(uint16_t addr)
{
   uint8_t v;
   CS_LOW();
   sendCmd(OP_READ, addr);
   v = spi_xfer(SPI_BUS, 0x00);
   CS_HIGH();
   return v;
}

uint32_t Mcp2518Fd_ReadWord(uint16_t addr)
{
   uint32_t v = 0;
   CS_LOW();
   sendCmd(OP_READ, addr);
   v  = (uint32_t)spi_xfer(SPI_BUS, 0x00);
   v |= (uint32_t)spi_xfer(SPI_BUS, 0x00) << 8;
   v |= (uint32_t)spi_xfer(SPI_BUS, 0x00) << 16;
   v |= (uint32_t)spi_xfer(SPI_BUS, 0x00) << 24;
   CS_HIGH();
   return v;
}

void Mcp2518Fd_WriteByte(uint16_t addr, uint8_t v)
{
   CS_LOW();
   sendCmd(OP_WRITE, addr);
   spi_xfer(SPI_BUS, v);
   CS_HIGH();
}

void Mcp2518Fd_WriteWord(uint16_t addr, uint32_t v)
{
   CS_LOW();
   sendCmd(OP_WRITE, addr);
   spi_xfer(SPI_BUS, v & 0xFF);
   spi_xfer(SPI_BUS, (v >> 8) & 0xFF);
   spi_xfer(SPI_BUS, (v >> 16) & 0xFF);
   spi_xfer(SPI_BUS, (v >> 24) & 0xFF);
   CS_HIGH();
}

void Mcp2518Fd_ReadSeq(uint16_t addr, uint8_t* buf, uint16_t len)
{
   CS_LOW();
   sendCmd(OP_READ, addr);
   for (uint16_t i = 0; i < len; i++)
      buf[i] = spi_xfer(SPI_BUS, 0x00);
   CS_HIGH();
}

void Mcp2518Fd_WriteSeq(uint16_t addr, const uint8_t* buf, uint16_t len)
{
   CS_LOW();
   sendCmd(OP_WRITE, addr);
   for (uint16_t i = 0; i < len; i++)
      spi_xfer(SPI_BUS, buf[i]);
   CS_HIGH();
}

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
#ifndef MCP25625CAN_H
#define MCP25625CAN_H

#include "canhardware.h"

class Mcp25625Can : public CanHardware
{
public:
    Mcp25625Can(enum baudrates baudrate);
    void SetBaudrate(enum baudrates baudrate) override;
    void Send(uint32_t canId, uint32_t data[2], uint8_t len) override;

    // Called from EXTI15 ISR — drains all buffered RX messages
    void HandleInterrupt();

    static Mcp25625Can* GetInstance() { return instance; }

private:
    void ConfigureFilters() override;

    static Mcp25625Can* instance;
};

#endif // MCP25625CAN_H

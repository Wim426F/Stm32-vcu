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
#include "mcp25625can.h"
#include "MCP2515.h"
#include "CANSPI.h"
#include <libopencm3/stm32/rtc.h>

// External helpers from CANSPI.cpp
extern void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg);
extern uint32_t convertReg2ExtendedCANid(uint8_t eidh, uint8_t eidl, uint8_t sidh, uint8_t sidl);
extern uint32_t convertReg2StandardCANid(uint8_t sidh, uint8_t sidl);

Mcp25625Can* Mcp25625Can::instance = 0;

Mcp25625Can::Mcp25625Can(enum baudrates baudrate)
{
    instance = this;

    MCP2515_Initialize();
    MCP2515_SetTo_ConfigMode();

    SetBaudrate(baudrate);
    ConfigureFilters();

    // Enable RX0/RX1 IRQ + wake IRQ
    MCP2515_Bit_Modify(MCP2515_CANINTF, 0x43, 0x00);
    MCP2515_Bit_Modify(MCP2515_CANINTE, 0x43, 0x43);

    MCP2515_SetTo_NormalMode();
}

void Mcp25625Can::SetBaudrate(enum baudrates baudrate)
{
    MCP2515_SetTo_ConfigMode();

    // CNF values assume 16 MHz crystal on the MCP25625
    switch (baudrate)
    {
    case Baud33:
        MCP2515_Write_Byte(MCP2515_CNF1, 0x4E);
        MCP2515_Write_Byte(MCP2515_CNF2, 0xE5);
        MCP2515_Write_Byte(MCP2515_CNF3, 0x83);
        break;
    case Baud500:
        MCP2515_Write_Byte(MCP2515_CNF1, 0x40);
        MCP2515_Write_Byte(MCP2515_CNF2, 0xE5);
        MCP2515_Write_Byte(MCP2515_CNF3, 0x83);
        break;
    default:
        // Unsupported on this chip — leave timings untouched
        break;
    }

    MCP2515_SetTo_NormalMode();
}

void Mcp25625Can::Send(uint32_t canId, uint32_t data[2], uint8_t len)
{
    id_reg_t idReg = { 0, 0, 0, 0 };
    uint8_t idType = (canId > 0x7FF) ? dEXTENDED_CAN_MSG_ID_2_0B : dSTANDARD_CAN_MSG_ID_2_0B;

    convertCANid2Reg(canId & 0x1FFFFFFF, idType, &idReg);

    ctrl_status_t st = {};
    st.ctrl_status = MCP2515_Read_Status();

    uint8_t loadInst;
    uint8_t rtsInst;
    if (st.ctrl.TXB0REQ == 0)      { loadInst = MCP2515_LOAD_TXB0SIDH; rtsInst = MCP2515_RTS_TX0; }
    else if (st.ctrl.TXB1REQ == 0) { loadInst = MCP2515_LOAD_TXB1SIDH; rtsInst = MCP2515_RTS_TX1; }
    else if (st.ctrl.TXB2REQ == 0) { loadInst = MCP2515_LOAD_TXB2SIDH; rtsInst = MCP2515_RTS_TX2; }
    else                           { return; } // All TX buffers busy, drop frame

    MCP2515_Load_TxSequence(loadInst, &idReg.tempSIDH, len, (uint8_t*)data);
    MCP2515_RequestToSend(rtsInst);
}

void Mcp25625Can::HandleInterrupt()
{
    ctrl_rx_status_t rxStatus;
    rxStatus.ctrl_rx_status = MCP2515_Get_RxStatus();

    while (rxStatus.ctrlRx.rxBuffer != 0)
    {
        rx_reg_t rxReg;
        uint8_t readInst;

        if (rxStatus.ctrlRx.rxBuffer == MSG_IN_RXB1)
            readInst = MCP2515_READ_RXB1SIDH;
        else
            readInst = MCP2515_READ_RXB0SIDH; // RXB0 or both — drain RXB0 first

        MCP2515_Read_RxbSequence(readInst, sizeof(rxReg.rx_reg_array), rxReg.rx_reg_array);

        uint32_t canId;
        if (rxStatus.ctrlRx.msgType == dEXTENDED_CAN_MSG_ID_2_0B)
            canId = convertReg2ExtendedCANid(rxReg.RxReg.RXBnEID8, rxReg.RxReg.RXBnEID0,
                                             rxReg.RxReg.RXBnSIDH, rxReg.RxReg.RXBnSIDL);
        else
            canId = convertReg2StandardCANid(rxReg.RxReg.RXBnSIDH, rxReg.RxReg.RXBnSIDL);

        uint8_t dlc = rxReg.RxReg.RXBnDLC & 0x0F;
        uint32_t data[2];
        data[0] = rxReg.RxReg.RXBnD0
               | (rxReg.RxReg.RXBnD1 << 8)
               | (rxReg.RxReg.RXBnD2 << 16)
               | (rxReg.RxReg.RXBnD3 << 24);
        data[1] = rxReg.RxReg.RXBnD4
               | (rxReg.RxReg.RXBnD5 << 8)
               | (rxReg.RxReg.RXBnD6 << 16)
               | (rxReg.RxReg.RXBnD7 << 24);

        HandleRx(canId, data, dlc);
        lastRxTimestamp = rtc_get_counter_val();

        // Clear the IF flag we just serviced and re-poll for more buffered frames
        MCP2515_Bit_Modify(MCP2515_CANINTF, 0x03, 0x00);
        rxStatus.ctrl_rx_status = MCP2515_Get_RxStatus();
    }
}

// Pack an 11-bit standard ID into SIDH/SIDL byte pair (filter format)
static void packStandardFilter(uint16_t id, uint8_t* sidh, uint8_t* sidl,
                               uint8_t* eid8, uint8_t* eid0)
{
    *sidh = (id >> 3) & 0xFF;
    *sidl = (id & 0x07) << 5;
    *eid8 = 0;
    *eid0 = 0;
}

void Mcp25625Can::ConfigureFilters()
{
    MCP2515_SetTo_ConfigMode();

    // Count standard and extended IDs. The chip only has 6 filters total and
    // each filter is tied to either standard-or-extended via its SIDL.EXIDE bit
    // (and to a buffer's mask). When the load exceeds what we can express
    // exactly, fall back to permissive masks and rely on software filtering
    // inside the registered subsystem callbacks.
    int totalIds = nextUserMessageIndex;
    bool anyMask = false;
    bool anyExtended = false;
    for (int i = 0; i < totalIds; i++)
    {
        if (userMasks[i] != 0) anyMask = true;
        if (userIds[i] > 0x7FF) anyExtended = true;
    }

    uint8_t filterBytes[6][4] = { {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
                                  {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
    uint8_t mask0[4] = { 0, 0, 0, 0 };       // accept-all by default
    uint8_t mask1[4] = { 0, 0, 0, 0 };

    if (totalIds == 0 || totalIds > 6 || anyMask || anyExtended)
    {
        // Permissive: both buffers accept everything, software filters in callbacks.
        // (Same effective behavior as the legacy CANSPI_Initialize.)
    }
    else
    {
        // Tight: exact-match filters with mask 0x7FF on both buffers.
        mask0[0] = 0xFF; mask0[1] = 0xE0;
        mask1[0] = 0xFF; mask1[1] = 0xE0;

        for (int i = 0; i < totalIds; i++)
        {
            packStandardFilter(userIds[i] & 0x7FF,
                               &filterBytes[i][0], &filterBytes[i][1],
                               &filterBytes[i][2], &filterBytes[i][3]);
        }
    }

    // Write masks
    MCP2515_Write_ByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0, mask0);
    MCP2515_Write_ByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0, mask1);

    // Write all 6 filters (zeroed when unused, harmless under accept-all mask)
    MCP2515_Write_ByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0, filterBytes[0]);
    MCP2515_Write_ByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0, filterBytes[1]);
    MCP2515_Write_ByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0, filterBytes[2]);
    MCP2515_Write_ByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0, filterBytes[3]);
    MCP2515_Write_ByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0, filterBytes[4]);
    MCP2515_Write_ByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0, filterBytes[5]);

    MCP2515_SetTo_NormalMode();
}

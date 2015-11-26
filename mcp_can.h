/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2014-1-16
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"
#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
    private:
    uint8   m_nExtFlg;                                                  /* identifier xxxID             */
                                                                        /* either extended (the 29 LSB) */
                                                                        /* or standard (the 11 LSB)     */
    uint32  m_nID;                                                      /* can id                       */
    uint8   m_nDlc;                                                     /* data length:                 */
    uint8   m_nDta[MAX_CHAR_IN_MESSAGE];                            	/* data                         */
    uint8   m_nRtr;                                                     /* rtr                          */
    uint8   m_nfilhit;
    uint8   SPICS;

   private:

    void mcp2515_reset(void);                                           /* reset mcp2515                */

    uint8 mcp2515_readRegister(const uint8 address);                    /* read mcp2515's register      */
    
    void mcp2515_readRegisterS(const uint8 address, 
	                       uint8 values[], 
                               const uint8 n);
    void mcp2515_setRegister(const uint8 address,                       /* set mcp2515's register       */
                             const uint8 value);

    void mcp2515_setRegisterS(const uint8 address,                      /* set mcp2515's registers      */
                              const uint8 values[],
                              const uint8 n);
    
    void mcp2515_initCANBuffers(void);
    
    void mcp2515_modifyRegister(const uint8 address,                    /* set bit of one register      */
                                const uint8 mask,
                                const uint8 data);

    uint8 mcp2515_readStatus(void);                                     /* read mcp2515's Status        */
    uint8 mcp2515_setCANCTRL_Mode(const uint8 newmode);                 /* set mode                     */
    uint8 mcp2515_configRate(const uint8 canSpeed);                     /* set boadrate                 */
    uint8 mcp2515_init(const uint8 canSpeed);                           /* mcp2515init                  */

    void mcp2515_write_id( const uint8 mcp_addr,                        /* write can id                 */
                               const uint8 ext,
                               const uint32 id );

    void mcp2515_read_id( const uint8 mcp_addr,                        /* read can id                  */
                                    uint8* ext,
                                    uint32* id );

    void mcp2515_write_canMsg( const uint8 buffer_sidh_addr );          /* write can msg                */
    void mcp2515_read_canMsg( const uint8 buffer_sidh_addr);            /* read can msg                 */
    void mcp2515_start_transmit(const uint8 mcp_addr);                  /* start transmit               */
    uint8 mcp2515_getNextFreeTXBuf(uint8 *txbuf_n);                     /* get Next free txbuf          */

    uint8 setMsg(uint32 id, uint8 ext, uint8 len, uint8 *pData);    /* set message                  */  
    uint8 clearMsg();                                               /* clear all message to zero    */
    uint8 readMsg();                                                /* read message                 */
    uint8 sendMsg();                                                /* send message                 */

public:
    MCP_CAN(uint8 _CS);
    uint8 begin(uint8 speedset);                              /* init can                     */
    uint8 init_Mask(uint8 num, uint8 ext, uint32 ulData);           /* init Masks                   */
    uint8 init_Filt(uint8 num, uint8 ext, uint32 ulData);           /* init filters                 */
    uint8 sendMsgBuf(uint32 id, uint8 ext, uint8 len, uint8 *buf);  /* send buf                     */
    uint8 readMsgBuf(uint8 *len, uint8 *buf);                       /* read buf                     */
    uint8 checkReceive(void);                                       /* if something received        */
    uint8 checkError(void);                                         /* if something error           */
    uint32 getCanId(void);                                          /* get can id when receive      */
};

extern MCP_CAN CAN;
#endif

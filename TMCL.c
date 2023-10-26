/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file TMCL.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: TMCL.c
 *         Description: This file contains the TMCL interpreter for the
 *                      Homebus Master.
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#include <stdlib.h>
#include "max32660.h"
#include "SysTick.h"
#include "TMCL.h"
#include "RS485.h"
#include "HomebusMaster.h"
#include "Homebus.h"

#define RS485_MODULE_ADDRESS 1
#define RS485_HOST_ADDRESS   2

extern const char VersionString[];
static uint8_t TMCLCommandState;              //!< State of the interpreter
static TTMCLCommand ActualCommand;            //!< TMCL command to be executed (with all parameters)
static TTMCLReply ActualReply;                //!< Reply of last executed TMCL command
static uint8_t TMCLReplyFormat;               //!< format of next reply (RF_NORMAL or RF_SPECIAL)
static uint8_t SpecialReply[9];               //!< buffer for special replies

static void GetVersion(void);


/***************************************************************//**
   \fn SendTMCLCommandHomebus()
   \brief Send TMCL command via Homebus

   Send the TMCL command via Homebus and wait for the answer from
   the Homebus slave. The command must have been written
   to the global variable "ActualCommand" before.
********************************************************************/
static void SendTMCLCommandHomebus()
{
  uint8_t i;
  uint8_t HomebusData[9];
  uint32_t Timeout;
  uint8_t RxFlag;

  HomebusData[0]=RS485_MODULE_ADDRESS;
  HomebusData[1]=ActualCommand.Opcode;
  HomebusData[2]=ActualCommand.Type;
  HomebusData[3]=ActualCommand.Motor;
  HomebusData[4]=ActualCommand.Value.Byte[3];
  HomebusData[5]=ActualCommand.Value.Byte[2];
  HomebusData[6]=ActualCommand.Value.Byte[1];
  HomebusData[7]=ActualCommand.Value.Byte[0];
  HomebusData[8]=0;
  for(i=0; i<8; i++) HomebusData[8]+=HomebusData[i];
  HomebusSendData(HomebusData);

  Timeout=GetSysTimer();
  do
  {
    RxFlag=HomebusGetData(HomebusData);
  } while(!RxFlag && abs(GetSysTimer()-Timeout)<100);

  if(RxFlag)
  {
    ActualReply.Status=HomebusData[2];
    ActualReply.Opcode=HomebusData[3];
    ActualReply.Value.Byte[3]=HomebusData[4];
    ActualReply.Value.Byte[2]=HomebusData[5];
    ActualReply.Value.Byte[1]=HomebusData[6];
    ActualReply.Value.Byte[0]=HomebusData[7];
  }
  else
  {
    ActualReply.Status=REPLY_CMD_NOT_AVAILABLE;
  }
}

/***************************************************************//**
   \fn ExecuteActualCommand()
   \brief Execute actual TMCL command

   Execute the TMCL command that must have been written
   to the global variable "ActualCommand" before.
********************************************************************/
static void ExecuteActualCommand(void)
{
  //Prepare answer
  ActualReply.Opcode=ActualCommand.Opcode;
  ActualReply.Status=REPLY_OK;
  ActualReply.Value.Int32=ActualCommand.Value.Int32;

  //Execute the command
  switch(ActualCommand.Opcode)
  {
    case TMCL_ROR:
    case TMCL_ROL:
    case TMCL_MST:
    case TMCL_MVP:
    case TMCL_SAP:
    case TMCL_GAP:
    case TMCL_GIO:
    case TMCL_RFS:
      SendTMCLCommandHomebus();
      break;

    case TMCL_GetVersion:
      GetVersion();
      break;

    default:
      ActualReply.Status=REPLY_INVALID_CMD;
      break;
  }
}


/***************************************************************//**
   \fn ProcessCommand(void)
   \brief Fetch and execute TMCL commands

   This is the main function for fetching and executing TMCL commands
   and has to be called periodically from the main loop.
********************************************************************/
void ProcessCommand(void)
{
  uint8_t RS485Cmd[9];
  uint8_t RS485Reply[9];
  uint8_t Checksum;
  uint32_t i;

  //**Send answer for the last command**
  if(TMCLCommandState==TCS_UART)  //via UART
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      RS485Reply[8]=RS485_HOST_ADDRESS+RS485_MODULE_ADDRESS+
                    ActualReply.Status+ActualReply.Opcode+
                    ActualReply.Value.Byte[3]+
                    ActualReply.Value.Byte[2]+
                    ActualReply.Value.Byte[1]+
                    ActualReply.Value.Byte[0];

      RS485Reply[0]=RS485_HOST_ADDRESS;
      RS485Reply[1]=RS485_MODULE_ADDRESS;
      RS485Reply[2]=ActualReply.Status;
      RS485Reply[3]=ActualReply.Opcode;
      RS485Reply[4]=ActualReply.Value.Byte[3];
      RS485Reply[5]=ActualReply.Value.Byte[2];
      RS485Reply[6]=ActualReply.Value.Byte[1];
      RS485Reply[7]=ActualReply.Value.Byte[0];
      SendRS485Reply(RS485Reply);
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      SendRS485Reply(SpecialReply);
    }
  }
  else if(TMCLCommandState==TCS_UART_ERROR)  //check sum of the last command has been wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    RS485Reply[8]=RS485_HOST_ADDRESS+RS485_MODULE_ADDRESS+
                  ActualReply.Status+ActualReply.Opcode+
                  ActualReply.Value.Byte[3]+
                  ActualReply.Value.Byte[2]+
                  ActualReply.Value.Byte[1]+
                  ActualReply.Value.Byte[0];

    RS485Reply[0]=RS485_HOST_ADDRESS;
    RS485Reply[1]=RS485_MODULE_ADDRESS;
    RS485Reply[2]=ActualReply.Status;
    RS485Reply[3]=ActualReply.Opcode;
    RS485Reply[4]=ActualReply.Value.Byte[3];
    RS485Reply[5]=ActualReply.Value.Byte[2];
    RS485Reply[6]=ActualReply.Value.Byte[1];
    RS485Reply[7]=ActualReply.Value.Byte[0];
    SendRS485Reply(RS485Reply);
  }

  //Reset state (answer has been sent now)
  TMCLCommandState=TCS_IDLE;
  TMCLReplyFormat=RF_STANDARD;

  //**Try to get a new command**
  if(GetRS485Command(RS485Cmd))  //Get data from UART
  {
    if(RS485Cmd[0]==RS485_MODULE_ADDRESS)  //Is this our addresss?
    {
      Checksum=0;
      for(i=0; i<8; i++) Checksum+=RS485Cmd[i];

      if(Checksum==RS485Cmd[8])  //Is the checksum correct?
      {
        ActualCommand.Opcode=RS485Cmd[1];
        ActualCommand.Type=RS485Cmd[2];
        ActualCommand.Motor=RS485Cmd[3];
        ActualCommand.Value.Byte[3]=RS485Cmd[4];
        ActualCommand.Value.Byte[2]=RS485Cmd[5];
        ActualCommand.Value.Byte[1]=RS485Cmd[6];
        ActualCommand.Value.Byte[0]=RS485Cmd[7];

        TMCLCommandState=TCS_UART;
      }
      else TMCLCommandState=TCS_UART_ERROR;  //Checksum wrong
    }
  }

  //**Execute the command**
  //Check if a command could be fetched and execute it.
  if(TMCLCommandState!=TCS_IDLE && TMCLCommandState!=TCS_UART_ERROR) ExecuteActualCommand();
}

/***************************************************************//**
  \fn GetVersion(void)
  \brief Command 136 (get version)

  Get the version number (when type==0) or
  the version string (when type==1).
********************************************************************/
static void GetVersion(void)
{
  uint32_t i;

  switch(ActualCommand.Type)
  {
    case 0:
      TMCLReplyFormat=RF_SPECIAL;
      SpecialReply[0]=RS485_HOST_ADDRESS;
      for(i=0; i<8; i++)
        SpecialReply[i+1]=VersionString[i];
      break;

    case 1:
      ActualReply.Value.Byte[3]=SW_TYPE_HIGH;
      ActualReply.Value.Byte[2]=SW_TYPE_LOW;
      ActualReply.Value.Byte[1]=SW_VERSION_HIGH;
      ActualReply.Value.Byte[0]=SW_VERSION_LOW;
      break;

    case 4:
      ActualCommand.Type=1;
      SendTMCLCommandHomebus();
      break;

    default:
      ActualReply.Status=REPLY_WRONG_TYPE;
      break;
  }
}

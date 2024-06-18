/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file HomebusMaster.c ******************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: HomebusMaster.c
 *         Description: Main program for the Homebus Master
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#include <stdlib.h>
#include "max32660.h"
#include "gpio.h"
#include "SysTick.h"
#include "RS485.h"
#include "TMCL.h"
#include "HomebusMaster.h"
#include "Homebus.h"

const char VersionString[]="0025V100";  //!< Version information for the TMCL-IDE

static gpio_cfg_t led_out;     //!< Pin for LED (P0.13)
static uint32_t Delay;         //!< Delay for LED blinking

/***************************************************************//**
   \fn InitIO()
   \brief I/O intialization

   Initialize I/O ports.
********************************************************************/
void InitIO(void)
{
  led_out.port = PORT_0;
  led_out.mask = PIN_13;
  led_out.pad = GPIO_PAD_NONE;
  led_out.func = GPIO_FUNC_OUT;
  GPIO_Config(&led_out);
}

//Main program
void main()
{
  InitSysTick();
  InitIO();
  InitRS485(9600);
  HomebusInit(230400);

  for(;;)
  {
    if(abs(GetSysTimer()-Delay)>1000)
    {
      GPIO_OutToggle(&led_out);
      Delay=GetSysTimer();
    }

    ProcessCommand();
  }
}

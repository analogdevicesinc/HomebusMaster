/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file RS485.h ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: RS485.h
 *         Description: RS485 interface functions
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#ifndef __RS485_H
#define __RS485_H

void InitRS485(uint32_t Baudrate);
void SendRS485Reply(uint8_t *data);
uint8_t GetRS485Command(uint8_t *data);

#endif

/*
  This file is part of the ArduinoBLE library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "utility/ATT.h"
#include "utility/HCI.h"
#include "utility/GAP.h"
#include "utility/GATT.h"
#include "utility/L2CAPSignaling.h"

#include "BTClassicLocalDevice.h"

#if defined(ARDUINO_PORTENTA_H7_M4) || defined(ARDUINO_PORTENTA_H7_M7)
#ifndef BT_REG_ON
#define BT_REG_ON PJ_12
#endif
#endif

BTClassicLocalDevice::BTClassicLocalDevice()
{
}

BTClassicLocalDevice::~BTClassicLocalDevice()
{
}

int BTClassicLocalDevice::begin()
{
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_AVR_UNO_WIFI_REV2) || defined(ARDUINO_SAMD_NANO_33_IOT)
  // reset the NINA in BLE mode
  pinMode(SPIWIFI_SS, OUTPUT);
  pinMode(NINA_RESETN, OUTPUT);
  
  digitalWrite(SPIWIFI_SS, LOW);
#endif

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  digitalWrite(NINA_RESETN, HIGH);
  delay(100);
  digitalWrite(NINA_RESETN, LOW);
  delay(750);
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
  // inverted reset
  digitalWrite(NINA_RESETN, LOW);
  delay(100);
  digitalWrite(NINA_RESETN, HIGH);
  delay(750);
#elif defined(ARDUINO_PORTENTA_H7_M4) || defined(ARDUINO_PORTENTA_H7_M7)
  // BT_REG_ON -> HIGH
  pinMode(BT_REG_ON, OUTPUT);
  digitalWrite(BT_REG_ON, HIGH);
#endif


#ifdef ARDUINO_AVR_UNO_WIFI_REV2
  // set SS HIGH
  digitalWrite(SPIWIFI_SS, HIGH);

  // set RTS HIGH
  pinMode(NINA_RTS, OUTPUT);
  digitalWrite(NINA_RTS, HIGH);

  // set CTS as input
  pinMode(NINA_CTS, INPUT);
#endif

  if (!HCI.begin()) {
    end();
    return 0;
  }

  delay(100);

  if (HCI.reset() != 0) {
    end();

    return 0;
  }

  uint8_t hciVer;
  uint16_t hciRev;
  uint8_t lmpVer;
  uint16_t manufacturer;
  uint16_t lmpSubVer;

  if (HCI.readLocalVersion(hciVer, hciRev, lmpVer, manufacturer, lmpSubVer) != 0) {
    end();
    return 0;
  }

  if (HCI.setEventMask(0x3FFFFFFFFFFFFFFF) != 0) {
    end();
    return 0;
  }

  HCI.writeLocalName("PortentaH7");

  uint8_t locAddr[6];
  HCI.readBdAddr(locAddr);

  HCI.writeInquiryScanActivity(0x0050,0x0012);

  HCI.writeScanEnable(0x01); // Enabling inquiry scan

  return 1;
}

void BTClassicLocalDevice::end()
{

  HCI.end();

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  // disable the NINA
  digitalWrite(NINA_RESETN, HIGH);
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
  // disable the NINA
  digitalWrite(NINA_RESETN, LOW);
#elif defined(ARDUINO_PORTENTA_H7_M4) || defined(ARDUINO_PORTENTA_H7_M7)
  // BT_REG_ON -> LOW
  digitalWrite(BT_REG_ON, LOW);
#endif
}

int BTClassicLocalDevice::setLocalName(char *localName)
{
    return HCI.writeLocalName(localName);
}

int enableScans(uint8_t scanEnable)
{
    return HCI.writeScanEnable(scanEnable);
}

void BTClassicLocalDevice::debug(Stream& stream)
{
  HCI.debug(stream);
}

void BTClassicLocalDevice::noDebug()
{
  HCI.noDebug();
}

BTClassicLocalDevice BTClassic;
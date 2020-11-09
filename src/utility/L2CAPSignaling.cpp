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

#include "HCI.h"

#include "L2CAPSignaling.h"

// BT Classic L2CAP Signalling Commands
#define L2CAP_CMD_COMMAND_REJECT        0x01
#define L2CAP_CMD_CONNECTION_REQUEST    0x02
#define L2CAP_CMD_CONNECTION_RESPONSE   0x03
#define L2CAP_CMD_CONFIG_REQUEST        0x04
#define L2CAP_CMD_CONFIG_RESPONSE       0x05
#define L2CAP_CMD_DISCONNECT_REQUEST    0x06
#define L2CAP_CMD_DISCONNECT_RESPONSE   0x07
#define L2CAP_CMD_INFORMATION_REQUEST   0x0A
#define L2CAP_CMD_INFORMATION_RESPONSE  0x0B

/* Bluetooth L2CAP PSM - see http://www.bluetooth.org/Technical/AssignedNumbers/logical_link.htm */
#define SDP_PSM         0x01 // Service Discovery Protocol PSM Value
#define RFCOMM_PSM      0x03 // RFCOMM PSM Value

// LE L2CAP Signalling Commands
#define CONNECTION_PARAMETER_UPDATE_REQUEST  0x12
#define CONNECTION_PARAMETER_UPDATE_RESPONSE 0x13

L2CAPSignalingClass::L2CAPSignalingClass() :
  _minInterval(0),
  _maxInterval(0)
{
}

L2CAPSignalingClass::~L2CAPSignalingClass()
{
}

void L2CAPSignalingClass::addConnection(uint16_t handle, uint8_t role, uint8_t /*peerBdaddrType*/,
                                        uint8_t /*peerBdaddr*/[6], uint16_t interval,
                                        uint16_t /*latency*/, uint16_t /*supervisionTimeout*/,
                                        uint8_t /*masterClockAccuracy*/)
{
  if (role != 1) {
    // ignore
    return;
  }

  if (!_minInterval || !_maxInterval) {
    // no connection intervale to request
    return;
  }

  if (interval >= _minInterval && interval <= _maxInterval) {
    // all good, within interval range
    return;
  }

  struct __attribute__ ((packed)) L2CAPConnectionParameterUpdateRequest {
    uint8_t code;
    uint8_t identifier;
    uint16_t length;
    uint16_t minInterval;
    uint16_t maxInterval;
    uint16_t latency;
    uint16_t supervisionTimeout;
  } request = { CONNECTION_PARAMETER_UPDATE_REQUEST, 0x01, 8,
                _minInterval, _maxInterval, 0x0000, 0x00c8 };

  HCI.sendAclPkt(handle, SIGNALING_CID, sizeof(request), &request);
}

void L2CAPSignalingClass::handleData(uint16_t connectionHandle, uint8_t dlen, uint8_t data[])
{
  struct __attribute__ ((packed)) L2CAPSignalingHdr {
    uint8_t code;
    uint8_t identifier;
    uint16_t length;
  } *l2capSignalingHdr = (L2CAPSignalingHdr*)data;

  if (dlen < sizeof(L2CAPSignalingHdr)) {
    // too short, ignore
    return;
  }

  if (dlen != (sizeof(L2CAPSignalingHdr) + l2capSignalingHdr->length)) {
    // invalid length, ignore
    return;
  }

  uint8_t code = l2capSignalingHdr->code;
  uint8_t identifier = l2capSignalingHdr->identifier;
  uint16_t length = l2capSignalingHdr->length;
  data = &data[sizeof(L2CAPSignalingHdr)];

  if (code == CONNECTION_PARAMETER_UPDATE_REQUEST) {
    connectionParameterUpdateRequest(connectionHandle, identifier, length, data);
  } else if (code == CONNECTION_PARAMETER_UPDATE_RESPONSE) {
    connectionParameterUpdateResponse(connectionHandle, identifier, length, data);
  } else if (code == L2CAP_CMD_INFORMATION_REQUEST) {
    uint16_t infoType = (uint16_t)data[0] + ((uint16_t)data[1])<<8;
    informationResponse(connectionHandle, identifier, infoType);
  } else if (code == L2CAP_CMD_CONNECTION_REQUEST) {
    connectionRequestReply(connectionHandle, identifier, length, data);
  }
   
}

void L2CAPSignalingClass::informationResponse(uint16_t handle, uint8_t identifier, uint16_t infoType)
{
  uint16_t replyLen;
  struct __attribute__ ((packed)) L2CAPInfoResponse {
    uint8_t code;
    uint8_t identifier;
    uint16_t length;
    uint16_t infoType;
    uint16_t result;
    uint8_t infoArray[8];
  } response;
  response.code = L2CAP_CMD_INFORMATION_RESPONSE;
  response.identifier = identifier;
  response.length = 8;
  response.infoType = infoType;
  response.result = 0x0; // Success
  memset(response.infoArray,0,8);
  
  if(infoType == 0x0002) // Connectionless MTU
  {
    replyLen = sizeof(response) - 8;
    response.result = 0x01; // Not supported
  } else if(infoType == 0x0002) // Extended Features Infor
  {
    replyLen = sizeof(response) - 4;
    response.infoArray[0] = 0x80; // Indicating support for fixed channels       
  }
  else if(infoType == 0x0003) // Supported Fixed Channels
  {
    response.length = 12;
    response.infoArray[0] = 2; // Indicates only L2CAP signalling channel is supportes
    replyLen = sizeof(response);
  }
  
  HCI.sendAclPkt(handle, BTCLASSIC_SIGNALING_CID, replyLen, &response);
}

void L2CAPSignalingClass::connectionRequestReply(uint16_t handle, uint8_t identifier, uint8_t dlen, uint8_t data[])
{
  struct __attribute__ ((packed)) L2CAPConnRequest {
    uint16_t PSM;
    uint16_t sourceCID;
  } *request = (L2CAPConnRequest*)data;

  if(request->PSM == SDP_PSM) // Service discovery protocol
  {

  }
  uint16_t replyLen;
  struct __attribute__ ((packed)) L2CAPConnRequestReply {
    uint8_t code;
    uint8_t identifier;
    uint16_t length;
    uint16_t destCID;
    uint16_t srcCID;
    uint16_t result;
    uint16_t status;
  } response;
  response.code = L2CAP_CMD_CONNECTION_RESPONSE;
  response.identifier = identifier;
  response.length = 8;
  response.destCID = 0x0050;
  response.srcCID = request->sourceCID;
  response.result = 0x0000; // Success
  response.status = 0;
  
  HCI.sendAclPkt(handle, BTCLASSIC_SIGNALING_CID, sizeof(response), &response);
}

void L2CAPSignalingClass::removeConnection(uint8_t /*handle*/, uint16_t /*reason*/)
{
}

void L2CAPSignalingClass::setConnectionInterval(uint16_t minInterval, uint16_t maxInterval)
{
  _minInterval = minInterval;
  _maxInterval = maxInterval;
}

void L2CAPSignalingClass::connectionParameterUpdateRequest(uint16_t handle, uint8_t identifier, uint8_t dlen, uint8_t data[])
{
  struct __attribute__ ((packed)) L2CAPConnectionParameterUpdateRequest {
    uint16_t minInterval;
    uint16_t maxInterval;
    uint16_t latency;
    uint16_t supervisionTimeout;
  } *request = (L2CAPConnectionParameterUpdateRequest*)data;

  if (dlen < sizeof(L2CAPConnectionParameterUpdateRequest)) {
    // too short, ignore
    return;
  }

  struct __attribute__ ((packed)) L2CAPConnectionParameterUpdateResponse {
    uint8_t code;
    uint8_t identifier;
    uint16_t length;
    uint16_t value;
  } response = { CONNECTION_PARAMETER_UPDATE_RESPONSE, identifier, 2, 0x0000 };

  if (_minInterval && _maxInterval) {
    if (request->minInterval < _minInterval || request->maxInterval > _maxInterval) {
      response.value = 0x0001; // reject
    }
  }

  HCI.sendAclPkt(handle, SIGNALING_CID, sizeof(response), &response);

  if (response.value == 0x0000) {
    HCI.leConnUpdate(handle, request->minInterval, request->maxInterval, request->latency, request->supervisionTimeout);
  }
}

void L2CAPSignalingClass::connectionParameterUpdateResponse(uint16_t /*handle*/, uint8_t /*identifier*/, uint8_t /*dlen*/, uint8_t /*data*/[])
{
}

L2CAPSignalingClass L2CAPSignaling;

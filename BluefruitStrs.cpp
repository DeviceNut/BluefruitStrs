/*
This file is part of the BluefruitStrs library.

BluefruitStrs is free software : you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

BluefruitStrs is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License. If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <SPI.h>
#include "BluefruitStrs.h"

#define DEBUG_OUTPUT 0 // 1 to debug this file
#if DEBUG_OUTPUT
#define DBG(x) x
#define DBGSTR(x) (char*)x
#else
#define DBG(x)
#define DBGSTR(x) (char*)""
#endif

#define SPI_CS_ENABLE()           digitalWrite(m_cs_pin, LOW)
#define SPI_CS_DISABLE()          digitalWrite(m_cs_pin, HIGH)
#define SPI_DATA_READY()          (digitalRead(m_irq_pin) != 0)

#define SPI_IGNORED_BYTE          254   // SPI default character, clocked out in case of an ignored transaction
#define SPI_IGNORED_BYTE2         255   // not documented, but also must be ignored

#define SPI_DEFAULT_DELAY_US      1500  // in microseconds (without delays it gets packet errors)
#define BLE_DEFAULT_TIMEOUT_MS    3000  // allows for the hardware to catch up when sending a lot of data

#define SDEP_MAX_PACKETSIZE       16    // Maximum payload per packet

#define ATTR_PACKED __attribute__ ((packed))

typedef enum
{
  SDEP_CMDTYPE_INITIALIZE     = 0xBEEF,   /** Controls the on board LED(s) */
  SDEP_CMDTYPE_AT_WRAPPER     = 0x0A00,
  SDEP_CMDTYPE_BLE_UARTTX     = 0x0A01,
  SDEP_CMDTYPE_BLE_UARTRX     = 0x0A02,
}
sdepCmdType_t;

typedef enum
{
  SDEP_MSGTYPE_COMMAND          = 0x10,
  SDEP_MSGTYPE_RESPONSE         = 0x20,
  SDEP_MSGTYPE_ALERT            = 0x40,
  SDEP_MSGTYPE_ERROR            = 0x80
}
sdepMsgType_t;

enum packetRetCode
{
  PACKET_CODE_SUCCESS = 0,
  PACKET_CODE_TIMEOUT,          // might happen if firmware busy? should cause retries?
  PACKET_CODE_OVERRUN,          // shouldn't happen if input buffer is large enough
  PACKET_CODE_ALERT,            // are alerts ever sent? (low battery, ...??)
  PACKET_CODE_ERRMSG,           // probably due to an invalid AT command...
  PACKET_CODE_BAD_MSGTYPE,      // can be caused by bad timing to some parts
  PACKET_CODE_BAD_CMDTYPE,      // these shouldn't happen normally, and
  PACKET_CODE_BAD_DATALEN,      //  might indicate bad wiring or part
};

typedef struct ATTR_PACKED
{
  byte msg_type;                // 8-bit message type indicator (sdepMsgType_t)

  union 
  {
    uint16_t cmd_id;            // 16-bit command ID
    struct
    {
      byte cmd_id_low;
      byte cmd_id_high;
    };
  };

  struct
  {
    byte length    : 7;         // Payload length (for this packet)
    byte more_data : 1;         // 'more' bit for multiple packet transfers
  };
}
sdepMsgHeader_t;

typedef struct ATTR_PACKED 
{
  sdepMsgHeader_t header;
  byte payload[SDEP_MAX_PACKETSIZE];
}
sdepMsgCommand_t;

typedef sdepMsgCommand_t sdepMsgResponse_t;

static byte m_cs_pin, m_irq_pin, m_rst_pin;
static char *m_buffer;
static uint16_t m_buflen;

static bool msgAlert = false;         // not implemented? or just never been able to cause?
static bool msgError = false;         // command not recognized? probably incorrect software
static bool deviceConnected = false;  // true if connection query indicates connection active
static packetRetCode packetCode;      // current error code
static sdepMsgResponse_t msgResponse; // saved response buffer (header.length is always valid)
static byte *responseBufPtr;          // saved pointer into response buffer
static NotifyCallback notifyCB;       // pointer to notification callback routine

static bool getReplyMsg(void);
static bool packetsToStr(char *strbuf, uint16_t buflen);
static bool packetRead(sdepMsgResponse_t *presponse);

static bool strToPackets(char *str, bool atcmd);
static bool packetWrite(uint16_t command, byte *buffer, byte count, byte more_data);

BluefruitStrs::BluefruitStrs(byte csPin, byte irqPin, byte rstPin)
{
  m_cs_pin  = csPin;
  m_irq_pin = irqPin;
  m_rst_pin = rstPin;

  pinMode(m_irq_pin, INPUT);

  // de-assert CS and RST by default
  pinMode(m_cs_pin, OUTPUT);
  digitalWrite(m_cs_pin, HIGH);

  pinMode(m_rst_pin, OUTPUT);
  digitalWrite(m_rst_pin, HIGH);
}

void BluefruitStrs::reset(void)
{
  // note that this command does NOT generate a response
  if (!packetWrite(SDEP_CMDTYPE_INITIALIZE, NULL,0,0))
  { DBG( Serial.println("Bluefruit initialize failed!"); ) }

  DBG( Serial.println("Bluefruit hardware reset..."); )

  // pull the hardware reset pin RST to GND for 10 ms
  digitalWrite(m_rst_pin, LOW);
  delay(10);
  digitalWrite(m_rst_pin, HIGH);

  // discard data in last read packet and global buffer
  msgResponse.header.length = 0;
  m_buffer[0] = 0;

  delay(1000); // Bluefruit takes 1 second to reboot

  DBG( Serial.println("..hardware reset done"); )
}

// must be called only once at setup() time
void BluefruitStrs::init(char *buffer, uint16_t buflen, NotifyCallback cb)
{
  SPI.begin(); // initialize SPI library

  m_buffer = buffer;
  m_buflen = buflen;
  notifyCB = cb;

  reset(); // must reset hardware to begin
}

static void notifyHandler(void)
{
  DBG( Serial.print("PacketCode: "); Serial.println(packetCode); )

  if (packetCode < PACKET_CODE_ALERT)
    notifyCB(NotifyMsg_SoftErr, DBGSTR("SoftErr"));
  else
  if (packetCode == PACKET_CODE_ALERT)
    notifyCB(NotifyMsg_Alert, m_buffer);
  else
  if (packetCode == PACKET_CODE_ERRMSG)
    notifyCB(NotifyMsg_ErrMsg, m_buffer);
  else
    notifyCB(NotifyMsg_PktErr, DBGSTR("PktErr"));

  m_buffer[0] = 0;
}

bool BluefruitStrs::writeDataStr(char *datastr)
{
  if (!strToPackets(datastr, false))
  {
    notifyHandler();
    return false;
  }
  return true;
}

static void CheckReturnValue(void)
{
  if (isdigit(m_buffer[0]))
  {
    int val = atoi(m_buffer);
    if (val > 0) deviceConnected = true;
  }
}
bool BluefruitStrs::isConnected(void)
{
  deviceConnected = false;
  if (!sendCmdStr((char*)"AT+GAPGETCONN", CheckReturnValue))
    return false;

  DBG( Serial.print("Connected="); Serial.println(deviceConnected); )
  return deviceConnected;
}

bool BluefruitStrs::sendCmdStr(char *cmdstr, ResponseCallback cb)
{
  if (!strToPackets(cmdstr, true))
  {
    notifyHandler();
    return false;
  }

  bool success = false;
  bool ismore = false;
  do
  {
    ismore = getReplyMsg();
    if (packetCode != PACKET_CODE_SUCCESS)
      return false;

    if (!strcmp(m_buffer, "OK")) success = true;

    if (cb != NULL) cb();
  }
  while (ismore);

  m_buffer[0] = 0;

  if (success) return true;

  if (packetCode == PACKET_CODE_SUCCESS)
    notifyCB(NotifyMsg_NoAck, DBGSTR("NoACK"));

  return false;
}

// Returns false if device communications is not working,
// or there is no response message available, else true.
bool BluefruitStrs::readDataStr(ResponseCallback cb)
{
  bool ismore = false;
  do
  {
    if (!ismore)
    {
      // query for BLE UART data, then read all data packets
      if (!packetWrite(SDEP_CMDTYPE_BLE_UARTRX, NULL,0,0))
        return false;
    }

    ismore = getReplyMsg();
    if (packetCode != PACKET_CODE_SUCCESS)
      return false;

    // return string to caller if not empty
    if (m_buffer[0] != 0) cb();
  }
  while (ismore);

  return true;
}

// Returns true if successful and there is more data to be retrieved.
// 'buffer' will have any retrieved string, or could be empty.
// If packetCode != PACKET_CODE_SUCCESS the handler has been called.
static bool getReplyMsg(void)
{
  bool ismore = packetsToStr(m_buffer, m_buflen);

  if (packetCode == PACKET_CODE_SUCCESS)
  {
    if (msgAlert)
    {
      // now convert to error after having retrieved message text
      packetCode = PACKET_CODE_ALERT;
      msgAlert = false;
    }

    if (msgError)
    {
      // now convert to error after having retrieved message text
      packetCode = PACKET_CODE_ERRMSG;
      msgError = false;
    }
  }

  if (packetCode != PACKET_CODE_SUCCESS)
  {
    notifyHandler();
    return false;
  }

  DBG( if (m_buffer[0]) { Serial.print("Message: \""); Serial.print(m_buffer); Serial.println("\""); } )
  //DBG( if (ismore) Serial.println("...ismore");  )

  return ismore;
}

// Forms strings from packets, delimited by a '\n' char (which is
// not returned in the buffer). Keeps state of a partially read
// packet as strings can be terminated in the middle of a packet.
//
// The input 'strbuff' is always terminated before returning.
// Returns true if more data is available to be read.
static bool packetsToStr(char *strbuf, uint16_t buflen)
{
  uint16_t bufpos = 0;
  strbuf[0] = 0;

  bool isdone = false;
  do
  {
    if (!msgResponse.header.length) // must read new data
    {
      if (!packetRead(&msgResponse))
        break;

      // reset position to the start of the payload
      responseBufPtr = (byte*)msgResponse.payload;

      #if DEBUG_OUTPUT
      if (msgResponse.header.length)
      {
        Serial.print("Bytes:");
        for (int i = 0; i < msgResponse.header.length; ++i)
        {
          Serial.print(" ");
          Serial.print(responseBufPtr[i], HEX);
        }
        Serial.println();
      }
      #endif
    }

    //DBG( else Serial.println("Read data from packet..."); )
    //DBG( if (!msgResponse.header.length) Serial.println("Read empty packet"); )

    while (msgResponse.header.length)
    {
      if (*responseBufPtr == '\r')
      {
        // remove from input:
        ++responseBufPtr;
        --msgResponse.header.length;
        continue; // ignore this char
      }
      else if (*responseBufPtr == '\n')
      {
        // remove from input:
        ++responseBufPtr;
        --msgResponse.header.length;

        isdone = true;
        break;
      }
      else if (buflen <= 1) // no more room in output
      {
        packetCode = PACKET_CODE_OVERRUN;
        isdone = true;
        break;
      }

      strbuf[bufpos++] = *responseBufPtr++;
      --msgResponse.header.length;
      --buflen;
    }
  }
  while (!isdone && msgResponse.header.more_data);

  strbuf[bufpos] = 0;

  return msgResponse.header.length || msgResponse.header.more_data;
}

static void spiReadBuff(byte *buff, int len)
{
  for (int i = 0; i < len; ++i)
    buff[i] = SPI.transfer(0xff);
}

// Returns false if there was any errors while reading a packet,
// in which case the global error code packetCode is set, and the
// device might have to be reset to be able to continue.
// Guaranteed to return within the timeout period.
static bool packetRead(sdepMsgResponse_t *presponse)
{
  memset(presponse, 0, sizeof(sdepMsgResponse_t));
  sdepMsgHeader_t *pheader = &presponse->header;

  packetCode = PACKET_CODE_TIMEOUT;
  uint32_t endtime = millis() + BLE_DEFAULT_TIMEOUT_MS;

  DBG( int rdycount = 0; )
  DBG( int spicount = 0; )
  do
  {
    if (SPI_DATA_READY()) // wait for data ready signal from device
    {
      DBG( ++rdycount; )

      //DBG( Serial.println("Data is ready..."); )
      SPI.beginTransaction( SPISettings(4000000, MSBFIRST, SPI_MODE0) );
      SPI_CS_ENABLE();

      // MUST have this delay or it blows up!
      delayMicroseconds(SPI_DEFAULT_DELAY_US);

      while (millis() < endtime)
      {
        pheader->msg_type = SPI.transfer(0xff);
        if ((pheader->msg_type != SPI_IGNORED_BYTE) &&
            (pheader->msg_type != SPI_IGNORED_BYTE2))
          break;

        SPI_CS_DISABLE();
        delayMicroseconds(SPI_DEFAULT_DELAY_US);
        SPI_CS_ENABLE();
        delayMicroseconds(SPI_DEFAULT_DELAY_US);

        DBG( ++spicount; )
      }

      while (millis() < endtime)
      {
        if (pheader->msg_type & SDEP_MSGTYPE_ALERT)
        {
          msgAlert = true;
          DBG( Serial.println("Alert!!..."); )
          // still get payload containing message text
        }
        else if (pheader->msg_type & SDEP_MSGTYPE_ERROR)
        {
          msgError = true;
          DBG( Serial.println("Error!!..."); )
          // still get payload containing message text
        }
        else if (!(pheader->msg_type & SDEP_MSGTYPE_RESPONSE))
        {
          DBG( Serial.print("Bad MsgType: 0x"); Serial.println(pheader->msg_type, HEX); )
          packetCode = PACKET_CODE_BAD_MSGTYPE;
          break;
        }

        // have read the first byte, now read the rest of the packet
        spiReadBuff((&pheader->msg_type)+1, sizeof(sdepMsgHeader_t)-1);

        if (pheader->length > SDEP_MAX_PACKETSIZE)
        {
          DBG( Serial.print("Bad DataLen: "); Serial.println(pheader->length); )
          packetCode = PACKET_CODE_BAD_DATALEN;
          break;
        }

        // command is 16-bits at odd address!! - must be aligned for 32-bit processors
        uint16_t cmd_id = word(pheader->cmd_id_high, pheader->cmd_id_low);
        if ((cmd_id != SDEP_CMDTYPE_AT_WRAPPER) &&
            (cmd_id != SDEP_CMDTYPE_BLE_UARTTX) &&
            (cmd_id != SDEP_CMDTYPE_BLE_UARTRX))
        {
          DBG( Serial.print("Bad CmdType: 0x"); Serial.println(pheader->cmd_id, HEX); )
          packetCode = PACKET_CODE_BAD_CMDTYPE;
          break;
        }

        spiReadBuff(presponse->payload, pheader->length);
        packetCode = PACKET_CODE_SUCCESS;
        break;
      }

      SPI_CS_DISABLE();
      SPI.endTransaction();
      break;
    }
  }
  while (millis() < endtime);

  DBG( if (rdycount != 1) { Serial.print("RdyCount="); Serial.println(rdycount); } )
  DBG( if (spicount != 0) { Serial.print("SpiCount="); Serial.println(spicount); } )

  return (packetCode == PACKET_CODE_SUCCESS);
}

// Writes one or more packets with data from the input string.
// The return conditions are the same as from calling packetWrite().
static bool strToPackets(char *str, bool atcmd)
{
  uint16_t slen = strlen(str);
  if (!slen) return true;

  uint16_t cmd = (atcmd ? SDEP_CMDTYPE_AT_WRAPPER : SDEP_CMDTYPE_BLE_UARTTX);

  DBG( Serial.print("strToPackets(");
        Serial.print(atcmd ? "Cmd" : "Data");
        Serial.print("): \"");
        if (str[0] != '\n') Serial.print(str);
        Serial.println("\""); )

  while (slen)
  {
    uint16_t count = min(slen, SDEP_MAX_PACKETSIZE);
    slen -= count;

    if (!packetWrite(cmd, (byte*)str, count, (slen ? 1 : 0)))
      return false;

    str += count;
  }

  return true;
}

// Returns false if write operation timed out, in which case the
// global error code 'packetCode' is set. This usually indicates
// a hardware failure or the pins are not set correctly.
// Guaranteed to return within the timeout period.
static bool packetWrite(uint16_t command, byte *buf, byte count, byte more_data)
{
  sdepMsgCommand_t cmdmsg;
  cmdmsg.header.msg_type    = SDEP_MSGTYPE_COMMAND;
  cmdmsg.header.cmd_id_high = highByte(command);
  cmdmsg.header.cmd_id_low  = lowByte(command);
  cmdmsg.header.length      = count;
  cmdmsg.header.more_data   = more_data;

  // copy data into the payload if there is any
  if ((buf != NULL) && (count > 0)) memcpy(cmdmsg.payload, buf, count);

  #if 0 //DEBUG_OUTPUT
  Serial.print("Payload:");
  for (int i = 0; i < cmdmsg.header.length; ++i)
  {
    Serial.print(" ");
    Serial.print(cmdmsg.payload[i], HEX);
  }
  Serial.println();
  #endif

  byte *buff = (byte*)&cmdmsg;

  SPI.beginTransaction( SPISettings(4000000, MSBFIRST, SPI_MODE0) );
  SPI_CS_ENABLE();

  packetCode = PACKET_CODE_TIMEOUT;
  uint32_t endtime = millis() + BLE_DEFAULT_TIMEOUT_MS;
  do
  {
    if (SPI.transfer(buff[0]) == SPI_IGNORED_BYTE)
    {
      SPI_CS_DISABLE();
      delayMicroseconds(SPI_DEFAULT_DELAY_US);
      SPI_CS_ENABLE();
    }
    else
    {
      for (size_t i = 1; i < (sizeof(sdepMsgHeader_t) + count); ++i)
        SPI.transfer(buff[i]);

      packetCode = PACKET_CODE_SUCCESS;
      break;
    }
  }
  while (millis() < endtime);

  SPI_CS_DISABLE();
  SPI.endTransaction();

  return (packetCode == PACKET_CODE_SUCCESS);
}

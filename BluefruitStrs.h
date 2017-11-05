#ifndef BLUEFRUIT_STRS_H
#define BLUEFRUIT_STRS_H

enum NotifyMessage
{
  NotifyMsg_Alert = 0,        // haven't ever seen this happen
  NotifyMsg_ErrMsg,           // or this...
  NotifyMsg_PktErr,           // bad packet data (hardware/timing issues)
  NotifyMsg_SoftErr,          // buggy software or power issue?
  NotifyMsg_NoAck,            // never received OK acknowledgement
  NotifyMsg_Count
};

typedef void (*NotifyCallback)(NotifyMessage msgval, char *msgstr);
typedef void (*ResponseCallback)(void); // response is in 'strbuf'

class BluefruitStrs
{
public:
  BluefruitStrs(byte csPin, byte irqPin, byte rstPin);

  // The 'strbuf' and 'buflen' parameters are reused by
  // some of the following routines for reading responses,
  // avoiding storing them in local data to conserve space.
  void init(char *strbuf, uint16_t buflen, NotifyCallback cb);

  // Performs a reset on the hardware device, which of course
  // will break any outstanding connection. Should be called
  // after a notify callback before trying to continue.
  // 'strbuf' from init() is cleared upon return.
  void reset(void);

  // Queries the connection status, returns true if active.
  // Uses 'strbuf' from init() to read responses.
  // Errors invoke the notify callback and returns false;
  // 'strbuf' from init() is cleared upon return.
  bool isConnected(void);

  // Sends an AT command to the device, with all responses
  // returned if a callback is specified, or ignored if NULL.
  // Stores each returned string into 'strbuf' from init().
  // Errors invoke the notify callback and returns false;
  // 'strbuf' from init() is cleared upon return.
  bool sendCmdStr(char *cmdstr, ResponseCallback cb);

  // Writes a string to the device (chars up to ending 0).
  // Any desired newline must be added by the caller.
  // Errors invoke the notify callback and returns false;
  // 'strbuf' from init() is cleared upon return.
  bool writeDataStr(char *datastr);

  // Retrieves all strings (delimited by '\n', which is
  // not returned), and invokes the callback on each one.
  // Stores each returned string into 'strbuf' from init().
  // Errors invoke the notify callback and returns false;
  // 'strbuf' from init() is NOT cleared upon return.
  bool readDataStr(ResponseCallback cb);
};

#endif

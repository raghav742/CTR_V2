#pragma once
// ============================================================
//  Serial output for the protocol
//  Every module prints through "Out" instead of "Serial".
//  Out behaves exactly like Serial.print/println, plus:
//   - while a command with a request id ("#12 pos") runs, every
//     line it prints starts with "#12 ", so a laptop program can
//     match replies to its requests;
//   - room() tells how much fits in the send buffer without
//     waiting (used by the status stream to skip, not block);
//   - decimals (print(x, 2)) are printed about 5x faster than
//     Arduino's own float printing (0.5 ms per number), so long
//     lines with positions no longer slow the main loop.
// ============================================================
#include <Arduino.h>

class SerialOut : public Print {
public:
  size_t write(uint8_t c) override;
  using Print::write;
  using Print::print;
  using Print::println;
  size_t print(double v, int digits = 2);      // fast fixed-point version
  size_t println(double v, int digits = 2);
  void beginReply(int32_t id);   // id < 0: no tag
  void endReply();
  int  room() { return Serial.availableForWrite(); }
private:
  bool    atLineStart = true;
  int32_t tag = -1;
};

extern SerialOut Out;

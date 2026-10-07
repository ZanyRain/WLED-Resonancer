#include "wled.h"

// A usermod may take over frames of a serial protocol the core only parses (see
// usermods/Resonancer); the core itself stays free of usermod types.
static SerialFrameSink serialFrameSink = nullptr;
void setSerialFrameSink(SerialFrameSink sink) { serialFrameSink = sink; }

// forward declarations
static void sendBytes();

/*
 * Adalight and TPM2 handler
 */

enum class AdaState {
  Header_A,
  Header_d,
  Header_a,
  Header_CountHi,
  Header_CountLo,
  Header_CountCheck,
  Data_Red,
  Data_Green,
  Data_Blue,
  TPM2_Header_Type,
  TPM2_Header_CountHi,
  TPM2_Header_CountLo,
  LumaFlow_Head2,
  LumaFlow_Length,
  LumaFlow_Cmd,
  LumaFlow_Data,
  LumaFlow_Checksum,
  LumaFlow_Tail
};

static uint16_t currentBaud = 1152; //default baudrate 115200 (divided by 100)
static bool continuousSendLED = false;
static uint32_t lastUpdate = 0;

void updateBaudRate(uint32_t rate){
  unsigned rate100 = rate/100;
  if (rate100 == currentBaud || rate100 < 96) return;
  currentBaud = rate100;

  if (serialCanTX){
    Serial.print(F("Baud is now ")); Serial.println(rate);
  }

  Serial.flush();
  Serial.begin(rate);
}

// RGB LED data return as JSON array. Slow, but easy to use on the other end.
static inline void sendJSON(){
  if (serialCanTX) {
    unsigned used = strip.getLengthTotal();
    Serial.write('[');
    for (unsigned i=0; i<used; i++) {
      Serial.print(strip.getPixelColor(i));
      if (i != used-1) Serial.write(',');
    }
    Serial.println("]");
  }
}

// RGB LED data returned as bytes in TPM2 format. Faster, and slightly less easy to use on the other end.
static void sendBytes(){
  if (serialCanTX) {
    Serial.write(0xC9); Serial.write(0xDA);
    unsigned used = strip.getLengthTotal();
    unsigned len = used*3;
    Serial.write(highByte(len));
    Serial.write(lowByte(len));
    for (unsigned i=0; i < used; i++) {
      uint32_t c = strip.getPixelColor(i);
      Serial.write(qadd8(W(c), R(c))); //R, add white channel to RGB channels as a simple RGBW -> RGB map
      Serial.write(qadd8(W(c), G(c))); //G
      Serial.write(qadd8(W(c), B(c))); //B
    }
    Serial.write(0x36); Serial.write('\n');
  }
}

void handleSerial()
{
  if (!(serialCanRX && Serial)) return; // arduino docs: `if (Serial)` indicates whether or not the USB CDC serial connection is open. For all non-USB CDC ports, this will always return true

  static auto state = AdaState::Header_A;
  static uint16_t count = 0;
  static uint16_t pixel = 0;
  static byte check = 0x00;
  static byte red   = 0x00;
  static byte green = 0x00;
  static byte blue  = 0x00;
  static uint8_t lf_length = 0;
  static uint8_t lf_cmd = 0;
  static uint8_t lf_checksum = 0;
  static uint8_t lf_idx = 0;
  static uint8_t lf_buf[128];

  while (Serial.available() > 0)
  {
    yield();
    byte next = Serial.peek();
    switch (state) {
      case AdaState::Header_A:
        if      (next == 'A')  { state = AdaState::Header_d; }
        else if (next == 0xC9) { state = AdaState::TPM2_Header_Type; } //TPM2 start byte
        else if (next == 'I')  { handleImprovPacket(); return; }
        else if (next == 'v')  { Serial.print("WLED"); Serial.write(' '); Serial.println(VERSION); }
        else if (next == 0xB0) { updateBaudRate( 115200); }
        else if (next == 0xB1) { updateBaudRate( 230400); }
        else if (next == 0xB2) { updateBaudRate( 460800); }
        else if (next == 0xB3) { updateBaudRate( 500000); }
        else if (next == 0xB4) { updateBaudRate( 576000); }
        else if (next == 0xB5) { updateBaudRate( 921600); }
        else if (next == 0xB6) { updateBaudRate(1000000); }
        else if (next == 0xB7) { updateBaudRate(1500000); }
        else if (next == 0xEB) { state = AdaState::LumaFlow_Head2; }
        else if (next == 'l')  { sendJSON(); } // Send LED data as JSON Array
        else if (next == 'L')  { sendBytes(); } // Send LED data as TPM2 Data Packet
        else if (next == 'o')  { continuousSendLED = false; } // Disable Continuous Serial Streaming
        else if (next == 'O')  { continuousSendLED = true; } // Enable Continuous Serial Streaming
        else if (next == '{')  { //JSON API
          bool verboseResponse = false;
          if (!requestJSONBufferLock(JSON_LOCK_SERIAL)) {
            Serial.printf_P(PSTR("{\"error\":%d}\n"), ERR_NOBUF);
            return;
          }
          Serial.setTimeout(100);
          DeserializationError error = deserializeJson(*pDoc, Serial);
          if (!error) {
            verboseResponse = deserializeState(pDoc->as<JsonObject>());
            //only send response if TX pin is unused for other purposes
            if (verboseResponse && serialCanTX) {
              pDoc->clear();
              JsonObject stateDoc = pDoc->createNestedObject("state");
              serializeState(stateDoc);
              JsonObject info  = pDoc->createNestedObject("info");
              serializeInfo(info);

              serializeJson(*pDoc, Serial);
              Serial.println();
            }
          }
          releaseJSONBufferLock();
        }
        break;
      case AdaState::Header_d:
        if (next == 'd') state = AdaState::Header_a;
        else             state = AdaState::Header_A;
        break;
      case AdaState::Header_a:
        if (next == 'a') state = AdaState::Header_CountHi;
        else             state = AdaState::Header_A;
        break;
      case AdaState::Header_CountHi:
        pixel = 0;
        count = next * 0x100;
        check = next;
        state = AdaState::Header_CountLo;
        break;
      case AdaState::Header_CountLo:
        count += next + 1;
        check = check ^ next ^ 0x55;
        state = AdaState::Header_CountCheck;
        break;
      case AdaState::Header_CountCheck:
        if (check == next) state = AdaState::Data_Red;
        else               state = AdaState::Header_A;
        break;
      case AdaState::TPM2_Header_Type:
        state = AdaState::Header_A; //(unsupported) TPM2 command or invalid type
        if (next == 0xDA) state = AdaState::TPM2_Header_CountHi; //TPM2 data
        else if (next == 0xAA) Serial.write(0xAC); //TPM2 ping
        break;
      case AdaState::TPM2_Header_CountHi:
        pixel = 0;
        count = (next * 0x100) /3;
        state = AdaState::TPM2_Header_CountLo;
        break;
      case AdaState::TPM2_Header_CountLo:
        count += next /3;
        state = AdaState::Data_Red;
        break;
      case AdaState::Data_Red:
        red   = next;
        state = AdaState::Data_Green;
        break;
      case AdaState::Data_Green:
        green = next;
        state = AdaState::Data_Blue;
        break;
      case AdaState::Data_Blue:
        blue  = next;
        if (!realtimeOverride) setRealtimePixel(pixel++, red, green, blue, 0);
        if (--count > 0) state = AdaState::Data_Red;
        else {
          realtimeLock(realtimeTimeoutMs, REALTIME_MODE_ADALIGHT);

          if (!realtimeOverride) strip.show();
          state = AdaState::Header_A;
        }
        break;
      case AdaState::LumaFlow_Head2:
        if (next == 0x90) {
            state = AdaState::LumaFlow_Length;
        } else {
            state = AdaState::Header_A;
        }
        break;
      case AdaState::LumaFlow_Length:
        if (next > 0 && next <= 128) {
            lf_length = next;
            lf_checksum = next;
            lf_idx = 0;
            state = AdaState::LumaFlow_Cmd;
        } else {
            state = AdaState::Header_A;
        }
        break;
      case AdaState::LumaFlow_Cmd:
        lf_cmd = next;
        lf_checksum += next;
        state = (lf_length > 1) ? AdaState::LumaFlow_Data : AdaState::LumaFlow_Checksum;
        break;
      case AdaState::LumaFlow_Data:
        lf_buf[lf_idx++] = next;
        lf_checksum += next;
        if (lf_idx == lf_length - 1) {
            state = AdaState::LumaFlow_Checksum;
        }
        break;
      case AdaState::LumaFlow_Checksum:
        if ((lf_checksum & 0xFF) == next) {
            state = AdaState::LumaFlow_Tail;
        } else {
            state = AdaState::Header_A; // checksum mismatch, drop the frame
        }
        break;
      case AdaState::LumaFlow_Tail:
        // Hand the binary payload to a registered sink. The frame is already
        // validated (header, length, checksum), so no JSON or hex conversion is
        // needed on this path.
        if (next == 0xED && serialFrameSink != nullptr) {
          serialFrameSink(lf_cmd, lf_buf, lf_idx);
        }
        state = AdaState::Header_A; // a frame is complete either way
    }

    // All other received bytes will disable Continuous Serial Streaming
    if (continuousSendLED && next != 'O'){
      continuousSendLED = false;
    }

    Serial.read(); //discard the byte
  }

  // If Continuous Serial Streaming is enabled, send new LED data as bytes
  if (continuousSendLED && (lastUpdate != strip.getLastShow())){
    sendBytes();
    lastUpdate = strip.getLastShow();
  }
}

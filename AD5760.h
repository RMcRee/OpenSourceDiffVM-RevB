#ifndef AD5760_H
#define AD5760_H

#include <Arduino.h>
#include <SPI.h>

class AD5760 {
private:
  // Register addresses
  static constexpr uint8_t REG_NOP       = 0x0;
  static constexpr uint8_t REG_DAC       = 0x1;
  static constexpr uint8_t REG_CTRL      = 0x2;
  static constexpr uint8_t REG_CLEARCODE = 0x3;
  static constexpr uint8_t REG_SWCTRL    = 0x4;

  // Control register low bits
  static constexpr uint16_t CTRL_RBUF    = (1u << 1);
  static constexpr uint16_t CTRL_OPGND   = (1u << 2);
  static constexpr uint16_t CTRL_DACTRI  = (1u << 3);
  static constexpr uint16_t CTRL_BIN_2SC = (1u << 4);
  static constexpr uint16_t CTRL_SDODIS  = (1u << 5);

  // Software control bits
  static constexpr uint8_t SW_LDAC  = (1u << 0);
  static constexpr uint8_t SW_CLR   = (1u << 1);
  static constexpr uint8_t SW_RESET = (1u << 2);

  uint8_t _csPin;
  uint8_t _clrPin;
  uint8_t _resetPin;

  SPIClass &_spi;
  SPISettings _spiSettings;

  float _vrefp;
  float _vrefn;

  bool    _bitbang = false;
  bool    _bitbangShared = false;  // if true, switch pins to GPIO per-transfer then restore to LPSPI4
  uint8_t _mosiPin = 0;
  uint8_t _misoPin = 0;
  uint8_t _sckPin  = 0;
public:
  // Constructor
  // csPin   = SYNC pin on AD5760
  // no ldacPin, held low
  // clrPin  = CLR pin on AD5760
  // resetPin= RESET pin on AD5760
  // spi     = SPI bus to use (default SPI)
  AD5760(uint8_t csPin,
         uint8_t clrPin,
         uint8_t resetPin,
         SPIClass &spi = SPI)
    : _csPin(csPin),
      _clrPin(clrPin),
      _resetPin(resetPin),
      _spi(spi),
      _spiSettings(1000000, MSBFIRST, SPI_MODE1),
      _vrefp(5.0f),
      _vrefn(0.0f)  {  }

  // Switch to bit-bang mode: bypasses hardware SPI entirely, uses plain GPIO on
  // the same physical pins. Call after begin(). Useful when the SPI peripheral
  // is suspected damaged. MODE1: SCK idles LOW, MOSI set during HIGH phase,
  // slave (AD5760) samples MOSI on falling edge, drives MISO after rising edge.
  void enableBitbang(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin) {
    _bitbang = true;
    _mosiPin = mosiPin;
    _misoPin = misoPin;
    _sckPin  = sckPin;
    pinMode(_mosiPin, OUTPUT);
    pinMode(_misoPin, INPUT);
    pinMode(_sckPin,  OUTPUT);
    digitalWrite(_mosiPin, LOW);
    digitalWrite(_sckPin,  LOW);  // SCK idles LOW for MODE1
  }

  // Like enableBitbang() but for a shared SPI bus (e.g. shared with an ADC).
  // Pins stay in their normal LPSPI4 (ALT3) mux function between transfers.
  // On each transfer the driver switches them to GPIO, bit-bangs, then restores
  // them to LPSPI4 so the other device is unaffected.
  // Call after begin(). Pins mosiPin/misoPin/sckPin must be the hardware SPI pins.
  void enableBitbangShared(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin) {
    _bitbang       = true;
    _bitbangShared = true;
    _mosiPin = mosiPin;
    _misoPin = misoPin;
    _sckPin  = sckPin;
    // Pins remain in LPSPI4 (ALT3) mux mode until needed
  }

  // Call once from setup().
  // spiClockHz can be much higher, but 1 MHz is a conservative starting point.
  void begin(uint32_t spiClockHz = 1000000, float vrefp = 5.0f, float vrefn = 0.0f) {
    _vrefp = vrefp;
    _vrefn = vrefn;
    _spiSettings = SPISettings(spiClockHz, MSBFIRST, SPI_MODE1);

    pinMode(_csPin, OUTPUT);
    pinMode(_clrPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);

    digitalWrite(_csPin, HIGH);     // SYNC inactive high
    digitalWrite(_clrPin, HIGH);    // inactive high
    digitalWrite(_resetPin, HIGH);  // inactive high

    _spi.begin();
  }

  // Hardware reset pulse.
  void hardwareReset() {
    digitalWrite(_resetPin, LOW);
    delayMicroseconds(1);
    digitalWrite(_resetPin, HIGH);
    delayMicroseconds(10);
  }

  // RBUF=0, BIN_2SC=0: gain-of-two mode, two's complement. Requires external op-amp on VOUT.
  void configureGain2TwosComplement(bool disableSDO = false) {
    uint16_t ctrl = 0;
    if (disableSDO) ctrl |= CTRL_SDODIS;
    writeControlRegister(ctrl);
  }

  // RBUF=1, BIN_2SC=0: unity-gain mode, two's complement. Internal buffer drives VOUT directly.
  // Output spans 0V to VREFP (unipolar). No external op-amp needed. Good for initial bring-up.
  void configureUnityGainTwosComplement(bool disableSDO = false) {
    uint16_t ctrl = CTRL_RBUF;
    if (disableSDO) ctrl |= CTRL_SDODIS;
    writeControlRegister(ctrl);
  }

  // Write raw signed 16-bit two's-complement DAC code.
  void writeCode(int16_t code) {
    writeRegister16(REG_DAC, static_cast<uint16_t>(code));
  }

  // Trigger software LDAC.
  void softwareLDAC() {
    writeSoftwareControl(SW_LDAC);
  }

  // Write code, then update output using software LDAC.
  void writeCodeAndUpdate(int16_t code) {
    writeCode(code);
    //softwareLDAC(); LDAC pin tied low
  }

  // Set clearcode register.
  void setClearCode(int16_t code) {
    writeRegister16(REG_CLEARCODE, static_cast<uint16_t>(code));
  }

  // Software clear and software reset.
  void softwareClear() {
    writeSoftwareControl(SW_CLR);
  }

  void softwareReset() {
    writeSoftwareControl(SW_RESET);
  }

  // Convert desired output voltage to signed 16-bit code for gain-of-two mode.
  // General gain-of-two span:
  //   Vout_min = 2*VREFN - VREFP
  //   Vout_max = VREFP
  //
  // Common case with VREFN = 0, VREFP = 5V:
  //   Vout span ~ -5V to +5V
  int16_t voltageToCode(float volts) const {
    const float span = _vrefp - _vrefn;
    const float minV = 2.0f * _vrefn - _vrefp;
    const float maxV = _vrefp;

    float v = volts;
    if (v < minV) v = minV;
    if (v > maxV) v = maxV;

    float codeF = ((v - _vrefn) / span) * 32768.0f;

    if (codeF > 32767.0f)  codeF = 32767.0f;
    if (codeF < -32768.0f) codeF = -32768.0f;

    return static_cast<int16_t>(lroundf(codeF));
  }

  void setVoltageAndUpdate(float volts) {
    int16_t code = voltageToCode(volts);
    writeCodeAndUpdate(code);
  }


  void writeControlRegister(uint16_t ctrl) {
    writeFrame(0, REG_CTRL, static_cast<uint32_t>(ctrl) & 0xFFFFF);
  }

  void writeSoftwareControl(uint8_t sw) {
    writeFrame(0, REG_SWCTRL, static_cast<uint32_t>(sw) & 0x7);
  }

  // For DAC and clearcode registers, the 16-bit value goes into DB19..DB4.
  void writeRegister16(uint8_t reg, uint16_t value) {
    uint32_t data20 = static_cast<uint32_t>(value) << 4;
    writeFrame(0, reg, data20);
  }

  // --- Bit-bang helper ---
  // Transfer one byte, MODE1: MOSI driven before rising SCK, MISO sampled
  // during high phase (slave drives after rising edge per AD5760 t19).
  // ~100 kHz (5 µs half-period).
  uint8_t bbXfer(uint8_t out) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
      digitalWrite(_mosiPin, (out >> i) & 1);
      digitalWrite(_sckPin, HIGH);
      delayMicroseconds(5);
      in = (in << 1) | digitalRead(_misoPin);
      digitalWrite(_sckPin, LOW);
      delayMicroseconds(5);
    }
    return in;
  }

  // --- Teensy 4.x direct LPSPI4 helpers ---
  // Root cause of SPI hang: beginTransaction() writes TCR while LPSPI is disabled (MEN=0),
  // which NXP RT1060 silently drops. When MEN goes high the TX FIFO has a TDR but no
  // preceding command entry, so LPSPI never generates SCLK. Fix: push TCR + TDR as
  // back-to-back stores while MEN=1, same as Teensyduino's transfer16() does internally.

  // Write byte, discard received byte. TCR+TDR pushed back-to-back (no instruction between).
  // Caller must have called SPI.beginTransaction() and flushed FIFOs first.
  static void dacXferByte(uint8_t data) {
    // MODE1 8-bit command: FRAMESZ=7, CPHA=1. Both stores hit the AHB write buffer
    // together before LPSPI can dequeue either, avoiding the lone-TDR stall.
    static constexpr uint32_t kTcr = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CPHA;
    IMXRT_LPSPI4_S.TCR = kTcr;
    IMXRT_LPSPI4_S.TDR = data;
    uint32_t deadline = ARM_DWT_CYCCNT + 600000UL;  // 1 ms at 600 MHz
    while (!((IMXRT_LPSPI4_S.FSR >> 16) & 0x1F)) {
      if ((int32_t)(ARM_DWT_CYCCNT - deadline) >= 0) {
        Serial.print("[AD5760 TIMEOUT dacXferByte=0x"); Serial.print(data, HEX);
        Serial.print(" CR=0x");  Serial.print(IMXRT_LPSPI4_S.CR,  HEX);
        Serial.print(" FSR=0x"); Serial.print(IMXRT_LPSPI4_S.FSR, HEX);
        Serial.print(" TCR=0x"); Serial.println(IMXRT_LPSPI4_S.TCR, HEX);
        while (true) {}
      }
    }
    (void)IMXRT_LPSPI4_S.RDR;
  }

  // Write byte, return received byte (for readback). TCR+TDR pushed back-to-back,
  // same fix as dacXferByte — avoids TCR-drop when MEN was 0 during beginTransaction().
  static uint8_t dacXferByteAndRead(uint8_t data) {
    static constexpr uint32_t kTcr = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CPHA;
    IMXRT_LPSPI4_S.TCR = kTcr;
    IMXRT_LPSPI4_S.TDR = data;
    uint32_t deadline = ARM_DWT_CYCCNT + 600000UL;
    while (IMXRT_LPSPI4_S.RSR & LPSPI_RSR_RXEMPTY) {
      if ((int32_t)(ARM_DWT_CYCCNT - deadline) >= 0) {
        Serial.print("[AD5760 TIMEOUT dacXferByteAndRead=0x"); Serial.println(data, HEX);
        while (true) {}
      }
    }
    return IMXRT_LPSPI4_S.RDR;
  }

  // Hardware CLR pulse: loads DAC register from clearcode register, updates output.
  // NOTE: softwareClear() has NO effect when LDAC is hardwired LOW (datasheet Table 14, fn.1).
  // This hardware method works regardless of LDAC state.
  void hardwareClear() {
    digitalWrite(_clrPin, LOW);
    delayMicroseconds(1);
    digitalWrite(_clrPin, HIGH);
    delayMicroseconds(10);
  }

  // Read back a register. Sends a read-request frame (R/W=1), then a NOP frame while
  // capturing the device's SDO output. Returns the raw 24-bit response.
  // Requires SDO connected to MISO (Teensy default SPI: pin 12).
  uint32_t readBack(uint8_t reg) {
    writeFrame(1, reg, 0);  // read-request frame

    // NOP frame: device clocks out the requested register contents on SDO.
    uint8_t d0, d1, d2;
    if (_bitbang) {
      if (_bitbangShared) {
        pinMode(_mosiPin, OUTPUT);
        pinMode(_misoPin, INPUT);
        pinMode(_sckPin,  OUTPUT);
        digitalWriteFast(_sckPin, LOW);
      }
      digitalWrite(_csPin, LOW);
      d0 = bbXfer(0x00);
      d1 = bbXfer(0x00);
      d2 = bbXfer(0x00);
      digitalWrite(_csPin, HIGH);
      if (_bitbangShared) {
        *(portConfigRegister(_mosiPin)) = 3 | 0x10;
        *(portConfigRegister(_sckPin))  = 3 | 0x10;
        *(portConfigRegister(_misoPin)) = 3 | 0x10;
      }
    } else {
      _spi.beginTransaction(_spiSettings);
      IMXRT_LPSPI4_S.CR |= LPSPI_CR_RTF | LPSPI_CR_RRF;  // flush FIFOs
      digitalWrite(_csPin, LOW);
      d0 = dacXferByteAndRead(0x00);
      d1 = dacXferByteAndRead(0x00);
      d2 = dacXferByteAndRead(0x00);
      digitalWrite(_csPin, HIGH);
      _spi.endTransaction();
    }

    uint32_t response = ((uint32_t)d0 << 16) | ((uint32_t)d1 << 8) | d2;

#ifdef AD5760_DEBUG_SPI
    Serial.print("[AD5760] readBack REG="); Serial.print(reg);
    static const char* rNames[] = {"NOP","DAC","CTRL","CLEARCODE","SWCTRL"};
    if (reg < 5) { Serial.print("("); Serial.print(rNames[reg]); Serial.print(")"); }
    Serial.print(" -> 0x");
    if (response < 0x100000) Serial.print("0");
    Serial.print(response, HEX);
    Serial.print("  ");
    Serial.print((response >> 23) & 1);
    Serial.print('|');
    for (int i = 22; i >= 20; i--) Serial.print((response >> i) & 1);
    Serial.print('|');
    for (int i = 19; i >= 4; i--) Serial.print((response >> i) & 1);
    Serial.print('|');
    for (int i = 3; i >= 0; i--) Serial.print((response >> i) & 1);
    if (reg == REG_DAC || reg == REG_CLEARCODE) {
      int16_t code16 = static_cast<int16_t>((response >> 4) & 0xFFFF);
      Serial.print("  code="); Serial.print(code16);
    }
    Serial.println();
#endif
    return response;
  }

  // Read back the signed 16-bit code currently in the DAC register (DB19:DB4).
  int16_t readDacCode() {
    return static_cast<int16_t>((readBack(REG_DAC) >> 4) & 0xFFFF);
  }

  // Read back the signed 16-bit code currently in the clearcode register (DB19:DB4).
  int16_t readClearCode() {
    return static_cast<int16_t>((readBack(REG_CLEARCODE) >> 4) & 0xFFFF);
  }

  // Read back the control register (bits DB19:DB0 of the response frame).
  uint32_t readControlReg() {
    return readBack(REG_CTRL) & 0xFFFFF;
  }

  void writeFrame(uint8_t rw, uint8_t reg, uint32_t data20) {
    uint32_t frame = 0;
    frame |= (static_cast<uint32_t>(rw & 0x1) << 23);
    frame |= (static_cast<uint32_t>(reg & 0x7) << 20);
    frame |= (data20 & 0xFFFFF);

    uint8_t b0 = (frame >> 16) & 0xFF;
    uint8_t b1 = (frame >> 8) & 0xFF;
    uint8_t b2 = frame & 0xFF;

#ifdef AD5760_DEBUG_SPI
    // Hex
    Serial.print("[AD5760] 0x");
    if (frame < 0x100000) Serial.print("0");
    Serial.print(frame, HEX);
    Serial.print("  ");
    // Binary with field separators: RW | REG(3) | DATA[19:4](16) | [3:0]
    Serial.print((frame >> 23) & 1);
    Serial.print('|');
    for (int i = 22; i >= 20; i--) Serial.print((frame >> i) & 1);
    Serial.print('|');
    for (int i = 19; i >= 4; i--) Serial.print((frame >> i) & 1);
    Serial.print('|');
    for (int i = 3; i >= 0; i--) Serial.print((frame >> i) & 1);
    // Decoded fields
    Serial.print("  RW="); Serial.print(rw);
    Serial.print(" REG="); Serial.print(reg);
    static const char* regNames[] = {"NOP","DAC","CTRL","CLEARCODE","SWCTRL"};
    if (reg < 5) { Serial.print("("); Serial.print(regNames[reg]); Serial.print(")"); }
    Serial.print(" DATA=0x"); Serial.print(data20 & 0xFFFFF, HEX);
    // For write operations on DAC/clearcode registers, show the decoded 16-bit signed code
    if (rw == 0 && (reg == REG_DAC || reg == REG_CLEARCODE)) {
      int16_t code = static_cast<int16_t>((data20 >> 4) & 0xFFFF);
      Serial.print(" code="); Serial.print(code);
    }
    Serial.println();
#endif

    if (_bitbang) {
      if (_bitbangShared) {
        // Temporarily switch MOSI/SCK/MISO from LPSPI4 (ALT3) to GPIO
        pinMode(_mosiPin, OUTPUT);
        pinMode(_misoPin, INPUT);
        pinMode(_sckPin,  OUTPUT);
        digitalWriteFast(_sckPin, LOW);  // SCK idles LOW for MODE1
      }
      digitalWrite(_csPin, LOW);
      bbXfer(b0);
      bbXfer(b1);
      bbXfer(b2);
      digitalWrite(_csPin, HIGH);
      if (_bitbangShared) {
        // Restore MOSI/SCK/MISO to LPSPI4 function (ALT3 + SION) for ADC use
        *(portConfigRegister(_mosiPin)) = 3 | 0x10;
        *(portConfigRegister(_sckPin))  = 3 | 0x10;
        *(portConfigRegister(_misoPin)) = 3 | 0x10;
      }
    } else {
      _spi.beginTransaction(_spiSettings);
      IMXRT_LPSPI4_S.CR |= LPSPI_CR_RTF | LPSPI_CR_RRF;  // flush FIFOs before transfer
      digitalWrite(_csPin, LOW);
      dacXferByte(b0);
      dacXferByte(b1);
      dacXferByte(b2);
      digitalWrite(_csPin, HIGH);
      _spi.endTransaction();
    }
  }
};


#endif
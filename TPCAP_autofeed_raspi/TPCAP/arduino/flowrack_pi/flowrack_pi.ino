// === ESP8266: MCP23S17 x2 -> NDJSON over Serial (with 1s snapshot) ===
// No external MCP23S17 library needed. Only SPI.h.
#include <Arduino.h>
#include <SPI.h>

// -------------------- Wiring --------------------
// MCP#1 CS -> D2, MCP#2 CS -> D1  (change if needed)
const uint8_t CS1 = D2;
const uint8_t CS2 = D1;

// If you actually wired A2..A0 and want to use them, set these.
// With separate CS lines you can leave both = 0; HAEN not required.
const uint8_t ADDR1 = 0;   // A2..A0 of chip#1 (0..7)
const uint8_t ADDR2 = 0;   // A2..A0 of chip#2 (0..7)

// ---------------- MCP23S17 registers ----------------
const uint8_t REG_IODIRA = 0x00;
const uint8_t REG_IODIRB = 0x01;
const uint8_t REG_IPOLA  = 0x02;
const uint8_t REG_IPOLB  = 0x03;
const uint8_t REG_GPINTENA = 0x04;
const uint8_t REG_GPINTENB = 0x05;
const uint8_t REG_DEFVALA  = 0x06;
const uint8_t REG_DEFVALB  = 0x07;
const uint8_t REG_INTCONA  = 0x08;
const uint8_t REG_INTCONB  = 0x09;
const uint8_t REG_IOCON    = 0x0A; // or 0x0B (same bank)
const uint8_t REG_GPPUA    = 0x0C;
const uint8_t REG_GPPUB    = 0x0D;
const uint8_t REG_INTFA    = 0x0E;
const uint8_t REG_INTFB    = 0x0F;
const uint8_t REG_INTCAPA  = 0x10;
const uint8_t REG_INTCAPB  = 0x11;
const uint8_t REG_GPIOA    = 0x12;
const uint8_t REG_GPIOB    = 0x13;
const uint8_t REG_OLATA    = 0x14;
const uint8_t REG_OLATB    = 0x15;

// ---------------- Types & timing ----------------
using U2  = uint16_t;  // 16-bit
using U4  = uint32_t;  // 32-bit
using U64 = uint64_t;  // 64-bit millis math

const U4 POLL_MS     = 20;     // 50 Hz
const U4 DEBOUNCE_MS = 80;     // 80 ms stable
const U4 SNAPSHOT_MS = 1000;   // 1 s snapshot

U64 lastPoll = 0, lastSnapshot = 0;
U2  debounced1 = 0, debounced2 = 0;
U2  candidate1 = 0, candidate2 = 0;
U64 candidateAt = 0;

// ---------------- Low-level SPI helpers ----------------
inline uint8_t mcp_op_write(uint8_t addr) { return 0x40 | (addr << 1) | 0x00; } // 0100 A2 A1 A0 0
inline uint8_t mcp_op_read (uint8_t addr) { return 0x40 | (addr << 1) | 0x01; } // 0100 A2 A1 A0 1

uint8_t mcp_read_reg(uint8_t cs, uint8_t addr, uint8_t reg)
{
  digitalWrite(cs, LOW);
  SPI.transfer(mcp_op_read(addr));
  SPI.transfer(reg);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  return val;
}

void mcp_write_reg(uint8_t cs, uint8_t addr, uint8_t reg, uint8_t val)
{
  digitalWrite(cs, LOW);
  SPI.transfer(mcp_op_write(addr));
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
}

void mcp_init_inputs(uint8_t cs, uint8_t addr, bool enablePullups = false, bool inverted = false)
{
  // All pins input
  mcp_write_reg(cs, addr, REG_IODIRA, 0xFF);
  mcp_write_reg(cs, addr, REG_IODIRB, 0xFF);

  // Optional: input polarity (false = normal, true = invert)
  mcp_write_reg(cs, addr, REG_IPOLA,  inverted ? 0xFF : 0x00);
  mcp_write_reg(cs, addr, REG_IPOLB,  inverted ? 0xFF : 0x00);

  // Optional: enable internal pull-ups
  mcp_write_reg(cs, addr, REG_GPPUA,  enablePullups ? 0xFF : 0x00);
  mcp_write_reg(cs, addr, REG_GPPUB,  enablePullups ? 0xFF : 0x00);

  // IOCON: BANK=0, SEQOP=0, HAEN=0 (ok w/ separate CS), ODR=0, MIRROR=0, INTPOL=0
  // If you want to use address pins A2..A0 (HAEN=1), set bit 3 (0b00001000).
  uint8_t iocon = 0x00;
  mcp_write_reg(cs, addr, REG_IOCON, iocon);
}

inline U2 mcp_read16(uint8_t cs, uint8_t addr)
{
  uint8_t a = mcp_read_reg(cs, addr, REG_GPIOA);
  uint8_t b = mcp_read_reg(cs, addr, REG_GPIOB);
  return (U2(b) << 8) | a; // bit0=GPA0 … bit15=GPB7
}

// ---------------- NDJSON helpers ----------------
static inline void send_change(U64 ts, uint8_t idx, uint8_t state) {
  Serial.print("{\"ts\":");    Serial.print(ts);
  Serial.print(",\"idx\":");   Serial.print(idx);      // 1..32
  Serial.print(",\"state\":"); Serial.print(state);    // 0/1
  Serial.println("}");
}

static inline void send_snapshot(U64 ts, U2 low16, U2 high16) {
  Serial.print("{\"ts\":");        Serial.print(ts);
  Serial.print(",\"snapshot\":["); Serial.print(low16);
  Serial.print(",");               Serial.print(high16);
  Serial.println("]}");
}

// ---------------- Arduino lifecycle ----------------
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);

  Serial.begin(115200);
  delay(30);

  SPI.begin();            // ESP8266 default: SCK=D5, MISO=D6, MOSI=D7
  SPI.setFrequency(8000000); // 8 MHz is fine for short wires

  // Configure both expanders as INPUTs with pull-ups if needed (set true if your inputs float)
  const bool USE_PULLUPS = false;   // change to true if sensors are open-collector
  const bool INVERT_LOGIC = false;  // set true if you want 1<->0 flipped
  mcp_init_inputs(CS1, ADDR1, USE_PULLUPS, INVERT_LOGIC);
  mcp_init_inputs(CS2, ADDR2, USE_PULLUPS, INVERT_LOGIC);

  // Seed debounce + first snapshot
  U2 r1 = mcp_read16(CS1, ADDR1);
  U2 r2 = mcp_read16(CS2, ADDR2);
  debounced1 = candidate1 = r1;
  debounced2 = candidate2 = r2;
  candidateAt = millis();
  send_snapshot(millis(), debounced1, debounced2);
}

void loop()
{
  U64 now = millis();

  // Poll + debounce
  if (now - lastPoll >= POLL_MS) {
    lastPoll = now;

    U2 r1 = mcp_read16(CS1, ADDR1);
    U2 r2 = mcp_read16(CS2, ADDR2);

    if (r1 != candidate1 || r2 != candidate2) {
      candidate1 = r1;
      candidate2 = r2;
      candidateAt = now;
    }

    if ((now - candidateAt) >= DEBOUNCE_MS) {
      if (debounced1 != candidate1 || debounced2 != candidate2) {
        uint32_t old32 = (uint32_t(debounced2) << 16) | debounced1;
        uint32_t new32 = (uint32_t(candidate2) << 16) | candidate1;
        uint32_t diff  = old32 ^ new32;

        for (uint8_t i = 0; i < 32; ++i) {
          if (diff & (1UL << i)) {
            uint8_t state = (new32 >> i) & 0x01;
            send_change(now, i + 1, state);  // idx 1..32
          }
        }
        debounced1 = candidate1;
        debounced2 = candidate2;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    }
    yield(); // keep WDT happy
  }

  // 1 s snapshot
  if (now - lastSnapshot >= SNAPSHOT_MS) {
    lastSnapshot = now;
    send_snapshot(now, debounced1, debounced2);
  }
}

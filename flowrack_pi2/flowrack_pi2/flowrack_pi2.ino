// === ESP8266: MCP23S17 x2 -> NDJSON over Serial (with 1s snapshot) ===
// No external MCP23S17 library needed. Only SPI.h.
#include <Arduino.h>
#include <SPI.h>
#define DBG 1
#if DBG
  #define DPRINT(...)   Serial.printf(__VA_ARGS__)
  #define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif


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
const U4 SNAPSHOT_MS = 5000;   // 5-10 s I think 1 sec too low snapshot

U64 lastPoll = 0, lastSnapshot = 0;
U2  debounced1 = 0, debounced2 = 0;
U2  candidate1 = 0, candidate2 = 0;
U64 candidateAt = 0;
volatile uint32_t g_skipped_changes = 0, g_skipped_snapshots = 0;
// ---------------- Low-level SPI helpers ----------------
//inline uint8_t mcp_op_write(uint8_t addr) { return 0x40 | (addr << 1) | 0x00; } // 0100 A2 A1 A0 0
//inline uint8_t mcp_op_read (uint8_t addr) { return 0x40 | (addr << 1) | 0x01; } // 0100 A2 A1 A0 1

// ===== SPI settings (use everywhere for consistency) =====
static constexpr uint32_t SPI_HZ = 8000000; // 8 MHz is safe
#define SPI_SET  SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0)



// Read HP (4-bit DIP) from MCP#2 bits 4..7 (same as original)
static inline uint8_t read_hp_from_dip() {
  U2 r2 = mcp_read16(CS2, ADDR2);
  uint8_t dip = (r2 >> 4) & 0x0F;     // bits 7..4
  return dip;
}
static inline void send_hp(uint8_t hp) {
  Serial.print("{\"hp\":"); Serial.print(hp); Serial.println("}");
}



// ---- write one MCP register ----
inline void mcp_write_reg(uint8_t cs, uint8_t addr, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPI_SET);
  digitalWrite(cs, LOW);
  SPI.transfer(0x40 | (addr << 1) | 0x00); // write opcode
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// ---- read one MCP register ----
inline uint8_t mcp_read_reg(uint8_t cs, uint8_t addr, uint8_t reg) {
  SPI.beginTransaction(SPI_SET);
  digitalWrite(cs, LOW);
  SPI.transfer(0x40 | (addr << 1) | 0x01); // read opcode
  SPI.transfer(reg);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v;
}

// ---- read both ports A+B in one burst (faster) ----
inline uint16_t mcp_read16(uint8_t cs, uint8_t addr) {
  SPI.beginTransaction(SPI_SET);
  digitalWrite(cs, LOW);
  SPI.transfer(0x40 | (addr << 1) | 0x01); // read opcode
  SPI.transfer(0x12);                      // REG_GPIOA, auto-increments to GPIOB
  uint8_t a = SPI.transfer(0x00);
  uint8_t b = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return (uint16_t(b) << 8) | a;           // bit0=GPA0 … bit15=GPB7
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



// ---------------- NDJSON helpers ----------------
// Tune these if you like; bigger guard = less chance to skip
static constexpr int SERIAL_GUARD_CHANGE   = 48;
static constexpr int SERIAL_GUARD_SNAPSHOT = 64;

static inline void send_change(uint64_t ts, uint8_t idx, uint8_t state) {
  if (Serial.availableForWrite() < SERIAL_GUARD_CHANGE) { g_skipped_changes++; return; }  // <<< CHANGED
  Serial.print("{\"ts\":");    Serial.print(ts);
  Serial.print(",\"idx\":");   Serial.print(idx);
  Serial.print(",\"state\":"); Serial.print(state);
  Serial.println("}");
}

static inline void send_snapshot(uint64_t ts, uint16_t low16, uint16_t high16) {
  if (Serial.availableForWrite() < SERIAL_GUARD_SNAPSHOT) { g_skipped_snapshots++; return; }  // <<< CHANGED
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
  DPRINTLN("\n[BOOT] ESP8266 start, 115200 baud");
  delay(30);

  SPI.begin();                     // ESP8266: SCK=D5, MISO=D6, MOSI=D7
  SPI.setFrequency(SPI_HZ);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  DPRINT("[SPI] OK freq=%lu Hz, mode=0\n", (unsigned long)SPI_HZ);
  
  // Configure both expanders as INPUTs with pull-ups if needed (set true if your inputs float)
  const bool USE_PULLUPS = false;   // change to true if sensors are open-collector
  const bool INVERT_LOGIC = false;  // set true if you want 1<->0 flipped
  
  mcp_init_inputs(CS1, ADDR1, USE_PULLUPS, INVERT_LOGIC);
  uint8_t t1a = mcp_read_reg(CS1, ADDR1, REG_IODIRA);
  uint8_t t1b = mcp_read_reg(CS1, ADDR1, REG_IODIRB);
  DPRINT("[MCP1] IODIR A=0x%02X B=0x%02X (expect FF FF)\n", t1a, t1b);
  mcp_init_inputs(CS2, ADDR2, USE_PULLUPS, INVERT_LOGIC);
  uint8_t t2a = mcp_read_reg(CS2, ADDR2, REG_IODIRA);
  uint8_t t2b = mcp_read_reg(CS2, ADDR2, REG_IODIRB);
  DPRINT("[MCP2] IODIR A=0x%02X B=0x%02X (expect FF FF)\n", t2a, t2b);

  // Seed debounce + first snapshot
  U2 r1 = mcp_read16(CS1, ADDR1);
  U2 r2 = mcp_read16(CS2, ADDR2);
  debounced1 = candidate1 = r1;
  debounced2 = candidate2 = r2;
  candidateAt = millis();
  send_snapshot(millis(), debounced1, debounced2);

  // >>> ADD: announce HP (DIP) once at boot
  uint8_t hp = read_hp_from_dip();
  send_hp(hp);
  
  DPRINT("[INIT] first read: ch1=0x%04X ch2=0x%04X\n", debounced1, debounced2);
}

void loop()
{
  U64 now = millis();

  // Poll + debounce
  if (now - lastPoll >= POLL_MS) {
    lastPoll = now;

    U2 r1 = mcp_read16(CS1, ADDR1);
    U2 r2 = mcp_read16(CS2, ADDR2);
    static U2 lastRaw1 = 0xFFFF, lastRaw2 = 0xFFFF;
    if (r1 != lastRaw1 || r2 != lastRaw2) {
      DPRINT("[RAW ] ch1=0x%04X ch2=0x%04X\n", r1, r2);
      lastRaw1 = r1; lastRaw2 = r2;
    }

    
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
            DPRINT("[EDGE] idx=%u state=%u ts=%llu\n", (unsigned)(i+1), (unsigned)state, (unsigned long long)now);

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


  // --- periodic HP announce (every 60s) ---  // <== PASTE THIS BLOCK HERE
  static U64 lastHp = 0;
  if (millis() - lastHp >= 60000) {
    lastHp = millis();
    uint8_t hp = read_hp_from_dip();   // your helper
    send_hp(hp);                       // prints {"hp":N}
  }
  
  // 1 s snapshot
  if (now - lastSnapshot >= SNAPSHOT_MS) {
    lastSnapshot = now;
    send_snapshot(now, debounced1, debounced2);
    static uint32_t hb = 0;
    hb++;
    if ((hb % 10) == 0) {
      DPRINT("[HEALTH] skipped change=%lu snap=%lu avail=%u\n",
             (unsigned long)g_skipped_changes,
             (unsigned long)g_skipped_snapshots,
             (unsigned)Serial.availableForWrite());
    }
    
  }
}

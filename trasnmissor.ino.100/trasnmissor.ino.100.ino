#include <Arduino.h>
#include <string.h>

static const uint8_t encode_table[] = {
  0x0D, 0x0E, 0x13, 0x15, 0x16, 0x19,
  0x1A, 0x1C, 0x23, 0x25, 0x26, 0x29,
  0x2A, 0x2C, 0x32, 0x34
};

uint8_t dec4b6b(uint8_t symbol) {
  for (uint8_t i = 0; i < 16; i++)
    if (encode_table[i] == symbol)
      return i;
  return 0xFF;
}

void enc4b6b(uint8_t byte_val, uint8_t &hi, uint8_t &lo) {
  hi = encode_table[byte_val >> 4];
  lo = encode_table[byte_val & 0x0F];
}

#define PREAMBLE_BYTE      0x2A
#define PREAMBLE_BYTES_TX  12
#define PREAMBLE_BYTES_ACK 12
#define TIMEOUT_MS         5000
#define RX_SETTLE_MS       200
#define FLAG_BYTE          0x7E
#define RX_PIN             26
#define TX_PIN             27
#define TX_VCC_PIN         33

enum class RecvResult : uint8_t { OK, DISCARD, TIMEOUT };
enum Type : uint8_t { DATA = 0, ACK = 1, END = 2, IMG = 3 };

struct Frame;
uint8_t  crc8(const Frame &frame);
void     enc4b6b(uint8_t byte_val, uint8_t &hi, uint8_t &lo);
bool     readRawByte(uint8_t &out, uint32_t timeoutMs);
bool     read4b6b(uint8_t &out, uint32_t timeoutMs);
bool     waitForFlag(uint32_t timeoutMs);
void     sendFrame(const Frame &frame);
uint8_t  receiveACK();
void     sendWithARQ(const Frame &frame);

struct Frame {
  uint8_t flag;
  Type    type;
  uint8_t seq;
  uint8_t len;
  uint8_t data[16];
  uint8_t fcs;

  Frame(Type t, uint8_t s, uint8_t l, const uint8_t *payload = nullptr)
      : flag(FLAG_BYTE), type(t), seq(s), len(l), fcs(0) {
    memset(this->data, 0, sizeof(this->data));
    if (payload && l > 0)
      memcpy(this->data, payload, l);
    this->fcs = crc8(*this);
  }

  Frame(Type t) : flag(FLAG_BYTE), type(t), seq(0), len(0), fcs(0) {
    memset(this->data, 0, sizeof(this->data));
  }

  Frame() : flag(FLAG_BYTE), type(Type::DATA), seq(0), len(0), fcs(0) {
    memset(this->data, 0, sizeof(this->data));
  }

  uint8_t       *bytes()       { return reinterpret_cast<uint8_t *>(this); }
  const uint8_t *bytes() const { return reinterpret_cast<const uint8_t *>(this); }

  bool validate() const { return crc8(*this) == 0; }

  const uint8_t *begin() const { return bytes(); }
  const uint8_t *end()   const {
    return bytes() + sizeof(flag) + sizeof(type) + sizeof(seq) +
           sizeof(len) + sizeof(data) + sizeof(fcs);
  }
};

uint8_t crc8(const Frame &frame) {
  uint8_t crc = 0x00;
  const uint8_t *b = frame.bytes();
  for (uint8_t i = 1; i < 20; i++) {
    crc ^= b[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x87 : (crc << 1);
  }
  return crc;
}

// Livro de estudo
const uint8_t custom_char[] = {
  0x00, 0x0F, 0x15, 0x15, 0x15, 0x15, 0x1F, 0x00
};

inline void txPowerOn() {
  digitalWrite(TX_VCC_PIN, HIGH);
  delay(100);
}

inline void txPowerOff() {
  digitalWrite(TX_VCC_PIN, LOW);
  delay(20);
}

bool readRawByte(uint8_t &out, uint32_t timeoutMs = TIMEOUT_MS) {
  uint32_t t0 = millis();
  while (!Serial2.available()) {
    if (millis() - t0 > timeoutMs) return false;
  }
  out = Serial2.read();
  return true;
}

bool read4b6b(uint8_t &out, uint32_t timeoutMs = TIMEOUT_MS) {
  uint8_t hi, lo;
  if (!readRawByte(hi, timeoutMs) || !readRawByte(lo, timeoutMs))
    return false;
  uint8_t a = dec4b6b(hi);
  uint8_t b = dec4b6b(lo);
  if (a == 0xFF || b == 0xFF) return false;
  out = (a << 4) | b;
  return true;
}

bool waitForFlag(uint32_t timeoutMs = TIMEOUT_MS) {
  uint32_t t0 = millis();
  uint8_t hi;
  if (!readRawByte(hi, timeoutMs)) return false;
  while (millis() - t0 < timeoutMs) {
    uint8_t lo;
    if (!readRawByte(lo, timeoutMs)) return false;
    uint8_t dhi = dec4b6b(hi);
    uint8_t dlo = dec4b6b(lo);
    if (dhi != 0xFF && dlo != 0xFF)
      if (((dhi << 4) | dlo) == FLAG_BYTE) return true;
    hi = lo;
  }
  return false;
}

void sendFrame(const Frame &frame) {
  while (Serial2.available()) Serial2.read();
  txPowerOn();
  for (uint8_t i = 0; i < PREAMBLE_BYTES_TX; i++) {
    uint8_t hi, lo;
    enc4b6b(PREAMBLE_BYTE, hi, lo);
    Serial2.write(hi);
    Serial2.write(lo);
  }
  for (const uint8_t b : frame) {
    uint8_t hi, lo;
    enc4b6b(b, hi, lo);
    Serial2.write(hi);
    Serial2.write(lo);
  }
  Serial2.flush();
  txPowerOff();
  while (Serial2.available()) Serial2.read();
}

uint8_t receiveACK() {
  if (!waitForFlag(5000)) {
    Serial.println("[TX] Timeout aguardando flag do ACK");
    return 0xFF;
  }

  uint8_t type_raw, seq, len;
  if (!read4b6b(type_raw) || !read4b6b(seq) || !read4b6b(len)) {
    Serial.println("[TX] Timeout lendo header do ACK");
    return 0xFF;
  }

  if (type_raw != Type::ACK) {
    Serial.printf("[TX] Tipo inesperado: %d (esperava ACK)\n", type_raw);
    // Drena o restante e tenta de novo
    delay(100);
    while (Serial2.available()) Serial2.read();
    return 0xFF;
  }

  if (len != 0) {
    Serial.println("[TX] ACK com len != 0");
    return 0xFF;
  }

  Frame ack(Type::ACK, seq, 0);

  uint8_t fcs;
  if (!read4b6b(fcs)) {
    Serial.println("[TX] Timeout lendo FCS do ACK");
    return 0xFF;
  }
  ack.fcs = fcs;

  if (!ack.validate()) {
    Serial.printf("[TX] FCS inválido no ACK (seq=%d)\n", seq);
    return 0xFF;
  }

  return seq;
}

void sendWithARQ(const Frame &frame) {
  uint8_t tentativas = 0;
  while (tentativas < 10) {
    sendFrame(frame);
    Serial.printf("[TX] Enviado seq=%d tipo=%d len=%d (tentativa %d)\n",
                  frame.seq, frame.type, frame.len, tentativas + 1);

    uint32_t t0 = millis();
    while (millis() - t0 < TIMEOUT_MS) {
      if (!Serial2.available()) continue;

      uint8_t ackSeq = receiveACK();
      if (ackSeq == frame.seq) {
        Serial.printf("[TX] ACK confirmado seq=%d\n", ackSeq);
        return;
      }
      if (ackSeq != 0xFF) {
        Serial.printf("[TX] ACK seq errado: %d (esperava %d)\n", ackSeq, frame.seq);
      }
    }
    tentativas++;
    Serial.printf("[TX] Timeout — retransmitindo (tentativa %d)\n", tentativas + 1);
  }
  Serial.println("[TX] Falha após 10 tentativas");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(1200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(TX_VCC_PIN, OUTPUT);
  txPowerOff();

  Serial.println("[TX] Transmissor pronto!");
  Serial.println("  Digite uma mensagem (max 16 chars) + Enter para enviar texto");
  Serial.println("  Digite 'IMG' para enviar livro");
  Serial.println("  Digite 'END' para encerrar");
}

uint8_t seq = 0;

void loop() {
  if (!Serial.available()) return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();
  if (entrada.length() == 0) return;

  if (entrada.equalsIgnoreCase("END")) {
    Frame end_frame(Type::END, 0, 0);
    sendFrame(end_frame);
    Serial.println("[TX] END enviado");
    seq = 0;
    return;
  }

  if (entrada.equalsIgnoreCase("IMG")) {
    Frame frame(Type::IMG, seq, (uint8_t)sizeof(custom_char), custom_char);
    sendWithARQ(frame);
    seq ^= 1;
    return;
  }

  if (entrada.length() > 16) {
    Serial.println("[TX] ERRO: max 16 caracteres");
    return;
  }

  Frame frame(Type::DATA, seq, (uint8_t)entrada.length(),
              (const uint8_t *)entrada.c_str());
  sendWithARQ(frame);
  seq ^= 1;
}
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define Pino_RX_RF 26
#define Pino_TX_RF 27

HardwareSerial RFSerial(1);
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define FLAG_BYTE          0x7E
#define PREAMBLE_BYTE      0x2A
#define TIMEOUT_MS         3000
#define RX_SETTLE_MS       20
#define PREAMBLE_BYTES_ACK 12

enum Type : uint8_t { DATA = 0, ACK = 1, END = 2, IMG = 3 };

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

uint8_t calcularCRC(uint8_t tipo, uint8_t seq, uint8_t len, uint8_t *data) {
  uint8_t frame_buf[21];
  frame_buf[0] = FLAG_BYTE;
  frame_buf[1] = tipo;
  frame_buf[2] = seq;
  frame_buf[3] = len;
  memset(&frame_buf[4], 0, 16);
  memcpy(&frame_buf[4], data, len);
  frame_buf[20] = 0;

  uint8_t crc = 0x00;
  for (uint8_t i = 1; i < 20; i++) {
    crc ^= frame_buf[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x87 : (crc << 1);
  }
  return crc;
}

bool validarFrame(uint8_t tipo, uint8_t seq, uint8_t len, uint8_t *data, uint8_t fcs_recebido) {
  uint8_t frame_buf[21];
  frame_buf[0] = FLAG_BYTE;
  frame_buf[1] = tipo;
  frame_buf[2] = seq;
  frame_buf[3] = len;
  memset(&frame_buf[4], 0, 16);
  memcpy(&frame_buf[4], data, len);
  frame_buf[20] = fcs_recebido;

  uint8_t crc = 0x00;
  for (uint8_t i = 1; i < 21; i++) {
    crc ^= frame_buf[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x87 : (crc << 1);
  }
  return crc == 0;
}

bool readRawByte(uint8_t &out, uint32_t timeoutMs = TIMEOUT_MS) {
  uint32_t t0 = millis();
  while (!RFSerial.available()) {
    if (millis() - t0 > timeoutMs) return false;
  }
  out = RFSerial.read();
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

void enviarAck(uint8_t seq) {
  delay(50);
  while (RFSerial.available()) RFSerial.read();

  for (uint8_t i = 0; i < PREAMBLE_BYTES_ACK; i++) {
    uint8_t hi, lo;
    enc4b6b(PREAMBLE_BYTE, hi, lo);
    RFSerial.write(hi);
    RFSerial.write(lo);
  }

  uint8_t data_vazio[16] = {0};
  uint8_t fcs = calcularCRC(ACK, seq, 0, data_vazio);

  uint8_t quadro[5] = { FLAG_BYTE, (uint8_t)ACK, seq, 0, fcs };
  for (int i = 0; i < 5; i++) {
    uint8_t hi, lo;
    enc4b6b(quadro[i], hi, lo);
    RFSerial.write(hi);
    RFSerial.write(lo);
  }

  RFSerial.flush();
  delay(200);
  while (RFSerial.available()) RFSerial.read();

  Serial.printf("[RX] ACK enviado seq=%d\n", seq);
}

uint8_t seqEsperada = 0;

void processarQuadro(uint8_t tipo, uint8_t seq, uint8_t len, uint8_t *dados, uint8_t fcs) {

  if (!validarFrame(tipo, seq, len, dados, fcs)) {
    Serial.printf("[RX] REJEITADO: FCS inválido (seq=%d)\n", seq);
    return;
  }

  if (tipo == END) {
    Serial.println("[RX] END recebido — reiniciando");
    seqEsperada = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Aguardando...");
    return;
  }

  if (seq != seqEsperada) {
    Serial.printf("[RX] Fora de ordem: chegou %d esperava %d\n", seq, seqEsperada);
    if (seqEsperada > 0) enviarAck(seqEsperada - 1);
    return;
  }

  if (tipo == DATA) {
    dados[len] = '\0';
    Serial.printf("[RX] ACEITO TEXTO [SEQ %d]: %s\n", seq, (char *)dados);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print((char *)dados);

    enviarAck(seqEsperada);
    seqEsperada ^= 1;
    return;
  }

  if (tipo == IMG) {
    Serial.printf("[RX] ACEITO IMAGEM [SEQ %d]\n", seq);

    lcd.createChar(0, dados);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Livro recebido:");
    lcd.setCursor(7, 1);
    lcd.write((uint8_t)0);

    enviarAck(seqEsperada);
    seqEsperada ^= 1;
    return;
  }
}

void setup() {
  Serial.begin(115200);
  RFSerial.begin(1200, SERIAL_8N1, Pino_RX_RF, Pino_TX_RF);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Aguardando...");

  Serial.println("[RX] Receptor ativo");
}

void loop() {
  if (!waitForFlag(100)) return;

  uint8_t tipo_raw, seq, len;
  if (!read4b6b(tipo_raw) || !read4b6b(seq) || !read4b6b(len)) {
    Serial.println("[RX] Timeout lendo header");
    return;
  }

  if (len > 16) {
    Serial.printf("[RX] len inválido: %d\n", len);
    return;
  }

  uint8_t dados[17] = {0};
  for (uint8_t i = 0; i < 16; i++) {
    if (!read4b6b(dados[i])) {
      Serial.println("[RX] Timeout lendo payload");
      return;
    }
  }

  uint8_t fcs;
  if (!read4b6b(fcs)) {
    Serial.println("[RX] Timeout lendo FCS");
    return;
  }

  processarQuadro(tipo_raw, seq, len, dados, fcs);
}
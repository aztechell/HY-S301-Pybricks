#define LUMP_HOST_DETECT_OFF

#include <LumpDeviceBuilder.h>

// --------------------------
// LPF2 (PUP) over Serial1
// --------------------------
#define DEVICE_SERIAL Serial1
#define RX_PIN        0      // D0 / RX1
#define TX_PIN        1      // D1 / TX0
#define BAUDRATE      115200

// --------------------------
// Линейный датчик (мультиплексор)
// --------------------------
#define S0_PIN        7
#define S1_PIN        8
#define S2_PIN        9
#define LINE_ANALOG   A1     // аналоговый выход мультиплексора

// --------------------------
// Один простой режим: 8 байт
// --------------------------
// name="LINE8", type=DATA8, numData=8
LumpMode modes[] = {
  { "LINE8", DATA8, 8, 3, 0, "",
    {0, 255}, {0, 100}, {0, 255} }
};

uint8_t numModes = sizeof(modes) / sizeof(LumpMode);

// type = 61 (можно оставить как у color sensor)
LumpDevice<HardwareSerial> device(
  &DEVICE_SERIAL,
  RX_PIN,
  TX_PIN,
  61,
  BAUDRATE,
  modes,
  numModes
);


// --------------------------
// Утилиты
// --------------------------
inline void selectChannel(uint8_t ch) {
  digitalWrite(S0_PIN, (ch >> 0) & 1);
  digitalWrite(S1_PIN, (ch >> 1) & 1);
  digitalWrite(S2_PIN, (ch >> 2) & 1);
}

inline uint8_t analogToByte(int raw) {
  if (raw < 0)   raw = 0;
  if (raw > 1023) raw = 1023;
  return (uint8_t)((uint32_t)raw * 255UL / 1023UL);
}


// --------------------------
// Чтение 8 каналов (левый→правый)
// --------------------------
void readLine(uint8_t out[8]) {
  for (uint8_t i = 0; i < 8; i++) {
    // физически читаем каналы 7,6,5,4,3,2,1,0
    uint8_t ch = 7 - i;
    selectChannel(ch);
    delayMicroseconds(5);
    int raw = analogRead(LINE_ANALOG);

    // логически складываем как [0..7] = слева направо
    out[i] = analogToByte(raw);
  }
}


// --------------------------
// LUMP state machine
// --------------------------
void runDeviceModes() {
  static uint32_t prev = 0;
  uint32_t now = millis();

  // Обновление и отправка раз в ~10 мс
  if (now - prev >= 10) {
    prev = now;

    if (device.state() == LumpDeviceState::Communicating) {
      uint8_t line[8];
      readLine(line);      // просто 8 байт 0..255
      device.send(line, 8);
    }
  }
}


// --------------------------
// setup / loop
// --------------------------
void setup() {
  DEVICE_SERIAL.begin(BAUDRATE);
  device.begin();

  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(LINE_ANALOG, INPUT);
}

void loop() {
  device.run();
  runDeviceModes();
}

#define LUMP_HOST_DETECT_OFF

#include <LumpDeviceBuilder.h>

// --------------------------------------------------
// LPF2 (PUP) UART over Serial1 (Pro Micro / 32U4)
// --------------------------------------------------
#define DEVICE_SERIAL Serial1
#define RX_PIN 0  // D0 / RX1
#define TX_PIN 1  // D1 / TX0
#define BAUDRATE 115200

// --------------------------------------------------
// Линия датчиков HW-529 / HY-S301
// --------------------------------------------------
#define S0_PIN 7
#define S1_PIN 8
#define S2_PIN 9
#define LINE_ANALOG A1  // аналоговый выход мультиплексора

// --------------------------------------------------
// Описание режимов как у SPIKE Color Sensor
// --------------------------------------------------
LumpMode modes[] = {
  // 0: COLOR (DATA8,1)
  { "COLOR", DATA8, 1, 3, 0, "", { 0, 10 }, { 0, 100 }, { 0, 10 } },

  // 1: REFLT (DATA8,1)
  { "REFLT", DATA8, 1, 3, 0, "%", { 0, 100 }, { 0, 100 }, { 0, 100 } },

  // 2: AMBI (DATA8,1)
  { "AMBI", DATA8, 1, 3, 0, "%", { 0, 100 }, { 0, 100 }, { 0, 100 } },

  // 3: LIGHT (DATA8,3)
  { "LIGHT", DATA8, 3, 3, 0, "", { 0, 100 }, { 0, 100 }, { 0, 100 } },

  // 4: RREFL (DATA16,2)
  { "RREFL", DATA16, 2, 5, 0, "", { 0, 1023 }, { 0, 100 }, { 0, 1023 } },

  // 5: RGB I (DATA16,4)
  { "RGB I", DATA16, 4, 5, 0, "", { 0, 1023 }, { 0, 100 }, { 0, 1023 } },

  // 6: HSVl (DATA16,3)
  { "HSVl", DATA16, 3, 5, 0, "", { 0, 1023 }, { 0, 100 }, { 0, 1023 } },

  // 7: SHSV (DATA16,4)
  { "SHSV", DATA16, 4, 5, 0, "", { 0, 1023 }, { 0, 100 }, { 0, 1023 } },

  // 8: DEBUG (DATA16,4)
  { "DEBUG", DATA16, 4, 5, 0, "", { 0, 65535 }, { 0, 100 }, { 0, 65535 } },

  // 9: CALIB (DATA16,7) — line-sensor пакет
  { "CALIB", DATA16, 7, 5, 0, "", { 0, 65535 }, { 0, 100 }, { 0, 65535 } },
};

uint8_t numModes = sizeof(modes) / sizeof(LumpMode);

// type = 61 → SPIKE_COLOR_SENSOR
LumpDevice<HardwareSerial> device(
  &DEVICE_SERIAL,
  RX_PIN,
  TX_PIN,
  61,
  BAUDRATE,
  modes,
  numModes);

// --------------------------------------------------
// Состояние линии
// --------------------------------------------------
struct LineState {
  uint8_t vals[8];  // 0..255
  uint8_t threshold;
  uint8_t blackBin;
  uint8_t whiteBin;
  uint8_t blackWidth;
  uint8_t whiteWidth;
  int8_t blackErr;   // -7..+7
  int8_t whiteErr;   // -7..+7
  int16_t calib[7];  // пакет для CALIB (mode 9)
} ls;


// --------------------------------------------------
// Утилиты
// --------------------------------------------------
inline void selectChannel(uint8_t ch) {
  digitalWrite(S0_PIN, (ch >> 0) & 1);
  digitalWrite(S1_PIN, (ch >> 1) & 1);
  digitalWrite(S2_PIN, (ch >> 2) & 1);
}

inline uint8_t analogToByte(int raw) {
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  return (uint8_t)((uint32_t)raw * 255UL / 1023UL);
}

inline uint8_t popcount8(uint8_t x) {
  uint8_t c = 0;
  while (x) {
    x &= (x - 1);
    c++;
  }
  return c;
}

int8_t calcError(uint8_t mask) {
  const int8_t w[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };
  int16_t sum = 0, cnt = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (mask & (1 << i)) {
      sum += w[i];
      cnt++;
    }
  }
  return (cnt == 0) ? 0 : (int8_t)(sum / cnt);
}

// --------------------------------------------------
// Обновление линии + заполнение CALIB
// --------------------------------------------------

void updateLineState() {
  uint8_t minv = 255;
  uint8_t maxv = 0;

  for (uint8_t i = 0; i < 8; i++) {
    // читаем физический канал 7,6,5,4,3,2,1,0
    uint8_t ch = 7 - i;
    selectChannel(ch);
    delayMicroseconds(5);
    uint8_t v = analogToByte(analogRead(LINE_ANALOG));

    // а в массив кладём уже в "логическом" порядке:
    // ls.vals[0] = левый, ls.vals[7] = правый
    ls.vals[i] = v;

    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }

  ls.threshold = (uint8_t)((minv + maxv) / 2);

  ls.blackBin = 0;
  ls.whiteBin = 0;

  for (uint8_t i = 0; i < 8; i++) {
    if (ls.vals[i] < ls.threshold)
      ls.blackBin |= (1 << i);
    else
      ls.whiteBin |= (1 << i);
  }

  ls.blackWidth = popcount8(ls.blackBin);
  ls.whiteWidth = popcount8(ls.whiteBin);

  ls.blackErr = calcError(ls.blackBin);
  ls.whiteErr = calcError(ls.whiteBin);

  uint8_t blackErrEnc = (uint8_t)(ls.blackErr + 8);
  uint8_t whiteErrEnc = (uint8_t)(ls.whiteErr + 8);

  // CALIB-пакет (mode 9) остаётся тем же — но теперь vals уже в нужном порядке
  ls.calib[0] = ((int16_t)ls.vals[1] << 8) | ls.vals[0];
  ls.calib[1] = ((int16_t)ls.vals[3] << 8) | ls.vals[2];
  ls.calib[2] = ((int16_t)ls.vals[5] << 8) | ls.vals[4];
  ls.calib[3] = ((int16_t)ls.vals[7] << 8) | ls.vals[6];
  ls.calib[4] = ((int16_t)ls.blackWidth << 8) | blackErrEnc;
  ls.calib[5] = ((int16_t)whiteErrEnc << 8) | ls.blackBin;
  ls.calib[6] = ((int16_t)ls.whiteBin << 8) | ls.whiteWidth;
}



// --------------------------------------------------
// Отправка данных в зависимости от режима
// --------------------------------------------------
void sendModeData(uint8_t mode) {

  // Mode 0: (0,)
  if (mode == 0) {
    uint8_t v = 0;
    device.send(v);  // DATA8,1
    return;
  }

  // Mode 1: (8,)
  if (mode == 1) {
    uint8_t v = 8;
    device.send(v);
    return;
  }

  // Mode 2: (8,)
  if (mode == 2) {
    uint8_t v = 8;
    device.send(v);
    return;
  }

  // Mode 3: (8, 0, 0)
  if (mode == 3) {
    uint8_t arr[3] = { 8, 0, 0 };
    device.send(arr, 3);  // DATA8,3
    return;
  }

  // Mode 4: (2048, 2048)
  if (mode == 4) {
    int16_t arr[2] = { 2048, 2048 };
    device.send(arr, 2);  // DATA16,2
    return;
  }

  // Mode 5: (827, 827, 827, 0)
  if (mode == 5) {
    int16_t arr[4] = { 827, 827, 827, 0 };
    device.send(arr, 4);  // DATA16,4
    return;
  }

  // Mode 6: (25700, 25700, 25700)
  if (mode == 6) {
    int16_t arr[3] = { 25700, 25700, 25700 };
    device.send(arr, 3);  // DATA16,3
    return;
  }

  // Mode 7: (8, 2056, 880, 800)
  if (mode == 7) {
    int16_t arr[4] = { 8, 2056, 880, 800 };
    device.send(arr, 4);  // DATA16,4
    return;
  }

  // Mode 8: (25700, 25700, 880, 800)
  if (mode == 8) {
    int16_t arr[4] = { 25700, 25700, 880, 800 };
    device.send(arr, 4);  // DATA16,4
    return;
  }

  // Mode 9: CALIB — реальный line-sensor пакет
  if (mode == 9) {
    device.send(ls.calib, 7);  // DATA16,7
    return;
  }
}


// --------------------------------------------------
// LUMP state machine c таймером
// --------------------------------------------------
void runDeviceModes() {
  uint32_t now = millis();
  static uint32_t prev = 0;

  // Обновляем сенсоры и отправляем данные раз в ~10 мс
  if (now - prev >= 10) {
    prev = now;

    updateLineState();

    if (device.state() == LumpDeviceState::Communicating) {
      sendModeData(device.mode());
    }
  }
}


// --------------------------------------------------
// setup / loop
// --------------------------------------------------
void setup() {
  DEVICE_SERIAL.begin(BAUDRATE);
  device.begin();

  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(LINE_ANALOG, INPUT);
}

void loop() {
  device.run();      // протокол LPF2
  runDeviceModes();  // логика режимов + линия по таймеру
}

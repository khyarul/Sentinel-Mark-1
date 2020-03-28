#include <Wire.h>
#include <U8x8lib.h>
#include <extEEPROM.h>
#include <Servo.h>
//#include <STM32ADC.h>
//STM32ADC adc(ADC1);

U8X8_SSD1306_128X64_NONAME_HW_I2C lcd(U8X8_PIN_NONE);
extEEPROM EE(kbits_256, 1, 64);
Servo servoL, servoR;

#define MAXPLAN 5

struct ee {
  int8 ACTION[MAXPLAN][100];
  int8 SENSOR[MAXPLAN][100];
  int8 RSPEED[MAXPLAN][100];
  int8 LSPEED[MAXPLAN][100];
  int8 COUNTERMODE[MAXPLAN][100];
  int16 COUNTER[MAXPLAN][100];
  int8 VA[MAXPLAN][100];
  int16 COUNTA[MAXPLAN][100];
  int8 COUNTERAMODE[MAXPLAN][100];
  int8 VB[MAXPLAN][100];
  int16 COUNTB[MAXPLAN][100];
  int8 COUNTERBMODE[MAXPLAN][100];
  int8 pid[MAXPLAN][100];
  int8 FOLLOWMODE[MAXPLAN][100];
  int8 JUMP[MAXPLAN][100];
  int8 jumpP[MAXPLAN][100];
  int8 jumpI[MAXPLAN][100];
  int8 PLAN;
  int8 I;
  int8 STOP[MAXPLAN];
  int8 CP[10];
  int8 CPI;
  int8 Va[10];
  uint8 Ta[10];
  uint16 sensorRef[14];
  uint8 V;
  int8 PID;
  uint8 kp[3], kd[3];
  int8 Vmin, Vmax;
  uint8 Ts;
  int8 LINE;
  int8 sensitivity;
  uint8 SV1[2], SV2[2];
  int16 Dly;
} ee;

void eeRead() {
  byte *p = (byte*)(void*)&ee;
  EE.read(0, p, sizeof(ee));
}

void eeWrite() {
  byte *p = (byte*)(void*)&ee;
  EE.write(0, p, sizeof(ee));
}

//Button
#define OK gpio_read_bit(GPIOB, 5) ? HIGH : LOW
#define UP gpio_read_bit(GPIOB, 4) ? HIGH : LOW
#define DOWN gpio_read_bit(GPIOB, 3) ? HIGH : LOW
#define BACK gpio_read_bit(GPIOB, 12) ? HIGH : LOW
#define PLUS gpio_read_bit(GPIOB, 13) ? HIGH : LOW
#define MIN gpio_read_bit(GPIOB, 11) ? HIGH : LOW

//Motor Driver
HardwareTimer timer(1);
#define pwmL PA10
#define pwmR PA9
#define dirL PB8
#define dirR PA8
//int16 speedL = 0, speedR = 0;
int16 error = 0, lastError = 0;
uint32 lastTime = 0;

//Servo
//#define servoR 5
//#define servoL 4

//Motor Encoder
int8 last_en1L, last_en1R;
volatile int rPos;
volatile int lPos;
#define RPOS rPos/5
#define LPOS lPos/5

//Sensor
#define led_ON digitalWrite(led, 1)
#define led_OFF digitalWrite(led, 0)
#define led PA0
#define sensorL PA4
#define sensorR PA5
#define A PC13
#define B PC14
#define C PC15
uint8 pinSensor[3] __FLASH__ = {PA5, PA4, PB0};
uint16 arraySensor[14], adcSensor[3];
int8 sensor_logic = 0;
uint16 bitSensor = 0;
uint16 sensor1 = 0, sensor2 = 0;

//Battery Indicator
#define batt PB0

//Navigation Mode
#define BLK 0
#define WHT 1
#define RGT 2
#define LFT 3
#define FWD 4
#define BWD 5
#define PICK 6
#define DROP 7
#define BLOW 8

#define NONE 0
#define DIRECT 1
#define EQ 2
#define OR 3
#define XOR 4

#define TIMER 0
#define ENCODERR 1
#define ENCODERL 2

#define FLC 0
#define FLR 1
#define FLL 2

#define off 0
#define on 1
int8 line;
//=================

uint8 bar[8][8] __FLASH__ = {{0, 128, 128, 128, 128, 128, 128, 0}, {0, 192, 192, 192, 192, 192, 192, 0},
  {0, 224, 224, 224, 224, 224, 224, 0}, {0, 240, 240, 240, 240, 240, 240, 0},
  {0, 248, 248, 248, 248, 248, 248, 0}, {0, 252, 252, 252, 252, 252, 252, 0},
  {0, 254, 254, 254, 254, 254, 254, 0}, {0, 255, 255, 255, 255, 255, 255, 0}
};
uint8 bat[6][8] __FLASH__ = {{254, 130, 131, 131, 131, 131, 130, 254}, {254, 194, 195, 195, 195, 195, 194, 254},
  {254, 226, 227, 227, 227, 227, 226, 254}, {254, 242, 243, 243, 243, 243, 242, 254},
  {254, 250, 251, 251, 251, 251, 250, 254}, {254, 254, 255, 255, 255, 255, 254, 254}
};
uint8 check[8] __FLASH__ = {255, 129, 189, 189, 189, 189, 129, 255};
uint8 uncheck[8] __FLASH__ = {255, 129, 129, 129, 129, 129, 129, 255};
uint8 savedata[8] __FLASH__ = {31, 123, 115, 96, 96, 115, 123, 31};
uint8 zzzz[8] __FLASH__ = {0, 0, 0, 0, 0, 0, 0, 255};
uint8 zzzzz[8] __FLASH__ = {6, 6, 6, 6, 6, 6, 6, 6};
char buff[16];
int8 idx = 4;

void setup() {
  initEncoder();

  initMotor();
  setMotor(0, 0);

  initButton();

  pinMode(led, OUTPUT);
  led_OFF;

  initSensor();
  switchSensor(0);

  lcd.begin();
  lcd.setPowerSave(0);
  lcd.setFlipMode(1);
  //u8x8_font_artossans8_r
  //u8x8_font_chroma48medium8_r
  //u8x8_font_5x8_f
  lcd.setFont(u8x8_font_chroma48medium8_r);

  EE.begin(EE.twiClock100kHz);

  rPos = 0;
  lPos = 0;

  eeRead();
  delay(1000);
  if (!OK) {
    if (!BACK) {
      for (int8 i = 0; i < MAXPLAN; i++) {
        for (int8 j = 0; j < 100; j++) {
          ee.ACTION[i][j] = 0;
          ee.SENSOR[i][j] = 0;
          ee.RSPEED[i][j] = 0;
          ee.LSPEED[i][j] = 0;
          ee.COUNTERMODE[i][j] = TIMER;
          ee.COUNTER[i][j] = 0;
          ee.VA[i][j] = 50;
          ee.COUNTA[i][j] = 0;
          ee.COUNTERAMODE[i][j] = TIMER;
          ee.VB[i][j] = 50;
          ee.COUNTB[i][j] = 0;
          ee.COUNTERBMODE[i][j] = TIMER;
          ee.pid[i][j] = 2;
          ee.FOLLOWMODE[i][j] = FLC;
          ee.JUMP[i][j] = off;
          ee.jumpP[i][j] = 0;
          ee.jumpI[i][j] = 0;
          ee.STOP[i] = 99;
        }
      }
      for (int8 i = 0; i < 10; i++) {
        ee.CP[i] = 0;
        ee.Va[i] = 0;
        ee.Ta[i] = 0;
      }
      ee.kp[0] = 14;   //14, 19, 27
      ee.kd[0] = 130;  //130, 140, 155
      ee.kp[1] = 20;
      ee.kd[1] = 155;
      ee.kp[2] = 27;  //
      ee.kd[2] = 180;
      ee.PID = 2;
      ee.V = 40;
      ee.Vmin = -70, ee.Vmax = 100;
      ee.Ts = 5;
      ee.PLAN = 0;
      ee.I = 0;
      ee.CPI = 0;
      ee.LINE = BLK;
      ee.sensitivity = 0;
      ee.SV1[0] = 90;
      ee.SV2[0] = 90;
      ee.SV1[1] = 90;
      ee.SV2[1] = 90;
      ee.Dly = 500;
      lcd.draw1x2String(0, 2, " FORMATTING...  ");
      eeWrite();
      delay(500);
      eeRead();
      lcd.clear();
    }
  }
  servoR.attach(PA6);
  servoL.attach(PA7);
  servoR.write(ee.SV1[0]);
  servoL.write(ee.SV2[0]);
  lcd.setContrast(5);
}

void loop() {
  if (!OK) {
    switch (7);
    menu();
    choose(0);
    led_ON;
    idx = 4;
  }

  if (!BACK) {
    led_OFF;
    lcd.clear();
    while (!BACK) {
      delay(100);
    }
    led_ON;
    lastError = 0;
    lastTime = 0;
    line = ee.LINE;
    int8 plan = ee.PLAN;
    int8 x = ee.CP[ee.CPI];
    int8 berhenti = off;
    uint32 count_now, stop_time;
    uint32 start_time = millis();
    if (ee.Ta[ee.CPI] > 0) {
      count_now = millis();
      while (millis() - count_now < (ee.Ta[ee.CPI] * 50)) {
        flc(ee.Va[ee.CPI], ee.PID);
      }
    }
    while (1) {
      int8 get_index = 0;
      sensor_list(plan, x);
      uint16 sensor_1 = 0, sensor_2 = 0;
      switch (sensor_logic) {
        case NONE:
          flc(ee.V, ee.PID);
          get_index = 0;
          break;
        case DIRECT:
          if (ee.I == 0) {
            flc(ee.V, ee.PID);
          }
          get_index = 1;
          break;
        case EQ:
          flc(ee.V, ee.PID);
          bitSensor = readSensor();
          //          if (line == BLK) {
          //            bitSensor = bitSensor;
          //          }
          //          else {
          //            bitSensor = ~bitSensor;
          //          }
          if (bitSensor == sensor1) {
            get_index = 1;
          }
          else {
            flc(ee.V, ee.PID);
          }
          break;
        case OR:
          flc(ee.V, ee.PID);
          bitSensor = readSensor();
          //          if (line == BLK) {
          //            bitSensor = bitSensor;
          //          }
          //          else {
          //            bitSensor = ~bitSensor;
          //          }
          sensor_1 = bitSensor & sensor1;
          if (sensor_1 > 0) {
            get_index = 1;
          }
          else {
            flc(ee.V, ee.PID);
          }
          break;
        case XOR:
          flc(ee.V, ee.PID);
          bitSensor = readSensor();
          //          if (line == BLK) {
          //            bitSensor = bitSensor;
          //          }
          //          else {
          //            bitSensor = ~bitSensor;
          //          }
          sensor_1 = bitSensor & sensor1;
          sensor_2 = bitSensor & sensor2;
          if (sensor_1 > 0 && sensor_2 > 0) {
            get_index = 1;
          }
          else {
            flc(ee.V, ee.PID);
          }
          break;
      }

      if (get_index == 1) {
        setMotor(0, 0);
        led_OFF;
        int VL = map(ee.LSPEED[plan][x], -100, 100, -65535, 65535);
        int VR = map(ee.RSPEED[plan][x], -100, 100, -65535, 65535);
        setMotor(VL, VR);
        switch (ee.ACTION[plan][x]) {
          case PICK: servoL.write(ee.SV2[0]); delay(ee.Dly); servoR.write(ee.SV1[0]); break;
          case DROP: servoR.write(ee.SV1[1]); delay(ee.Dly); servoL.write(ee.SV2[1]); break;
        }
        switch (ee.COUNTERMODE[plan][x]) {
          case TIMER:
            count_now = millis();
            while (millis() - count_now < ee.COUNTER[plan][x]) {}
            break;
          case ENCODERR:
            rPos = 0;
            lPos = 0;
            while (RPOS < ee.COUNTER[plan][x]) {}
            break;
          case ENCODERL:
            rPos = 0;
            lPos = 0;
            while (LPOS < ee.COUNTER[plan][x]) {}
            break;
        }
        setMotor(0, 0);
        lastError = 0;
        switch (ee.ACTION[plan][x]) {
          case BLK: line = BLK; break;
          case WHT: line = WHT; break;
        }

        if (ee.COUNTA[plan][x] > 0) {
          led_ON;
          //bitSensor = 0;
          bitSensor = readSensor();
          //          switch (line) {
          //            case BLK: bitSensor = bitSensor; break;
          //            case WHT: bitSensor = ~bitSensor; break;
          //          }
          if (bitSensor == 0b00000000000000) {
            while (1) {
              setMotor(VL / 8, VR / 8);
              bitSensor = readSensor();
              //              switch (line) {
              //                case BLK: bitSensor = bitSensor; break;
              //                case WHT: bitSensor = ~bitSensor; break;
              //              }
              if (bitSensor != 0b00000000000000) {
                break;
              }
            }
          }

          switch (ee.COUNTERAMODE[plan][x]) {
            case TIMER:
              count_now = millis();
              if (ee.FOLLOWMODE[plan][x] == FLR) {
                while (millis() - count_now < (ee.COUNTA[plan][x] * 50)) {
                  flr(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              else if (ee.FOLLOWMODE[plan][x] == FLL) {
                while (millis() - count_now < (ee.COUNTA[plan][x] * 50)) {
                  fll(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              else {
                while (millis() - count_now < (ee.COUNTA[plan][x] * 50)) {
                  flc(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              break;
            case ENCODERR:
              rPos = 0;
              lPos = 0;
              if (ee.FOLLOWMODE[plan][x] == FLR) {
                while (RPOS < ee.COUNTA[plan][x]) {
                  flr(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              else if (ee.FOLLOWMODE[plan][x] == FLL) {
                while (RPOS < ee.COUNTA[plan][x]) {
                  fll(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              else {
                while (RPOS < ee.COUNTA[plan][x]) {
                  flc(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              break;
            case ENCODERL:
              rPos = 0;
              lPos = 0;
              if (ee.FOLLOWMODE[plan][x] == FLR) {
                while (LPOS < ee.COUNTA[plan][x]) {
                  flr(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              else if (ee.FOLLOWMODE[plan][x] == FLL) {
                while (LPOS < ee.COUNTA[plan][x]) {
                  fll(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              else {
                while (LPOS < ee.COUNTA[plan][x]) {
                  flc(ee.VA[plan][x], ee.pid[plan][x]);
                  if (!BACK) break;
                }
              }
              break;
          }

          if (ee.COUNTB[plan][x] > 0) {
            switch (ee.COUNTERAMODE[plan][x]) {
              case TIMER:
                count_now = millis();
                if (ee.FOLLOWMODE[plan][x] == FLR) {
                  while (millis() - count_now < (ee.COUNTB[plan][x] * 50)) {
                    flr(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                else if (ee.FOLLOWMODE[plan][x] == FLL) {
                  while (millis() - count_now < (ee.COUNTB[plan][x] * 50)) {
                    fll(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                else {
                  while (millis() - count_now < (ee.COUNTB[plan][x] * 50)) {
                    flc(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                break;
              case ENCODERR:
                rPos = 0;
                lPos = 0;
                if (ee.FOLLOWMODE[plan][x] == FLR) {
                  while (RPOS < ee.COUNTB[plan][x]) {
                    flr(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                else if (ee.FOLLOWMODE[plan][x] == FLL) {
                  while (RPOS < ee.COUNTB[plan][x]) {
                    fll(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                else {
                  while (RPOS < ee.COUNTB[plan][x]) {
                    flc(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                break;
              case ENCODERL:
                rPos = 0;
                lPos = 0;
                if (ee.FOLLOWMODE[plan][x] == FLR) {
                  while (LPOS < ee.COUNTB[plan][x]) {
                    flr(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                else if (ee.FOLLOWMODE[plan][x] == FLL) {
                  while (LPOS < ee.COUNTB[plan][x]) {
                    fll(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                else {
                  while (LPOS < ee.COUNTB[plan][x]) {
                    flc(ee.VB[plan][x], ee.pid[plan][x]);
                    if (!BACK) break;
                  }
                }
                break;
            }
          }
        }

        if (ee.JUMP[plan][x] == on) {           //pindah plan & index
          int8 PNOW = plan;
          int8 INOW = x;
          plan = ee.jumpP[PNOW][INOW];
          x = ee.jumpI[PNOW][INOW];
        }
        else if (ee.STOP[plan] == x || x == 99 || !BACK) {          //berhenti
          setMotor(0, 0);
          led_OFF;
          break;
        }
        else if (ee.JUMP[plan][x] == off) {     //plan sekarang & index selanjutnya
          x++;
        }
      }
      else {
        flc(ee.V, ee.PID);
      }
      if (!BACK) break;
    }
    setMotor(0, 0);
    led_OFF;
    stop_time = millis();
    uint32 lap = (stop_time - start_time) / 10;
    float lap_f = float(lap) / 100.0;
    choose(1); lcd.drawString(12, 7, "Home"); choose(0);
    lcd.drawString(0, 3, "Time: ");
    lcd.setCursor(6, 3); lcd.print(lap_f);
    lcd.drawString(12, 3, " sec");
    eeWrite();
    led_DIM();
    //uint32 prigroup = SCB_BASE->AIRCR & SCB_AIRCR_PRIGROUP;
    //SCB_BASE->AIRCR = SCB_AIRCR_VECTKEY | prigroup;
    //nvic_sys_reset();
  }
  standby();
}

void choose(uint8 inverse) {
  lcd.setInverseFont(inverse);
}

void flc(uint16 SPEED, int8 pd) {
  led_ON;
  bitSensor = readSensor();
  //  switch (line) {
  //    case BLK: bitSensor = bitSensor; break;
  //    case WHT: bitSensor = ~bitSensor; break;
  //  }
  uint32 NOW = millis();
  uint32 interval = NOW - lastTime;
  if (interval >= ee.Ts) {
    switch (bitSensor) {
      case 0b00000000000001: error = -26; break;
      case 0b00000000000011: error = -24; break;

      case 0b00000000000111: error = -22; break;
      case 0b00000000000010: error = -22; break;

      //case 0b00000000001111: error = -20; break;
      case 0b00000000000110: error = -20; break;

      //case 0b00000000011111: error = -18; break;
      case 0b00000000001110: error = -18; break;
      case 0b00000000000100: error = -18; break;

      //case 0b00000000011110: error = -16; break;
      case 0b00000000001100: error = -16; break;

      //case 0b00000000111110: error = -14; break;
      case 0b00000000011100: error = -14; break;
      case 0b00000000001000: error = -14; break;

      //case 0b00000000111100: error = -12; break;
      case 0b00000000011000: error = -12; break;

      //case 0b00000001111100: error = -10; break;
      case 0b00000000111000: error = -10; break;
      case 0b00000000010000: error = -10; break;

      //case 0b00000001111000: error = -8; break;
      case 0b00000000110000: error = -8; break;

      //case 0b00000011111000: error = -5; break;
      case 0b00000001110000: error = -5; break;
      case 0b00000000100000: error = -5; break;

      //case 0b00000011110000: error = -3; break;
      case 0b00000001100000: error = -3; break;

      //case 0b00000111110000: error = -1; break;
      case 0b00000011100000: error = -1; break;
      case 0b00000001000000: error = -1; break;

      //perempatan
      //case 0b10000011000001: error = 0; break;
      //case 0b10000001100001: error = -3; break;
      //case 0b10000110000001: error = 3; break;
      //case 0b10000111000001: error = 1; break;
      //case 0b10000011100001: error = -1; break;
      //case 0b11000011000011: error = 0; break;

      //case 0b00100011000100: error = 0; break;
      //case 0b01000011000010: error = 0; break;
      //case 0b01100011000110: error = 0; break;
      //case 0b01000011000110: error = 0; break;
      //case 0b00100011000110: error = 0; break;
      //case 0b01100011000010: error = 0; break;
      //case 0b01100011000100: error = 0; break;
      //======================================
      //pertigaan kanan
      //case 0b00000011000011: error = 0; break;
      //case 0b00000011100011: error = 0; break;
      //case 0b00000011100110: error = 0; break;
      //case 0b00000011100111: error = 0; break;
      //case 0b00000011111110: error = 0; break;
      //case 0b00000011111100: error = 0; break;
      //case 0b00000111111000: error = 0; break;
      //======================================
      //pertigaan kiri
      //      case 0b11000011000000: error = 0; break;
      //      case 0b11000111000000: error = 0; break;
      //      case 0b01100111000000: error = 0; break;
      //      case 0b11100111000000: error = 0; break;
      //case 0b01111111000000: error = 0; break;
      //case 0b00111111000000: error = 0; break;
      //case 0b00011111100000: error = 0; break;
      //======================================

      case 0b00000111100000: error = 0; break;
      case 0b00001111110000: error = 0; break;
      case 0b00011111111000: error = 0; break;
      case 0b00111111111100: error = 0; break;
      case 0b01111111111110: error = 0; break;

      case 0b00000010000000: error = 1; break;
      case 0b00000111000000: error = 1; break;
      //case 0b00001111100000: error = 1; break;

      case 0b00000110000000: error = 3; break;
      //case 0b00001111000000: error = 3; break;

      case 0b00000100000000: error = 5; break;
      case 0b00001110000000: error = 5; break;
      //case 0b00011111000000: error = 5; break;

      case 0b00001100000000: error = 8; break;
      //case 0b00011110000000: error = 8; break;

      case 0b00001000000000: error = 10; break;
      case 0b00011100000000: error = 10; break;
      //case 0b00111110000000: error = 10; break;

      case 0b00011000000000: error = 12; break;
      //case 0b00111100000000: error = 12; break;

      case 0b00010000000000: error = 14; break;
      case 0b00111000000000: error = 14; break;
      //case 0b01111100000000: error = 14; break;

      case 0b00110000000000: error = 16; break;
      //case 0b01111000000000: error = 16; break;

      case 0b00100000000000: error = 18; break;
      case 0b01110000000000: error = 18; break;
      //case 0b11111000000000: error = 18; break;

      case 0b01100000000000: error = 20; break;
      //case 0b11110000000000: error = 20; break;

      case 0b01000000000000: error = 22; break;
      case 0b11100000000000: error = 22; break;

      case 0b11000000000000: error = 24; break;
      case 0b10000000000000: error = 26; break;
    }
    int16 rateError = error - lastError;
    lastError = error;

    int32 moveVal = int(((error * ee.kp[pd]) + (rateError * ee.kd[pd])) * 100);
    SPEED = map(SPEED, 0, 100, 0, 65535);
    int32 moveLeft = SPEED - moveVal;
    int32 moveRight = SPEED + moveVal;

    int VMAX = map(ee.Vmax, -100, 100, -65535, 65535);
    int VMIN = map(ee.Vmin, -100, 100, -65535, 65535);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

void flr(uint16 SPEED, int8 pd) {
  led_ON;
  bitSensor = readSensor();
  //  switch (line) {
  //    case BLK: bitSensor = bitSensor; break;
  //    case WHT: bitSensor = ~bitSensor; break;
  //  }

  uint32 NOW = millis();
  uint32 interval = NOW - lastTime;
  if (interval >= ee.Ts) {
    switch (bitSensor) {
      case 0b00000000000001: error = -26; break;
      case 0b00000000000011: error = -24; break;

      case 0b00000000000111: error = -22; break;
      case 0b00000000000010: error = -22; break;

      //case 0b00000000001111: error = -20; break;
      case 0b00000000000110: error = -20; break;

      //case 0b00000000011111: error = -18; break;
      case 0b00000000001110: error = -18; break;
      case 0b00000000000100: error = -18; break;

      //case 0b00000000011110: error = -16; break;
      case 0b00000000001100: error = -16; break;

      //case 0b00000000111110: error = -14; break;
      case 0b00000000011100: error = -14; break;
      case 0b00000000001000: error = -14; break;

      //case 0b00000000111100: error = -12; break;
      case 0b00000000011000: error = -12; break;

      //case 0b00000001111100: error = -10; break;
      case 0b00000000111000: error = -10; break;
      case 0b00000000010000: error = -10; break;

      //case 0b00000001111000: error = -8; break;
      case 0b00000000110000: error = -8; break;

      //case 0b00000011111000: error = -5; break;
      case 0b00000001110000: error = -5; break;
      case 0b00000000100000: error = -5; break;

      //case 0b00000011110000: error = -3; break;
      case 0b00000001100000: error = -3; break;

      //case 0b00000111110000: error = -1; break;
      case 0b00000011100000: error = -1; break;
      case 0b00000001000000: error = -1; break;

      //garis di kiri
      case 0b11111111111110: error = -24; break;
      case 0b11111111111100: error = -10; break;
      case 0b11111111111000: error = -8; break;
      case 0b11111111110000: error = -5; break;
      case 0b11111111100000: error = -3; break;
      case 0b11111111000000: error = -1; break;
      case 0b11111110000000: error = 0; break;
      case 0b11111100000000: error = 1; break;
      case 0b11111000000000: error = 3; break;
      case 0b11110000000000: error = 5; break;
      case 0b11100000000000: error = 10; break;
      case 0b11000000000000: error = 16; break;

      //perempatan
      case 0b10000011000001: error = -26; break;
      case 0b10000001100001: error = -26; break;
      case 0b10000110000001: error = -26; break;
      case 0b10000111000001: error = -26; break;
      case 0b10000011100001: error = -26; break;
      case 0b11000011000011: error = -26; break;

      case 0b00100011000100: error = -26; break;
      case 0b01000011000010: error = -26; break;
      case 0b01100011000110: error = -26; break;
      case 0b01000011000110: error = -26; break;
      case 0b00100011000110: error = -26; break;
      case 0b01100011000010: error = -26; break;
      case 0b01100011000100: error = -26; break;

      //case 0b00000000000000: error = -26; break;    //loss
      case 0b00001111110000: error = -26; break;
      case 0b00011111111000: error = -26; break;
      case 0b00111111111100: error = -26; break;
      case 0b11111111111111: error = -26; break;     //loss
      //======================================
      //pertigaan kanan
      case 0b00000001100001: error = -26; break;
      case 0b00000011100001: error = -26; break;
      case 0b00000011000001: error = -26; break;
      case 0b00000111000001: error = -26; break;
      case 0b00000110000001: error = -26; break;
      case 0b00001110000001: error = -26; break;
      case 0b00001100000001: error = -26; break;
      case 0b00011100000001: error = -26; break;
      case 0b00011000000001: error = -26; break;
      case 0b00111000000001: error = -26; break;
      case 0b00110000000001: error = -26; break;
      case 0b00100000000001: error = -26; break;
      case 0b01110000000001: error = -26; break;
      case 0b01100000000001: error = -26; break;
      case 0b01000000000001: error = -26; break;
      case 0b11100000000001: error = -26; break;
      case 0b11000000000001: error = -26; break;
      case 0b10000000000001: error = -26; break;

      case 0b00000011000011: error = -26; break;
      case 0b00000011100011: error = -26; break;
      case 0b00000011100110: error = -26; break;
      case 0b00000011100111: error = -26; break;
      case 0b00000011111110: error = -26; break;
      case 0b00000011111100: error = -26; break;
      case 0b00000111111000: error = -26; break;
      //========================================

      //pertigaan kiri
      case 0b11000011000000: error = 0; break;
      case 0b11000111000000: error = 0; break;
      case 0b01100111000000: error = 0; break;
      case 0b11100111000000: error = 0; break;
      case 0b01111111000000: error = 0; break;
      case 0b00111111000000: error = 0; break;
      case 0b00011111000000: error = 0; break;
      case 0b00011111100000: error = 0; break;
      case 0b00001111100000: error = 0; break;
      //========================================

      case 0b00000011000000: error = 0; break;
      case 0b00000111100000: error = 0; break;

      case 0b00000010000000: error = 1; break;
      case 0b00000111000000: error = 1; break;
      //case 0b00001111100000: error = 1; break;

      case 0b00000110000000: error = 3; break;
      //case 0b00001111000000: error = 3; break;

      case 0b00000100000000: error = 5; break;
      case 0b00001110000000: error = 5; break;
      //case 0b00011111000000: error = 5; break;

      case 0b00001100000000: error = 8; break;
      //case 0b00011110000000: error = 8; break;

      case 0b00001000000000: error = 10; break;
      case 0b00011100000000: error = 10; break;
      //case 0b00111110000000: error = 10; break;

      case 0b00011000000000: error = 12; break;
      //case 0b00111100000000: error = 12; break;

      case 0b00010000000000: error = 14; break;
      case 0b00111000000000: error = 14; break;
      //case 0b01111100000000: error = 14; break;

      case 0b00110000000000: error = 16; break;
      //case 0b01111000000000: error = 16; break;

      case 0b00100000000000: error = 18; break;
      case 0b01110000000000: error = 18; break;
      //case 0b11111000000000: error = 18; break;

      case 0b01100000000000: error = 20; break;
      //case 0b11110000000000: error = 20; break;

      case 0b01000000000000: error = 22; break;
      //case 0b11100000000000: error = 22; break;

      //case 0b11000000000000: error = 24; break;
      case 0b10000000000000: error = 26; break;
    }
    int16 rateError = error - lastError;
    lastError = error;

    int32 moveVal = int(((error * ee.kp[pd]) + (rateError * ee.kd[pd])) * 100);
    SPEED = map(SPEED, 0, 100, 0, 65535);
    int32 moveLeft = SPEED - moveVal;
    int32 moveRight = SPEED + moveVal;

    int VMAX = map(ee.Vmax, -100, 100, -65535, 65535);
    int VMIN = map(ee.Vmin, -100, 100, -65535, 65535);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

void fll(uint16 SPEED, int8 pd) {
  led_ON;
  bitSensor = readSensor();
  //  switch (line) {
  //    case BLK: bitSensor = bitSensor; break;
  //    case WHT: bitSensor = ~bitSensor; break;
  //  }

  uint32 NOW = millis();
  uint32 interval = NOW - lastTime;
  if (interval >= ee.Ts) {
    switch (bitSensor) {
      case 0b00000000000001: error = -26; break;
      //case 0b00000000000011: error = -24; break;

      //case 0b00000000000111: error = -22; break;
      case 0b00000000000010: error = -22; break;

      //case 0b00000000001111: error = -20; break;
      case 0b00000000000110: error = -20; break;

      //case 0b00000000011111: error = -18; break;
      case 0b00000000001110: error = -18; break;
      case 0b00000000000100: error = -18; break;

      //case 0b00000000011110: error = -16; break;
      case 0b00000000001100: error = -16; break;

      //case 0b00000000111110: error = -14; break;
      case 0b00000000011100: error = -14; break;
      case 0b00000000001000: error = -14; break;

      //case 0b00000000111100: error = -12; break;
      case 0b00000000011000: error = -12; break;

      //case 0b00000001111100: error = -10; break;
      case 0b00000000111000: error = -10; break;
      case 0b00000000010000: error = -10; break;

      //case 0b00000001111000: error = -8; break;
      case 0b00000000110000: error = -8; break;

      //case 0b00000011111000: error = -5; break;
      case 0b00000001110000: error = -5; break;
      case 0b00000000100000: error = -5; break;

      //case 0b00000011110000: error = -3; break;
      case 0b00000001100000: error = -3; break;

      //case 0b00000111110000: error = -1; break;
      case 0b00000011100000: error = -1; break;
      case 0b00000001000000: error = -1; break;

      //garis di kanan
      case 0b01111111111111: error = 24; break;
      case 0b00111111111111: error = 10; break;
      case 0b00011111111111: error = 8; break;
      case 0b00001111111111: error = 5; break;
      case 0b00000111111111: error = 3; break;
      case 0b00000011111111: error = 1; break;
      case 0b00000001111111: error = 0; break;
      case 0b00000000111111: error = -1; break;
      case 0b00000000011111: error = -3; break;
      case 0b00000000001111: error = -5; break;
      case 0b00000000000111: error = -10; break;
      case 0b00000000000011: error = -16; break;

      //perempatan
      case 0b10000011000001: error = 26; break;
      case 0b10000001100001: error = 26; break;
      case 0b10000110000001: error = 26; break;
      case 0b10000111000001: error = 26; break;
      case 0b10000011100001: error = 26; break;
      case 0b11000011000011: error = 26; break;

      case 0b00100011000100: error = 26; break;
      case 0b01000011000010: error = 26; break;
      case 0b01100011000110: error = 26; break;
      case 0b01000011000110: error = 26; break;
      case 0b00100011000110: error = 26; break;
      case 0b01100011000010: error = 26; break;
      case 0b01100011000100: error = 26; break;

      //case 0b00000000000000: error = 26; break;    //loss
      case 0b00001111110000: error = 26; break;
      case 0b00011111111000: error = 26; break;
      case 0b00111111111100: error = 26; break;
      case 0b11111111111111: error = 26; break;     //loss
      //======================================
      //pertigaan kiri
      case 0b10000110000000: error = 26; break;
      case 0b10000111000000: error = 26; break;
      case 0b10000011000000: error = 26; break;
      case 0b10000011100000: error = 26; break;
      case 0b10000001100000: error = 26; break;
      case 0b10000001110000: error = 26; break;
      case 0b10000000110000: error = 26; break;
      case 0b10000000111000: error = 26; break;
      case 0b10000000011000: error = 26; break;
      case 0b10000000011100: error = 26; break;
      case 0b10000000001100: error = 26; break;
      case 0b10000000001110: error = 26; break;
      case 0b10000000000110: error = 26; break;
      case 0b10000000000111: error = 26; break;
      case 0b10000000000011: error = 26; break;
      case 0b10000000000001: error = 26; break;

      case 0b11000011000000: error = 26; break;
      case 0b11000111000000: error = 26; break;
      case 0b01100111000000: error = 26; break;
      case 0b11100111000000: error = 26; break;
      case 0b11111111000000: error = 26; break;
      case 0b00111111000000: error = 26; break;
      case 0b00011111100000: error = 26; break;
      //========================================

      //pertigaan kanan
      case 0b00000011000011: error = 0; break;
      case 0b00000011100011: error = 0; break;
      case 0b00000011100110: error = 0; break;
      case 0b00000011100111: error = 0; break;
      case 0b00000011111110: error = 0; break;
      case 0b00000011111100: error = 0; break;
      case 0b00000111111000: error = 0; break;
      case 0b00000111110000: error = 0; break;
      //========================================

      case 0b00000011000000: error = 0; break;
      case 0b00000111100000: error = 0; break;

      case 0b00000010000000: error = 1; break;
      case 0b00000111000000: error = 1; break;
      //case 0b00001111100000: error = 1; break;

      case 0b00000110000000: error = 3; break;
      //case 0b00001111000000: error = 3; break;

      case 0b00000100000000: error = 5; break;
      case 0b00001110000000: error = 5; break;
      //case 0b00011111000000: error = 5; break;

      case 0b00001100000000: error = 8; break;
      //case 0b00011110000000: error = 8; break;

      case 0b00001000000000: error = 10; break;
      case 0b00011100000000: error = 10; break;
      //case 0b00111110000000: error = 10; break;

      case 0b00011000000000: error = 12; break;
      //case 0b00111100000000: error = 12; break;

      case 0b00010000000000: error = 14; break;
      case 0b00111000000000: error = 14; break;
      //case 0b01111100000000: error = 14; break;

      case 0b00110000000000: error = 16; break;
      //case 0b01111000000000: error = 16; break;

      case 0b00100000000000: error = 18; break;
      case 0b01110000000000: error = 18; break;
      //case 0b11111000000000: error = 18; break;

      case 0b01100000000000: error = 20; break;
      //case 0b11110000000000: error = 20; break;

      case 0b01000000000000: error = 22; break;
      case 0b11100000000000: error = 22; break;

      case 0b11000000000000: error = 24; break;
      case 0b10000000000000: error = 26; break;
    }
    int16 rateError = error - lastError;
    lastError = error;

    int32 moveVal = int(((error * ee.kp[pd]) + (rateError * ee.kd[pd])) * 100);
    SPEED = map(SPEED, 0, 100, 0, 65535);
    int32 moveLeft = SPEED - moveVal;
    int32 moveRight = SPEED + moveVal;

    int VMAX = map(ee.Vmax, -100, 100, -65535, 65535);
    int VMIN = map(ee.Vmin, -100, 100, -65535, 65535);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

void sensor_list(int8 plan, int8 x) {
  switch (ee.SENSOR[plan][x]) {
    case 0: sensor_logic = NONE; break;

    case 1: sensor_logic = DIRECT; break;

    case 2: sensor1 = 0b00000000000000; sensor_logic = EQ; break;
    case 3: sensor1 = 0b11111111111111; sensor_logic = EQ; break;

    case 4: sensor1 = 0b00000000000001; sensor_logic = OR; break;
    case 5: sensor1 = 0b00000000000011; sensor_logic = OR; break;
    case 6: sensor1 = 0b00000000000111; sensor_logic = OR; break;
    case 7: sensor1 = 0b00000000001111; sensor_logic = OR; break;
    case 8: sensor1 = 0b10000000000000; sensor_logic = OR; break;
    case 9: sensor1 = 0b11000000000000; sensor_logic = OR; break;
    case 10: sensor1 = 0b11100000000000; sensor_logic = OR; break;
    case 11: sensor1 = 0b11110000000000; sensor_logic = OR; break;
    case 12: sensor1 = 0b11111111111111; sensor_logic = OR; break;

    case 13: sensor1 = 0b00000111100000; sensor2 = 0b00000000000001; sensor_logic = XOR; break;
    case 14: sensor1 = 0b00000111100000; sensor2 = 0b00000000000011; sensor_logic = XOR; break;
    case 15: sensor1 = 0b00000111100000; sensor2 = 0b00000000000111; sensor_logic = XOR; break;
    case 16: sensor1 = 0b00000111100000; sensor2 = 0b00000000001111; sensor_logic = XOR; break;
    case 17: sensor1 = 0b00000111100000; sensor2 = 0b10000000000000; sensor_logic = XOR; break;
    case 18: sensor1 = 0b00000111100000; sensor2 = 0b11000000000000; sensor_logic = XOR; break;
    case 19: sensor1 = 0b00000111100000; sensor2 = 0b11100000000000; sensor_logic = XOR; break;
    case 20: sensor1 = 0b00000111100000; sensor2 = 0b11110000000000; sensor_logic = XOR; break;
    case 21: sensor1 = 0b10000000000000; sensor2 = 0b00000000000001; sensor_logic = XOR; break;
    case 22: sensor1 = 0b11000000000000; sensor2 = 0b00000000000011; sensor_logic = XOR; break;
    case 23: sensor1 = 0b11100000000000; sensor2 = 0b00000000000111; sensor_logic = XOR; break;
    case 24: sensor1 = 0b11110000000000; sensor2 = 0b00000000001111; sensor_logic = XOR; break;
    case 25: sensor1 = 0b11111000000000; sensor2 = 0b00000000011111; sensor_logic = XOR; break;
    case 26: sensor1 = 0b11111100000000; sensor2 = 0b00000000111111; sensor_logic = XOR; break;
  }
}

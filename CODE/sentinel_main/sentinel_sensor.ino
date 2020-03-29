void led_DIM() {
  while (1) {
    pinMode(led, PWM);
    for (int j = 65535; j > 0; j = j - 100) {
      pwmWrite(led, j);
      if (!BACK) {
        break;
      }
      delay(1);
    }
    for (int j = 0; j < 65535; j = j + 100) {
      pwmWrite(led, j);
      if (!BACK) {
        break;
      }
      delay(1);
    }
    if (!BACK) {
      pinMode(led, OUTPUT);
      led_OFF;
      lcd.clear();
      while (!BACK) {
        delay(50);
      }
      break;
    }
  }
}

uint16 readSensor() {
  if (line == BLK) {
    for (int8 i = 0; i <= 6; i++) {
      switchSensor(i);
      delayMicroseconds(10);
      arraySensor[i] = analogRead(sensorR);//adcSensor[0];
      if (arraySensor[i] > ee.sensorRef[i]) {
        bitSensor = bitSensor | (0b00000000000001 << i);
      }
      else {
        bitClear(bitSensor, i);
      }
      arraySensor[13 - i] = analogRead(sensorL);//adcSensor[1];
      if (arraySensor[13 - i] > ee.sensorRef[13 - i]) {
        bitSensor = bitSensor | (0b00000000000001 << (13 - i));
      }
      else {
        bitClear(bitSensor, 13 - i);
      }
    }
  }
  else {
    for (int8 i = 0; i <= 6; i++) {
      switchSensor(i);
      delayMicroseconds(10);
      arraySensor[i] = analogRead(sensorR);//adcSensor[0];
      if (arraySensor[i] > ee.sensorRef[i]) {
        bitClear(bitSensor, i);
      }
      else {
        bitSensor = bitSensor | (0b00000000000001 << i);
      }
      arraySensor[13 - i] = analogRead(sensorL);//adcSensor[1];
      if (arraySensor[13 - i] > ee.sensorRef[13 - i]) {
        bitClear(bitSensor, 13 - i);
      }
      else {
        bitSensor = bitSensor | (0b00000000000001 << (13 - i));
      }
    }
  }
  return bitSensor;
}

void switchSensor(uint8 muxSensor) {
  gpio_write_bit(GPIOC, 13, (muxSensor & 1) ? HIGH : LOW);
  gpio_write_bit(GPIOC, 14, (muxSensor & 2) ? HIGH : LOW);
  gpio_write_bit(GPIOC, 15, (muxSensor & 4) ? HIGH : LOW);
}

void calibration() {
  led_ON;
  uint8 i;
  int16 lRef[14], hRef[14];
  for (int8 i = 0; i <= 13; i++) {
    lRef[i] = 4095;
    hRef[i] = 0;
  }
  lcd.clear();
  choose(0); lcd.drawString(0, 3, " Calibrating... "); lcd.drawString(0, 4, "Sensitivity:"); lcd.drawString(15, 4, "%");
  choose(1); lcd.drawString(0, 7, "Save"); lcd.drawString(10, 7, "Cancel");
  delay(200);
  while (1) {
    for (int8 i = 0; i <= 6; i++) {
      switchSensor(i);
      delayMicroseconds(10);
      arraySensor[i] = analogRead(sensorR);//adcSensor[0];
      if (arraySensor[i] > hRef[i]) {
        hRef[i] = arraySensor[i];
      }
      if (arraySensor[i] < lRef[i]) {
        lRef[i] = arraySensor[i];
      }
      arraySensor[13 - i] = analogRead(sensorL);//adcSensor[1];
      if (arraySensor[13 - i] > hRef[13 - i]) {
        hRef[13 - i] = arraySensor[13 - i];
      }
      if (arraySensor[13 - i] < lRef[13 - i]) {
        lRef[13 - i] = arraySensor[13 - i];
      }
    }
    if (!PLUS) {
      if (++ee.sensitivity > 50) ee.sensitivity = 50;
      delay(100);
    }
    if (!MIN) {
      if (--ee.sensitivity < -50) ee.sensitivity = -50;
      delay(100);
    }
    sprintf(buff, "%3d", ee.sensitivity);
    lcd.drawString(12, 4, buff);
    if (!BACK) {
      led_OFF;
      lcd.clear();
      choose(0);
      lcd.draw1x2String(0, 3, "    CANCELED    ");
      goto cancel;
    }
    if (!OK) {
      led_OFF;
      break;
    }
  }
  for (int8 i = 0; i <= 13; i++) {
    ee.sensorRef[i] = (hRef[i] - lRef[i]) / 2 + lRef[i];
    ee.sensorRef[i] = ee.sensorRef[i] - (ee.sensorRef[i] * ee.sensitivity / 100);
  }
  lcd.clear();
  choose(0);
  lcd.draw1x2String(0, 3, "      DONE      ");
cancel:
  switch (7);
  delay(500);
  lcd.clear();
}

void readSensorAdc() {
  for (int8 i = 0; i <= 6; i++) {
    switchSensor(i);
    delayMicroseconds(10);
    arraySensor[i] = analogRead(sensorR);//adcSensor[0];
    arraySensor[13 - i] = analogRead(sensorL);//adcSensor[1];
  }
}

void initSensor() {
  pinMode(sensorL, INPUT_ANALOG);
  pinMode(sensorR, INPUT_ANALOG);
  pinMode(batt, INPUT_ANALOG);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);

  //  adc.calibrate();
  //  adc.setSampleRate(ADC_SMPR_1_5);
  //  adc.setPins(pinSensor, 3);
  //  adc.setScanMode();
  //  adc.setContinuous();
  //  adc.setDMA(adcSensor, 3, (DMA_MINC_MODE | DMA_CIRC_MODE), NULL);
  //  adc.startConversion();
}
